"""Reproducible, loaded-trip-only hoisting stops at existing transit nodes."""
import hashlib
import json
import math
import os

NODES = ('017192', '016911', '017024')
MODE_NODES = {0: (), 1: NODES[:1], 2: NODES[1:2], 3: NODES[2:], 4: NODES}


class Hoisting:
    def __init__(self, mode=0, probability=0.1, seconds=8.0, seed=42):
        if mode not in MODE_NODES:
            raise ValueError('HOIST_MODE must be 0..4')
        if not math.isfinite(probability) or not 0 <= probability <= 1:
            raise ValueError('HOIST_PROBABILITY must be between 0 and 1')
        if not math.isfinite(seconds) or seconds <= 0:
            raise ValueError('HOIST_SECONDS must be positive and finite')
        self.mode, self.probability, self.seconds, self.seed = mode, probability, seconds, seed
        self.active = frozenset(MODE_NODES[mode])
        self.events = []

    @classmethod
    def from_env(cls):
        return cls(int(os.getenv('HOIST_MODE', '0')),
                   float(os.getenv('HOIST_PROBABILITY', '0.1')),
                   float(os.getenv('HOIST_SECONDS', '8')),
                   int(os.getenv('HOIST_SEED', '42')))

    def reset(self):
        self.events.clear()

    def decision(self, lot, leg, node, visit):
        # No mode, OHT number, process hash(), or shared random generator.
        key = [self.seed, lot, leg, node, visit]
        digest = hashlib.sha256(json.dumps(key, separators=(',', ':'), ensure_ascii=True).encode()).digest()
        score = int.from_bytes(digest[:8], 'big') / 2**64
        return score < self.probability, score

    def plan(self, vehicle, path):
        job = vehicle.job
        if not self.active or job is None or vehicle.state != 'to_dest':
            return {}
        # Loaded jobs cannot be reassigned. Keep counts across route replans;
        # only actual arrivals commit a visit, never a reservation/retry.
        if getattr(vehicle, '_hoist_job', None) is not job:
            vehicle._hoist_job = job
            vehicle._hoist_visits = {}
        counts = vehicle._hoist_visits.copy()
        plan = {}
        for index, node in enumerate(path[1:], 1):
            if node not in self.active or node in (job.from_node, job.to_node):
                continue
            counts[node] = counts.get(node, 0) + 1
            visit = counts[node]
            selected, score = self.decision(job.lot_id, job.leg_index, node, visit)
            plan[index] = dict(lot_id=job.lot_id, leg_index=job.leg_index,
                               job_id=job.job_id, node=node, visit=visit,
                               selected=selected, score=score)
        return plan

    @staticmethod
    def speed_limit(plan, path, index, edge_length, deceleration):
        distance = 0.0
        for j in range(index, len(path)):
            if plan.get(j, {}).get('selected'):
                return math.sqrt(2.0 * deceleration * distance)
            if j + 1 < len(path):
                distance += edge_length(path[j], path[j+1])
        return math.inf

    def arrive(self, vehicle, item):
        vehicle._hoist_visits[item['node']] = item['visit']
        event = dict(item, mode=self.mode, oht_id=vehicle.oht_id,
                     arrival_second=vehicle.env.now(), start_second=None,
                     end_second=None, planned_seconds=self.seconds if item['selected'] else 0,
                     status='planned' if item['selected'] else 'passed')
        self.events.append(event)
        if not item['selected']:
            return
        if abs(vehicle._cur_speed) > 1e-5:
            raise RuntimeError('Hoist arrival was not stopped by the motion profile')
        previous_state = vehicle.state
        vehicle.state = 'hoisting'
        vehicle._hoist_accounted_at = vehicle.env.now()
        event['start_second'] = vehicle.env.now()
        event['status'] = 'running'
        end = vehicle.env.now() + self.seconds
        # A replan notification must not shorten service time.
        while vehicle.env.now() < end - 1e-9:
            before = vehicle.env.now()
            yield vehicle.hold(end - before)
            vehicle._job_hoist += vehicle.env.now() - before
            vehicle._hoist_accounted_at = vehicle.env.now()
        event['end_second'] = vehicle.env.now()
        event['status'] = 'completed'
        vehicle.state = previous_state

    @staticmethod
    def active_seconds(vehicle):
        """Include a service still running when the simulation ends."""
        extra = (vehicle.env.now() - vehicle._hoist_accounted_at
                 if vehicle.state == 'hoisting' else 0.0)
        return getattr(vehicle, '_job_hoist', 0.0) + extra

    def summary(self, now):
        selected = [e for e in self.events if e['selected']]
        elapsed = sum(max(0, (e['end_second'] if e['end_second'] is not None else now)
                          - e['start_second']) for e in selected if e['start_second'] is not None)
        return dict(mode=self.mode, active_nodes=sorted(self.active), seed=self.seed,
                    probability=self.probability, seconds=self.seconds,
                    eligible_visits=len(self.events), event_count=len(selected),
                    completed_events=sum(e['status']=='completed' for e in selected),
                    elapsed_hoist_seconds=elapsed, events=self.events)


HOIST = Hoisting.from_env()
