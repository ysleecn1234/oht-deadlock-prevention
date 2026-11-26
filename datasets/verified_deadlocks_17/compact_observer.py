"""Lossless sampled-state deltas and conservative candidate observation.

No simulator methods which grant permissions or change control are called.
Candidates are observations, not deadlock labels. Only direct next-node holder
cycles are recognized; capacity/permission dependencies require later review.
"""
import gzip
import hashlib
import json
from pathlib import Path


def encoded(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=False, separators=(',', ':'))


def maps(state):
    value = dict(state)
    value['vehicles'] = {str(v['id']): v for v in state['vehicles']}
    return value


def difference(previous, current):
    changed, removed = {}, {}
    for key, value in current.items():
        old = previous.get(key)
        if isinstance(value, dict) and isinstance(old, dict):
            updates = {k: v for k, v in value.items() if k not in old or old[k] != v}
            deleted = sorted(set(old) - set(value))
            if updates:
                changed[key] = updates
            if deleted:
                removed[key] = deleted
        elif value != old or key not in previous:
            changed[key] = value
    return dict(kind='delta', changed=changed, removed=removed)


def apply_delta(previous, record):
    if record['kind'] == 'full':
        return record['state']
    value = dict(previous)
    for key in set(record['changed']) | set(record['removed']):
        update = record['changed'].get(key)
        if isinstance(previous.get(key), dict):
            item = dict(previous[key])
            item.update(update or {})
            for name in record['removed'].get(key, []):
                item.pop(name, None)
            value[key] = item
        else:
            value[key] = update
    return value


def read_states(path):
    state = None
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        for line in stream:
            record = json.loads(line)
            if record['kind'] == 'schema':
                continue
            state = apply_delta(state, record)
            yield state


def cycles(edges):
    visited, found = set(), set()
    for start in edges:
        path, indexes, node = [], {}, start
        while node in edges and node not in visited and node not in indexes:
            indexes[node] = len(path)
            path.append(node)
            node = edges[node]
        if node in indexes:
            group = tuple(sorted(path[indexes[node]:]))
            if len(group) > 1:
                found.add(group)
        visited.update(path)
    return found


class CandidateTracker:
    def __init__(self, threshold=120):
        self.threshold = threshold
        self.previous = {}
        self.active = {}
        self.cycle_since = {}
        self.counts = {'long_wait': 0, 'direct_node_cycle': 0,
                       'motion_observed': 0, 'dependency_lost': 0}
        self.last_time = None

    def observe(self, state):
        now, events, waiting = state['t'], [], {}
        fleet = {v['id']: v for v in state['vehicles']}
        gap = self.last_time is not None and now - self.last_time != 1
        if gap:
            for key in list(self.active):
                events.append(dict(kind='candidate_end', t=now, key=key,
                                   reason='observation_gap_not_resolution'))
            self.active.clear(); self.previous.clear(); self.cycle_since.clear()
        for oid, v in fleet.items():
            token = (v['node'], v['_seg_start_t'], tuple(v['xy']))
            blocked = (v['_stall_start'] is not None or v['_ring_waiting']
                       or v['_merge_wait'] is not None)
            old = self.previous.get(oid)
            since = old[1] if old and old[0] == token and old[2] and blocked else now
            self.previous[oid] = (token, since, blocked)
            if blocked:
                waiting[oid] = since
            key = f'wait:{oid}'
            if key in self.active and old and old[0] != token:
                events.append(dict(kind='candidate_end', t=now, key=key,
                                   reason='motion_observed', start=self.active.pop(key)))
                self.counts['motion_observed'] += 1
            if blocked and now - since >= self.threshold and key not in self.active:
                self.active[key] = since
                self.counts['long_wait'] += 1
                events.append(dict(kind='candidate_start', category='long_wait',
                                   key=key, t=now, start=since, members=[oid]))
        # This is a limited direct occupancy graph, not the complete wait-for graph.
        edges = {}
        for oid in waiting:
            v = fleet[oid]
            if v['_ring_waiting'] or v['_merge_wait'] is not None:
                continue
            holder = state['node_holders'].get(str(v['_stall_next']))
            if holder in waiting and holder != oid and now > waiting[oid] and now > waiting[holder]:
                edges[oid] = holder
        current = {tuple((i, edges[i]) for i in group) for group in cycles(edges)}
        for relation in list(self.cycle_since):
            if relation not in current:
                key = 'cycle:' + encoded(relation)
                if key in self.active:
                    events.append(dict(kind='candidate_end', t=now, key=key,
                        start=self.active.pop(key), reason='dependency_lost_not_proof_of_recovery'))
                    self.counts['dependency_lost'] += 1
                del self.cycle_since[relation]
        for relation in current:
            since = self.cycle_since.setdefault(relation, now)
            key = 'cycle:' + encoded(relation)
            if now - since >= self.threshold and key not in self.active:
                self.active[key] = since
                self.counts['direct_node_cycle'] += 1
                events.append(dict(kind='candidate_start', category='direct_node_cycle',
                    key=key, t=now, start=since, members=[i for i, _ in relation], edges=relation))
        self.last_time = now
        return events


class CompactRecorder:
    def __init__(self, folder, threshold=120):
        self.folder = Path(folder)
        self.stream = gzip.open(self.folder / 'states.jsonl.gz', 'wt', encoding='utf-8', compresslevel=3)
        self.events = (self.folder / 'candidates.jsonl').open('w', encoding='utf-8')
        self.previous = None
        self.hash = hashlib.sha256()
        self.samples = 0
        self.tracker = CandidateTracker(threshold)
        self.stream.write(encoded(dict(kind='schema', version=1, interval_seconds=1,
            full_every_seconds=300, vehicles='map keyed by id',
            warning='Not a restart checkpoint; candidates are not confirmed deadlocks')) + '\n')

    def record(self, state):
        current = maps(state)
        self.hash.update((encoded(current) + '\n').encode('utf-8'))
        record = (dict(kind='full', state=current) if self.previous is None or state['t'] % 300 == 0
                  else difference(self.previous, current))
        self.stream.write(encoded(record) + '\n')
        self.previous = current
        self.samples += 1
        for event in self.tracker.observe(state):
            event['classification'] = 'candidate_pending_review'
            self.events.write(encoded(event) + '\n')
        if state['t'] % 60 == 0:
            self.stream.flush(); self.events.flush()

    def close(self):
        self.stream.close(); self.events.close()
        report = dict(samples=self.samples, state_sha256=self.hash.hexdigest(),
            candidate_counts=self.tracker.counts, active_candidates=self.tracker.active,
            confirmed_deadlocks=None,
            limitations=['Direct next-node cycles only; no complete capacity/permission graph.',
                        'Missing dependency is not proof of natural recovery.',
                        'Unresolved candidates at end are censored, not negatives.',
                        'One-second sampling can miss changes between observations.'])
        (self.folder / 'observation_summary.json').write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
