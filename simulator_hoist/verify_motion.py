"""Small real-map/real-OHT integration check; does not modify schedules."""
import sys, json, os
from pathlib import Path
from types import SimpleNamespace as NS
sys.argv=['agv_fab_sim.py','--analyze']
import agv_fab_sim as s
import salabim as sim
from hoisting import Hoisting, NODES

s.ENABLE_JUNCTION_REROUTE=False
s.ENABLE_RING_CONTROL=False
s.HOIST=Hoisting(int(os.environ.get('HOIST_MODE','4')),float(os.environ.get('HOIST_PROBABILITY','1')),8,42)
s.reset_sim_state(); sim.yieldless(False); env=sim.Environment(trace=False)
paths=[]
for node in NODES:
    before=[node]
    for _ in range(4):
        candidates=[a for a,b in s.EDGES if b==before[0]]
        before.insert(0,candidates[0])
    after=s.LINK_B[s.OUT_LINKS[node][0]]
    paths.append(before+[after])

completed=[]
class Probe(s.OHT):
    def process(self):
        path=paths[self.oht_id]
        self.job=NS(lot_id=self.oht_id+1,leg_index=0,job_id=self.oht_id+1,from_node=path[0],to_node=path[-1])
        self.state=s.OHT_TO_DEST
        self._cur_speed=0
        yield from self._travel_path(path)
        completed.append(dict(oht=self.oht_id,node=self.node,time=env.now(),hoist=self._job_hoist,speed=self._cur_speed))

class Check(sim.Component):
    def process(self):
        interrupted=set()
        for _ in range(1000):
            for v in s.OHT_LIST:
                if v.state=='hoisting':
                    assert s.NODE_HOLDER.get(v.node) is v
                    assert v._stall_start is None and v._stall_next is None
                    assert abs(v._cur_speed)<1e-5
                    if os.environ.get('VERIFY_INTERRUPT')=='1' and v.oht_id not in interrupted:
                        interrupted.add(v.oht_id)
                        v.activate()  # Simulate an external wakeup during service.
            yield self.hold(.05)

vehicles=[Probe(park_node=path[0]) for path in paths]
Check(); env.run(till=50)
assert len(completed)==3,completed
events=s.HOIST.summary(env.now())
expected=len(s.HOIST.active) if s.HOIST.probability==1 else 0
assert events['completed_events']==expected,events
assert all(abs(e['end_second']-e['start_second']-8)<1e-8 for e in events['events'] if e['selected'])
result=dict(completed=completed,hoisting=events)
out=Path(os.environ.get('VERIFY_OUTPUT','motion_check.json'))
out.write_text(json.dumps(result,indent=2),encoding='utf-8')
print('MOTION CHECK PASSED',out)
