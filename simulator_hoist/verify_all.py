"""Run fast reproducibility and mode-0 baseline checks in fresh processes."""
import json, os, subprocess, sys
from pathlib import Path
root=Path(__file__).resolve().parent
out=root/'verification'; out.mkdir(exist_ok=True)
def run(name, command, extra):
    env=os.environ.copy(); env.update(extra)
    result=subprocess.run([sys.executable,'-X','utf8',*command],cwd=root,env=env,capture_output=True,text=True,encoding='utf-8')
    (out/(name+'.log')).write_text(result.stdout+result.stderr,encoding='utf-8')
    if result.returncode: raise RuntimeError(name+': '+result.stderr[-2500:])
def motion(name,mode,**extra):
    run(name,['verify_motion.py'],dict(HOIST_MODE=str(mode),HOIST_PROBABILITY='1',VERIFY_OUTPUT=str(out/(name+'.json')),**extra))
    return json.loads((out/(name+'.json')).read_text())
for mode in range(5): motion('mode'+str(mode),mode)
a=motion('repeat4',4)
assert a==json.loads((out/'mode4.json').read_text())
motion('interrupt4',4,VERIFY_INTERRUPT='1')
for mode in range(1,4):
    single=json.loads((out/('mode'+str(mode)+'.json')).read_text())['hoisting']['events'][0]
    multi=next(e for e in a['hoisting']['events'] if e['node']==single['node'])
    for k in ('score','visit','start_second','end_second'): assert single[k]==multi[k],(mode,k)
# Fresh baseline process imports the original snapshot without altering either copy.
baseline=out/'baseline_f71735e.py'
snapshot=subprocess.run(['git','show','f71735e4871edede5cab82e3b3506b9a003b8439:dnd_work/agv_fab_sim.py'],
                        cwd=root.parent,capture_output=True,check=True)
baseline.write_bytes(snapshot.stdout)
code="import sys,importlib.util,runpy; sys.argv=['agv_fab_sim.py','--analyze']; spec=importlib.util.spec_from_file_location('agv_fab_sim',"+repr(str(baseline))+ "); m=importlib.util.module_from_spec(spec); sys.modules['agv_fab_sim']=m; spec.loader.exec_module(m); runpy.run_path('deadlock_detector.py',run_name='__main__')"
common=dict(HOIST_MODE='0',OHT_COUNT='20',SIMULATION_HOURS=str(120/3600),STOP_AFTER_SECONDS='0',STOP_WHEN_ALL_LOTS_DONE='0')
# The modified detector expects metadata from the hoisting module, even with the original simulator.
code=code.replace("runpy.run_path", "from hoisting import HOIST; m.HOIST=HOIST; runpy.run_path")
# Original vehicles have no new counter; for this baseline the detector's metric helper must tolerate it.
run('baseline',['-c',code],dict(common,RESULT_JSON_PATH=str(out/'baseline.json')))
run('off',['deadlock_detector.py'],dict(common,RESULT_JSON_PATH=str(out/'off.json')))
b=json.loads((out/'baseline.json').read_text()); c=json.loads((out/'off.json').read_text())
for key in ('wall_seconds','realtime_factor'): b.pop(key); c.pop(key)
assert b==c, [k for k in b if b[k]!=c[k]]
(out/'PASS.txt').write_text('PASS: modes 0..4; repeat mode 4 exact event log; single/combined per-visit decisions; interrupted hold remains 8 seconds; node ownership and zero-speed checks; 120 simulated seconds mode 0 baseline identical except wall-clock metrics.\n',encoding='utf-8')
print((out/'PASS.txt').read_text())
