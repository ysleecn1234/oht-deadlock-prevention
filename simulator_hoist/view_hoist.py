"""Windowed hoisting viewer; shares workload admission rules with detector."""
import argparse
import json
import os
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode',type=int,choices=range(5),default=4)
    parser.add_argument('--oht',type=int,default=1000)
    parser.add_argument('--probability',type=float,default=.1)
    parser.add_argument('--seconds',type=float,default=8)
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--smoke-frames',type=int,default=0,help=argparse.SUPPRESS)
    args=parser.parse_args()
    if not 1 <= args.oht <= 1000: parser.error('--oht must be 1..1000 for bundled positions')
    os.chdir(ROOT)
    os.environ.update(HOIST_MODE=str(args.mode),HOIST_PROBABILITY=str(args.probability),
                      HOIST_SECONDS=str(args.seconds),HOIST_SEED=str(args.seed),
                      APPLY_FIXES='1',PENDING_JOB_LIMIT='2000',ENABLE_RING_CONTROL='1',
                      RING_CAPACITY_MARGIN='2')
    os.environ.pop('CASE_DIRECTORY',None)
    sys.argv=['agv_fab_sim.py']  # Prepare fonts and background for the GUI.
    import agv_fab_sim as s
    import deadlock_detector  # Same admission cap/schedule-exhaustion guard as experiments.
    from PIL import ImageDraw
    s.reset_sim_state()
    original=s.render_frame
    last_time=0.
    frames=0
    out=ROOT/'runs'; out.mkdir(exist_ok=True)
    def render(t,vp):
        nonlocal last_time,frames
        last_time=t
        frame=original(t,vp)
        draw=ImageDraw.Draw(frame); cam=vp.camera()
        for number,node in enumerate(('017192','016911','017024'),1):
            if node not in s.HOIST.active: continue
            x,y=s.NODE_XY[node]; px,py=cam.pt(s.sx(x),s.sy_pil(y))
            busy=any(v.node==node and v.state=='hoisting' for v in s.OHT_LIST)
            color='#e53935' if busy else '#d97706'
            draw.ellipse((px-9,py-9,px+9,py+9),outline=color,width=3)
            draw.text((px+12,py-18),f'H{number} {node}',font=s.FONT_STATS,fill=color,stroke_width=1,stroke_fill='white')
        count=sum(e['selected'] for e in s.HOIST.events)
        draw.rectangle((8,8,min(vp.win_w-8,820),42),fill='white',outline='#d97706')
        draw.text((16,15),f'Hoist mode {args.mode} | OHT {args.oht} | p={args.probability:g} | {args.seconds:g}s | events {count} | sim {t:.1f}s',font=s.FONT_STATS,fill='#222222')
        frames+=1
        if args.smoke_frames and frames>=args.smoke_frames:
            frame.save(out/'gui_smoke.png')
            s.pygame.event.post(s.pygame.event.Event(s.pygame.QUIT))
        return frame
    s.render_frame=render
    try:
        s.main(num_oht=args.oht,spawn_interval=.45)
    finally:
        name='gui_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json'
        result=dict(viewer=True,simulated_seconds=last_time,oht_count=args.oht,
                    spawn_interval_seconds=.45,pending_job_limit=2000,
                    hoisting=s.HOIST.summary(last_time))
        (out/name).write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')

if __name__=='__main__': main()
