"""Offline V4 event/trajectory diagnostic; run outside official competition package."""
from __future__ import annotations
import argparse, json, re
from collections import defaultdict
from pathlib import Path

TRACE=re.compile(r'BLUE_V4 drone=(\d+) tick=(\d+) state=(\w+) pos=\(([-\d.]+),([-\d.]+)\) speed=([\d.]+) carry=(\d+) goal_dist=([\d.\-]+) route=(\d+) uncertainty=([\d.]+)')
EVENT=re.compile(r'BLUE_V4_EVENT (pickup|released) drone=(\d+) tick=(\d+)')
DOCK=re.compile(r'BLUE_V4_DOCK drone=(\d+) tick=(\d+) phase=(\d+) dist=([\d.]+) speed=([\d.]+)')

def diagnose(path:Path):
    s=path.read_text(encoding='utf8',errors='replace')
    bydrone=defaultdict(list);events=defaultdict(lambda:dict(pickup=0,released=0));dock=defaultdict(list)
    for a in TRACE.finditer(s):
        d,t,mode,x,y,v,car,gdist,route,unc=a.groups()
        bydrone[int(d)].append(dict(tick=int(t),state=mode,xy=[float(x),float(y)],speed=float(v),carrying=bool(int(car)),goal_distance=float(gdist),route=int(route),uncertainty=float(unc)))
    for a in EVENT.finditer(s):
        name,d,t=a.groups();events[int(d)][name]+=1
    for a in DOCK.finditer(s):
        d,t,phase,dist,v=a.groups();dock[int(d)].append(dict(tick=int(t),phase=int(phase),distance=float(dist),speed=float(v)))
    ids=sorted(set(bydrone)|set(events)|set(dock))
    summary=[]
    for ident in ids:
        seq=bydrone[ident]
        carrying=[x for x in seq if x['carrying']]
        stalled=[x for x in carrying if x['speed']<1.2 and x['goal_distance']>35]
        high_unc=[x for x in seq if x['uncertainty']>35]
        summary.append(dict(drone=ident,events=events[ident],last=seq[-1] if seq else None,
                            carrying_trace_points=len(carrying),slow_carry_trace_points=len(stalled),
                            uncertain_trace_points=len(high_unc),dock_samples=len(dock[ident]),
                            suspected_delivery_stall=len(stalled)>=3))
    total_disposed=re.findall(r'rescued nb:\s*(\d+)\s*/\s*(\d+)',s)
    score=re.findall(r'round score:\s*([\d.]+)%',s)
    return dict(file=str(path),disposed=list(map(int,total_disposed[-1])) if total_disposed else None,
                score=float(score[-1]) if score else None,
                has_engine_traceback='Traceback (most recent call last)' in s,
                drones=summary)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('logs',nargs='+');p.add_argument('--output',default='v4_diagnostics.json')
    args=p.parse_args();paths=[]
    for glob in args.logs:paths+=list(Path().glob(glob)) if '*' in glob else [Path(glob)]
    result=[diagnose(p) for p in paths if p.is_file()]
    Path(args.output).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print('Analysed',len(result),'logs ->',args.output)
