"""Offline V3-V6 post-mortem of official launcher logs, no engine changes."""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import re

PK=re.compile(r'BLUE_V\d+_EVENT\s+pickup\s+drone=(\d+)\s+tick=(\d+)')
REL=re.compile(r'BLUE_V\d+_EVENT\s+released\s+drone=(\d+)\s+tick=(\d+)')
DOCK=re.compile(r'BLUE_V\d+_DOCK\s+drone=(\d+)\s+tick=(\d+)\s+phase=(\d+)\s+gap=([-\d.]+)\s+speed=([-\d.]+)')
DAMAGE=re.compile(r'BLUE_V6_DAMAGE\s+drone=(\d+)\s+tick=(\d+)\s+loss=(\d+)\s+hp=(\d+)\s+mode=(\w+)\s+phase=(-?\d+)')
TRACE=re.compile(r'BLUE_V\d+\s+drone=(\d+)\s+tick=(\d+)\s+state=(\w+)\s+pos=\(([-\d.]+),([-\d.]+)\)\s+speed=([-\d.]+)\s+carry=(\d+)\s+goal_dist=([-\d.]+)')
SCORE=re.compile(r'round score:\s*([\d.]+)%',re.I)
RESCUED=re.compile(r'rescued nb:\s*(\d+)\s*/\s*(\d+)',re.I)


def analyze(text):
    pickups=Counter();released=Counter();phases=Counter();damage=Counter();damage_by_mode=Counter()
    trace=defaultdict(list)
    dock_stalls=defaultdict(int)
    for line in text.splitlines():
        m=PK.search(line)
        if m:pickups[int(m[1])]+=1
        m=REL.search(line)
        if m:released[int(m[1])]+=1
        m=DOCK.search(line)
        if m:
            did,_,phase,gap,speed=m.groups()
            phases[int(phase)]+=1
            if float(speed)<1.2 and abs(float(gap))<80:dock_stalls[int(did)]+=1
        m=DAMAGE.search(line)
        if m:
            did,tick,loss,hp,mode,phase=m.groups()
            damage[int(did)]+=int(loss)
            damage_by_mode[f'{mode}:phase={phase}']+=int(loss)
        m=TRACE.search(line)
        if m:
            did,tick,mode,px,py,speed,carry,dist=m.groups()
            trace[int(did)].append(dict(tick=int(tick),state=mode,
                      pos=(float(px),float(py)),speed=float(speed),
                      carrying=bool(int(carry)),goal_distance=float(dist)))
    close_carriers={}
    # Sparse telemetry gives only *potential* prolonged deadlocks, not true
    # continuous duration or bomb position. Do not count these as disposals.
    for did,items in trace.items():
        near=[x for x in items if x['state']=='DELIVER' and
              x['carrying'] and 0<=x['goal_distance']<210 and x['speed']<1.7]
        if len(near)>=3 and near[-1]['tick']-near[0]['tick']>=180:
            close_carriers[str(did)]={'snapshots':len(near),
                'first_tick':near[0]['tick'],'last_tick':near[-1]['tick']}
    score=SCORE.findall(text);resc=RESCUED.findall(text)
    return {'official_score':float(score[-1]) if score else None,
            'official_disposed':[int(x) for x in resc[-1]] if resc else None,
            'pickup_events':sum(pickups.values()),'release_events':sum(released.values()),
            'pickup_by_drone':dict(pickups),'release_by_drone':dict(released),
            'dock_phases':dict(phases),'low_speed_dock_observations':dict(dock_stalls),
            'health_loss_observed_by_drone':dict(damage),
            'damage_modes':dict(damage_by_mode),'possible_close_dock_deadlocks':close_carriers,
            'engine_callback_tracebacks':len(re.findall(r'Traceback \(most recent call last\)',text))}


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('log',nargs='+',type=Path,help='official run log files')
    ap.add_argument('--output',type=Path,default=Path('v6_diagnostics.json'))
    ns=ap.parse_args()
    rows={str(f):analyze(f.read_text(encoding='utf-8',errors='replace')) for f in ns.log}
    ns.output.write_text(json.dumps(rows,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps(rows,indent=2,ensure_ascii=False))

if __name__=='__main__':main()
