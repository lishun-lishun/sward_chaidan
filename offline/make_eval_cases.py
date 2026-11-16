"""OFFLINE benchmark builder, NEVER imported by the submission controller.

Run from project root: python /path/to/make_eval_cases.py --repo . --out bench_cases
Reads public map geometry ONLY for offline rejection sampling; the flight policy
only receives LiDAR/semantic/odometer observations and radio messages.
"""
import argparse
import json
import math
import random
from collections import deque
from pathlib import Path

FIXED = {
'Map01': [(-131,382),(-105,385),(-610,-382),(270,-380),(-97,-67),(305,131),(601,-383),(604,381),(270,381),(451,-380)],
'Map02': [(135,90),(-134,87),(-140,-78),(136,-78),(579,430),(-585,434),(-491,337),(485,335),(506,-189),(-505,-185)],
'Map03': [(16.5,-125),(-112.5,-124),(317.5,357),(-154.5,359),(-416.5,90),(-414.5,140),(-143.5,119),(-538.5,-161),(522.5,-347),(529.5,353)],
'Map04': [(-644,-275),(-816,-280),(-407,97),(-514,97),(-640,95),(-812,268),(-818,462),(809,-536),(586,96),(806,332)],
'Map05': [(164,253),(-172,-252),(391,-500),(499,-382),(-22,358),(23,-354),(-506,397),(-398,505),(577,580),(-583,585)],
'Map06': [(-543,-63),(-519,-519),(-504,-337),(-449,-339),(-694,109),(654,356),(-297,529),(-613,528),(-822,-528),(-831,-295)],
'Map07': [(-664,-180),(-423,-168),(-217,-282),(-650,92),(-410,104),(-805,275),(-814,360),(-810,460),(131,349),(225,-272)],
'Map07t4': [(662,180),(640,-94),(805,-313),(810,-418),(550,-540),(152,-538),(502,-95),(511,7),(432,172),(-125,-399)],
'Map08': [(566,-535),(-128,-428),(-239,-533),(-7,-533),(509,-96),(649,-90),(804,-49),(672,281),(207,288),(427,173)],
'Map09': [(-344,342),(-656,341),(608,530),(409,532),(49,472),(-230,472),(-551,112),(-687,-188),(-707,530),(823,528)],
}


def segdist(px, py, ax, ay, bx, by):
    dx, dy = bx-ax, by-ay
    t = max(0., min(1., ((px-ax)*dx+(py-ay)*dy)/(dx*dx+dy*dy+1e-9)))
    return math.hypot(px-ax-t*dx, py-ay-t*dy)


def world_to_img(x, y, w, h):
    return x+w/2, h/2-y


def clearance(x, y, data):
    w, h = data['canvas']['width'], data['canvas']['height']
    px, py = world_to_img(x, y, w, h)
    result = min(px, py, w-1-px, h-1-py)
    for item in data['elements']:
        kind = item['type']
        if kind == 'line_wall':
            a, b = item['points']
            result = min(result, segdist(px, py, *a, *b) - item.get('thickness', 6)*0.5)
        elif kind in ('rect_wall', 'disposal_center','start_area'):
            a, b = item['top_left'], item['bottom_right']
            ix, iy = max(a[0], min(px,b[0])), max(a[1],min(py,b[1]))
            result = min(result, math.hypot(px-ix, py-iy))
        # NO_GPS_ZONE/NO_COM_ZONE are *not* obstacles; keep them active.
    return result


def reachable_grid(data, resolution=20, margin=18):
    w,h=int(data['canvas']['width']),int(data['canvas']['height'])
    nx,ny=(w+resolution-1)//resolution,(h+resolution-1)//resolution
    open_set=set()
    for iy in range(ny):
        for ix in range(nx):
            x=(ix+.5)*resolution-w/2
            y=h/2-(iy+.5)*resolution
            if clearance(x,y,data)>margin:
                open_set.add((ix,iy))
    starts = [e for e in data['elements'] if e['type']=='start_area']
    if starts:
        item=starts[0]
        cx=(item['top_left'][0]+item['bottom_right'][0])*.5
        cy=(item['top_left'][1]+item['bottom_right'][1])*.5
        src=(int(cx//resolution),int(cy//resolution))
    else:
        src=(nx//2,ny//2)
    if src not in open_set:
        src=min(open_set, key=lambda p:(p[0]-src[0])**2+(p[1]-src[1])**2)
    reachable={src}
    queue=deque([src])
    while queue:
        ix,iy=queue.popleft()
        for n in ((ix+1,iy),(ix-1,iy),(ix,iy+1),(ix,iy-1)):
            if n in open_set and n not in reachable:
                reachable.add(n)
                queue.append(n)
    return reachable,resolution


def near_corner_points(data, rng, n=10):
    w,h=data['canvas']['width'],data['canvas']['height']
    reachable,cell=reachable_grid(data)
    out=[]
    for k in range(n):
        found=False
        for attempt in range(3200):
            # Bias to corners (75%) and remaining perimeter (25%), without
            # placing bombs on top of walls or inside the disposal center.
            if attempt < 2000 and rng.random()<.75:
                sx,sy=rng.choice([-1,1]),rng.choice([-1,1])
                x=sx*(w/2-rng.uniform(42,160))
                y=sy*(h/2-rng.uniform(42,155))
            elif rng.random()<.5:
                x=rng.uniform(-w/2+42,w/2-42)
                y=rng.choice([-1,1])*(h/2-rng.uniform(42,115))
            else:
                y=rng.uniform(-h/2+42,h/2-42)
                x=rng.choice([-1,1])*(w/2-rng.uniform(42,115))
            ix=int((x+w/2)//cell)
            iy=int((h/2-y)//cell)
            if (ix,iy) not in reachable or clearance(x,y,data)<24:
                continue
            if any(math.hypot(x-a,y-b)<48 for a,b in out):
                continue
            out.append((round(x,2),round(y,2)))
            found=True
            break
        if not found:
            raise RuntimeError(f'could only sample {len(out)}/{n} safely; check geometry')
    return out


def make_cases(repo, out, seeds=(11,29,53)):
    out.mkdir(parents=True,exist_ok=True)
    manifest=[]
    for name, pts in FIXED.items():
        map_file=repo/'map_data'/('map_'+name[3:].lower()+'.json')
        data=json.loads(map_file.read_text(encoding='utf8'))
        zones=[e['type'].upper() for e in data['elements'] if e['type'] in ('no_gps_zone','no_com_zone')]
        zones=list(dict.fromkeys(zones))
        def save(tag, positions):
            file=out/f'{name}_{tag}.json'
            payload={'map_name':name,'zones_config':zones,
                     'bombs':[{'x':x,'y':y,'theta':0.0} for x,y in positions]}
            file.write_text(json.dumps(payload,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
            manifest.append({'map':name,'case':tag,'file':str(file),'bombs':len(positions),
                             'minimal_clearance':round(min(clearance(x,y,data) for x,y in positions),1)})
        save('fixed',pts)
        for seed in seeds:
            rng=random.Random(seed+sum(map(ord,name))*17)
            save(f'corner_s{seed}',near_corner_points(data,rng))
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False),encoding='utf8')
    print(f'Generated {len(manifest)} cases in {out} (fixed + corner random).')
    for case in manifest:
        if case['case']=='fixed' and case['minimal_clearance']<12:
            print('WARNING fixed layout close to/on wall:',case['map'],case['minimal_clearance'])


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--repo',type=Path,default=Path('.'))
    parser.add_argument('--out',type=Path,default=Path('bench_cases'))
    parser.add_argument('--seeds',type=int,nargs='+',default=[11,29,53])
    args=parser.parse_args()
    make_cases(args.repo,args.out,args.seeds)
