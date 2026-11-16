"""Pure-Python V6 unit and stub integration tests (NOT official simulator)."""
from __future__ import annotations
import math
import runpy
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
runpy.run_path(str(ROOT/'offline'/'test_v5.py'))
from swarm_rescue.solutions.blue_nav import CenterDockController, LocalPilot, Occupancy
from swarm_rescue.solutions.blue_policy import MyDroneRescue

ANGLES=[-math.pi+i*math.pi/90 for i in range(181)]
FREE=[230.]*181
PILOT=LocalPilot()

def call(c,t,health_drop=0,xy=(-76.,0.),motion=(0.,0.),
         semantic_hit=None,health=20):
    return c.action(t,xy,0.,motion,(0.,0.),(50.,40.),
                    lidar=FREE,angles=ANGLES,pilot=PILOT,
                    semantic_hit=semantic_hit,health_drop=health_drop,health=health)

# 1. A damage pulse triggers actual backoff, not further inward pressure.
c=CenterDockController(1)
first=call(c,0)
assert first[3]==c.SWEEP and first[0]>0.0
call(c,25,health_drop=1)
out=call(c,55,health_drop=1)
assert out[3]==c.RECOVER, (out,c.damage_pressure)
assert out[0]<-0.15, out
assert c.recoveries>=1
back=call(c,80)
assert back[3] in (c.APPROACH,c.SWEEP),back

# 2. Zero health loss does not manufacture damage feedback.
c=CenterDockController(3)
for t in (0,10,20,30):
    out=call(c,t)
assert c.damage_pressure==0 and c.recoveries==0

# 3. Semantically measured surface should correct an uncertain global gap
# without being blindly interpreted as the actual rectangle center.
c=CenterDockController(1)
out=call(c,0,xy=(-85.,0.),semantic_hit=(-56.,0.))
assert 28.0<out[4]<35.0,out

# 4. Controller never creates command outside the competition bounds.
for did in range(10):
    c=CenterDockController(did)
    for tick in (0,65,167,257,290):
        a=call(c,tick,xy=(-67.,15.),health_drop=int(tick==167),health=5)
        assert a is not None and all(-1<=v<=1 for v in a[:3])

# 5. Local recovery falls back to reachable cells, not an obstacle endpoint.
g=Occupancy(288,288)
source=(0.,0.)
start=g.index(*source)
assert start is not None
for block in (start+1,start+g.nx,start-g.nx):
    g.update(block,6)
waypoint=g.safe_local_waypoint(source,(100.,100.))
assert waypoint!=source and g.passable(g.index(*waypoint)),waypoint
assert not any(g.index(*waypoint)==k for k in (start+1,start+g.nx,start-g.nx))

# 6. Health loss is retrieved exclusively from the documented own-drone API.
a=MyDroneRescue(identifier=1)
a.drone_health=lambda:25
a.elapsed_timestep=101
command=a.control()
assert command['grasper'] in (0,1)
a.drone_health=lambda:23
a.elapsed_timestep=102
command=a.control()
assert a._health_loss==2 and a._health_last==23 and len(a._damage_times)==1
assert set(command)=={'forward','lateral','rotation','grasper'}

# 7. Existing V5 cruise is preserved in open space; recent damage lowers it.
base=LocalPilot().command(0.,(0.,0.),(500.,0.),FREE,ANGLES,
                          velocity=(0.,0.),speed_scale=1.)
limited=LocalPilot().command(0.,(0.,0.),(500.,0.),FREE,ANGLES,
                             velocity=(0.,0.),speed_scale=.55)
assert 0<limited[0]<=base[0],(base,limited)
print('PASS V6: health feedback, responsive docking, local fallback, API, bounds, cruise.')
