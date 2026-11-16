"""V5 isolated regression tests. No Arcade/Pymunk and no claims about official scores."""
from __future__ import annotations
import math
import runpy
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# The V4 tests import the V3 official-interface stubs and validate the whole
# class without installing Arcade or Pymunk.
runpy.run_path(str(ROOT/'offline'/'test_v4.py'))
from swarm_rescue.solutions.blue_nav import CenterDockController, Occupancy, LocalPilot
from swarm_rescue.solutions.blue_policy import MyDroneRescue

pilot=LocalPilot()
angles=[-math.pi+i*math.pi/90 for i in range(181)]
scan=[230.]*181

def move(ctrl,tick,xy=(-76.,0.),motion=(0.,0.),relative=False,hit=None):
    return ctrl.action(tick,xy,0.,motion,(0.,0.),None if relative else (50.,40.),
                       relative=relative,lidar=scan,angles=angles,pilot=pilot,
                       semantic_hit=hit)

# Close-to-wall motion must have non-zero tangential speed and a stronger
# actuator than V4's 0.36 docking cap, while staying within legal [-1,1].
ctrl=CenterDockController(1)
a=move(ctrl,0)
assert a is not None and a[3]==1,(a,ctrl.phase)
assert abs(a[1])>.35 and a[0]>0.,a
assert math.hypot(a[0],a[1])<=.741,a
assert all(-1.<=v<=1. for v in a[:3])
# Stable choice of rectangle face when oscillating near a corner.
old_face=ctrl.face
b=move(ctrl,50,xy=(-71.,6.))
assert ctrl.face==old_face and b[3]==1
# Deterministic phases: wall-follow -> reverse -> backoff -> reapproach.
assert move(ctrl,110)[3]==2
assert move(ctrl,191)[3]==3
assert move(ctrl,216)[3] in (0,1)
# Recovery must change towing direction to escape a collision deadlock.
assert ctrl.reversals>=1


# Simplified force-integration sanity check: a body constrained by the
# rectangle face must translate along the face instead of remaining pinned
# with zero tangential movement.  This is NOT a Pymunk equivalence test.
toy=CenterDockController(1)
x,y=-80.,0.
vx,vy=0.,0.
max_along=0.
for t in range(80):
    command=toy.action(t,(x,y),0.,(vx,vy),(0.,0.),(50.,40.))
    vx=.93*vx+3.*command[0]
    vy=.93*vy+3.*command[1]
    x+=vx
    y+=vy
    if x>-70.:
        x,vx=-70.,0.  # hard wall: only tangential travel remains
    max_along=max(max_along,abs(y))
assert max_along>20.,max_along

# Relative frame must use the CURRENT semantic surface hit, not silently
# interpret a public world-center in an unrelated map.
relative_ctrl=CenterDockController(0)
r=relative_ctrl.action(10,(70.,0.),0.,(0.,0.),(999.,999.),None,
                       relative=True,lidar=scan,angles=angles,pilot=pilot,
                       semantic_hit=(55.,0.))
assert r is not None and r[4]<20,r
# Without any center observation, a far-away stale surface point must not
# initiate blind contact behavior.
no_hit=relative_ctrl.action(20,(70.,0.),0.,(0.,0.),(-200.,-200.),None,
                            relative=True)
assert no_hit is None

# When frontiers are gone, choose a distant patrol destination rather than
# clamping to the outer map border and parking forever.
g=Occupancy(480,480)
patrol=g.patrol((0.,0.),3,0)
assert patrol is not None and math.hypot(*patrol)>140,patrol

# Driver + carrier integration: independent controllers per drone, no shared
# dock phase, and continued grasper closure under zero-GPS observation.
a1=MyDroneRescue(identifier=1)
a2=MyDroneRescue(identifier=2)
assert a1.dock_controller is not a2.dock_controller
assert a1.dock_controller.identifier==1
# At an approach to a known center, the real entry path remains intact.
a1._carry=True
for tick in (101,102,135):
    a1.elapsed_timestep=tick
    cmd=a1.control()
    assert set(cmd)=={'forward','lateral','rotation','grasper'}
    assert cmd['grasper']==1

# Submission must not read map geometry, placement answers, or prohibited
# true-position properties at runtime.
for name in ('blue_nav.py','blue_policy.py','my_drone_eval.py'):
    src=(ROOT/'src/swarm_rescue/solutions'/name).read_text(encoding='utf-8')
    assert 'map_data/' not in src and '.true_' not in src

print('PASS V5: center-force phases, no-GPS surface, patrol, official-interface stubs.')
