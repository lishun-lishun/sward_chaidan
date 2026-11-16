"""V3 no-engine unit/regression tests. These do NOT establish contest scores."""
import math
import sys
import time
import types
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
for key in ('swarm_rescue','swarm_rescue.simulation','swarm_rescue.simulation.drone',
            'swarm_rescue.simulation.ray_sensors','swarm_rescue.simulation.reporting','swarm_rescue.solutions'):
    mod=types.ModuleType(key)
    mod.__path__=[str(ROOT/'src'/Path(*key.split('.')))]
    sys.modules[key]=mod

class DummyDrone:
    def __init__(self,identifier=None,misc_data=None,**kwargs):
        self.identifier=identifier
        self.size_area=(1250,800)
        self.elapsed_timestep=1
        self.communicator=types.SimpleNamespace(received_messages=[])
        self._gps=(-505.,98.)
        self._compass=0.
        self._odom=(0.,0.,0.)
        self._semantic=[]
        self._carry=False
        self._ranges=[230.]*181
        self._angles=[-math.pi+i*math.pi/90 for i in range(181)]
    def measured_gps_position(self):return self._gps
    def measured_compass_angle(self):return self._compass
    def odometer_values(self):return self._odom
    def lidar_values(self):return self._ranges
    def lidar_rays_angles(self):return self._angles
    def semantic_values(self):return self._semantic
    def grasped_bombs(self):return [1] if self._carry else []
    def communicator_is_disabled(self):return False

class Types:
    BOMB=1
    DISPOSAL_CENTER=2
    DRONE=3

for name,attrs in (
    ('swarm_rescue.simulation.drone.controller',{'CommandsDict':dict}),
    ('swarm_rescue.simulation.drone.drone_abstract',{'DroneAbstract':DummyDrone}),
    ('swarm_rescue.simulation.ray_sensors.drone_semantic_sensor',
     {'DroneSemanticSensor':types.SimpleNamespace(TypeEntity=Types)}),
):
    mod=types.ModuleType(name)
    mod.__dict__.update(attrs)
    sys.modules[name]=mod

from swarm_rescue.solutions.blue_nav import Occupancy,PoseTracker,LocalPilot,distance
from swarm_rescue.solutions.blue_policy import MyDroneRescue

# Correct dead reckoning, including heading change and no-GPS dropout.
p=PoseTracker()
p.update(1,(0.,0.),0.,(0.,0.,0.))
p.update(2,None,None,(5.,0.,0.))
assert abs(p.xy[0]-5)<.01 and p.gps_lost_steps==1
p.update(3,None,None,(5.,0.,0.))
assert abs(p.xy[0]-10)<.01

# Cached clearance responds to updated LiDAR geometry; forbid corner cutting.
g=Occupancy(240,240)
cx=g.nx//2;cy=g.ny//2
s=cy*g.nx+cx
right=s+1;up=s+g.nx
for k in (right,up):g.update(k,6)
assert not g.passable(right)
assert all(n!=s+g.nx+1 for n,_ in g._neighbors(s))
assert g.clearance(s)<=24
for k in (right,up):g.update(k,-6)
assert any(n==s+g.nx+1 for n,_ in g._neighbors(s))
assert g.astar(g.center(s),g.center(s+2*g.nx+2))

pilot=LocalPilot()
angles=[-math.pi+i*math.pi/90 for i in range(181)]
open_scan=[230.]*181
forward=pilot.command(0.,(0.,0.),(350.,0.),open_scan,angles,velocity=(0.,0.))
assert .5<=forward[0]<=.81,forward
# The intended bomb is a legitimate collision target, unlike a wall.
close=[230.]*181
for i,a in enumerate(angles):
    if abs(a)<.2:close[i]=26.
blocked=pilot.command(0.,(0.,0.),(26.,0.),close,angles,close_target=True)
allowed=pilot.command(0.,(0.,0.),(26.,0.),close,angles,
                      close_target=True,ignore=[(0.,26.)])
assert allowed[0]>=blocked[0]-1e-6,(blocked,allowed)

# Measured input-only controller contract, and carried grasper stays engaged.
a=MyDroneRescue(identifier=0)
for t in range(1,15):
    a.elapsed_timestep=t
    action=a.control()
    assert set(action)=={'forward','lateral','rotation','grasper'}
    assert all(-1<=action[k]<=1 for k in ('forward','lateral','rotation'))
assert a.disposal==(-505,302)
D=types.SimpleNamespace
b=MyDroneRescue(identifier=1)
b.elapsed_timestep=80
b._semantic=[D(distance=64.,angle=0.,entity_type=Types.BOMB,grasped=False)]
action=b.control()
assert b.mode=='CHASE' and action['grasper']==1,(b.mode,action)
b._carry=True
b.elapsed_timestep=81
b._semantic=[]
action=b.control()
assert b.mode=='DELIVER' and action['grasper']==1,(b.mode,action)
b._gps=None;b._compass=None;b._odom=(2.,0.,.1)
b.elapsed_timestep=82
assert b.control()['grasper']==1 and b.pose.xy is not None

# Basic physical response: force acts on mass 50 with 300 N and 0.95 damping.
# Test verifies cruise speed is not artificially limited to V2's 2.45 px/step.
v=0.
x=0.
control=LocalPilot()
for t in range(22):
    cmd=control.command(0.,(x,0.),(500.,0.),open_scan,angles,velocity=(v,0.))
    v=.95*v+6*cmd[0]
    x+=v
assert x>200,(x,v)

# A* performance regression on a grid similar to Map07.
big=Occupancy(1660,1122)
for i in range(10,big.nx-10):
    if i%13 not in (0,1,2):
        big.update((big.ny//2)*big.nx+i,6)
t0=time.perf_counter()
for _ in range(20):big.astar((-730,-300),(730,300),max_expansions=3400)
ms=(time.perf_counter()-t0)*1000/20
print('PASS V3 tests: syntax/control/grasper/odometry/occupancy/corner/path/velocity')
print('One-grid 20-route average path computation: %.2f ms (system dependent)'%ms)
print('Simulated unloaded travel in 22 steps: %.1f px (approximate physics)'%x)
