"""V4 local-only regression tests; does not require Arcade/Pymunk or establish game score."""
import math
import runpy
from pathlib import Path
runpy.run_path(str(Path(__file__).with_name('test_v3.py')))
from swarm_rescue.solutions.blue_nav import PoseTracker, LocalPilot
from swarm_rescue.solutions.blue_policy import MyDroneRescue
p=PoseTracker()
p.seed_relative()
assert p.relative_frame and p.xy==(0.,0.) and p.heading==0.
p.update(1,None,None,(3.,0.,0.5))
assert abs(p.xy[0]-3)<1e-6 and abs(p.heading-0.5)<1e-6
p.update(2,None,None,(4.,0.,0.))
assert abs(p.xy[1]-4*math.sin(0.5))<1e-6
# Upon sensor recovery, global measurement must be converted, not mixed
# with the already-established local map coordinates.
p.update(3,(101.,204.), math.pi/2, (0.,0.,0.))
assert p.relative_frame and p.global_from_local is not None
local_before=p.xy
p.update(4,(101.,204.), math.pi/2, (0.,0.,0.))
assert math.hypot(p.xy[0]-local_before[0],p.xy[1]-local_before[1])<1e-5

# Missing GPS AND compass at first control: no global-map center prior allowed.
a=MyDroneRescue(identifier=2)
a._gps=a._compass=None
a.elapsed_timestep=1
a.control()
assert a.pose.relative_frame and a.relative_center and a.disposal is None
# Sensor supplies a local center-surface anchor, while flying with no GPS.
from types import SimpleNamespace as D
# Dummy 'Types' is loaded by v3 test module in sys.modules.
from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor
T=DroneSemanticSensor.TypeEntity
a._semantic=[D(distance=85.,angle=0.,entity_type=T.DISPOSAL_CENTER,grasped=False)]
a.elapsed_timestep=2
a.control()
assert a.disposal is not None and abs(a.disposal[0]-85)<2
# Reverse docking should continue to hold grasper even during perimeter towing.
a._carry=True
a.was_carrying=True
a.elapsed_timestep=150
a._semantic=[]
a.pose.xy=(75.,0.)
a.pose.heading=0.
a.disposal=(85.,0.)
a.mode='DELIVER'
a.delivery_struggles=150
cmd=a.control()
assert cmd['grasper']==1
# Emergency restriction: free-flight cruise must be below V3 aggressive 17 px/step
pilot=LocalPilot()
angles=[-math.pi+i*math.pi/90 for i in range(181)]
scan=[230.]*181
out=pilot.command(0.,(0.,0.),(400.,0.),scan,angles,velocity=(0.,0.))
assert 0.2<out[0]<=0.8
print('PASS V4: local SE2 frame, GPS reacquisition, semantic center, carry docking, flight safety.')
