from pathlib import Path
from diagnose_v6 import analyze
from make_eval_cases import FIXED
assert len(FIXED)==10 and sum(map(len,FIXED.values()))==100
assert all(len(x)==10 and len(set(x))==10 for x in FIXED.values())
s='''BLUE_V6_EVENT pickup drone=3 tick=10 track=0
BLUE_V6_DOCK drone=3 tick=100 phase=1 gap=15.0 speed=0.6 force=0.5 hp=19 recoveries=1
BLUE_V6_DAMAGE drone=3 tick=110 loss=1 hp=18 mode=DELIVER phase=1
BLUE_V6 drone=3 tick=300 state=DELIVER pos=(3.0,4.0) speed=0.3 carry=1 goal_dist=80.0 route=0 uncertainty=9.0 hp=18
BLUE_V6 drone=3 tick=500 state=DELIVER pos=(3.0,5.0) speed=0.3 carry=1 goal_dist=75.0 route=0 uncertainty=9.0 hp=18
BLUE_V6 drone=3 tick=700 state=DELIVER pos=(3.0,4.0) speed=0.3 carry=1 goal_dist=80.0 route=0 uncertainty=9.0 hp=17
BLUE_V6_EVENT released drone=3 tick=701 track=0
rescued nb: 1/10
round score: 10.5%
'''
result=analyze(s)
assert result['official_disposed']==[1,10]
assert result['health_loss_observed_by_drone'][3]==1
assert result['dock_phases'][1]==1
assert result['possible_close_dock_deadlocks']['3']['snapshots']==3
print('PASS V6 diagnostics and restored 100 fixed coordinates')
