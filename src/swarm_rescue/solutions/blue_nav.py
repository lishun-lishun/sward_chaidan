"""Legal-sensor navigation. No map-file access, engine ground truth or shared globals.

Occupancy is built only from the drone's laser and received radio updates.
Velocities supplied by the simulator's odometer are displacement PER STEP.
"""
from __future__ import annotations
import heapq
import math
from typing import Optional, Tuple

Vec = Tuple[float, float]


def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, float(x)))


def wrap(a):
    return (float(a) + math.pi) % (2 * math.pi) - math.pi


def distance(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def finite_pair(v):
    try:
        a, b = float(v[0]), float(v[1])
        return (a,b) if math.isfinite(a) and math.isfinite(b) else None
    except (ValueError, TypeError, IndexError):
        return None


class PoseTracker:
    def __init__(self):
        self.xy = None
        self.heading = None
        self.uncertainty = float('inf')
        self.last_tick = -1
        self.gps_lost_steps = 0
        self.motion = (0., 0.)  # odometry displacement in world pixels / step

    def update(self, tick, gps, compass, odometer):
        if tick == self.last_tick:
            return
        self.last_tick = tick
        g = finite_pair(gps)
        h = None
        try:
            if compass is not None and math.isfinite(float(compass)):
                h = wrap(compass)
        except (ValueError, TypeError):
            pass
        if self.heading is None and h is not None:
            self.heading = h
        if self.xy is None and g is not None:
            self.xy, self.uncertainty = g, 7.
        self.motion = (0., 0.)
        if odometer is not None and self.xy is not None and self.heading is not None:
            try:
                ds, alpha, theta = map(float, odometer[:3])
                if all(map(math.isfinite, (ds, alpha, theta))):
                    ds = clamp(ds, 0., 12.)
                    previous_heading = self.heading
                    self.motion = (ds*math.cos(previous_heading+alpha),
                                   ds*math.sin(previous_heading+alpha))
                    self.xy = (self.xy[0]+self.motion[0], self.xy[1]+self.motion[1])
                    self.heading = wrap(self.heading+clamp(theta, -.5, .5))
                    self.uncertainty = min(250., self.uncertainty + .055 + .045*ds)
            except (TypeError, ValueError, IndexError):
                pass
        if h is not None:
            self.heading = h if self.heading is None else wrap(self.heading + .35 * wrap(h-self.heading))
        if g is not None:
            if self.xy is None:
                self.xy = g
            else:
                alpha = .13 if self.gps_lost_steps == 0 else .32
                if distance(g,self.xy) > 90: alpha = .50
                self.xy = (self.xy[0]*(1-alpha)+g[0]*alpha,
                           self.xy[1]*(1-alpha)+g[1]*alpha)
            self.gps_lost_steps = 0
            self.uncertainty = max(5., min(20., self.uncertainty*.75))
        else:
            self.gps_lost_steps += 1


class Occupancy:
    CELL = 24
    def __init__(self, width, height):
        self.width, self.height = int(width), int(height)
        self.nx = math.ceil(self.width/self.CELL)
        self.ny = math.ceil(self.height/self.CELL)
        self.n = self.nx*self.ny
        self.log = [0]*self.n
        self.seen = bytearray(self.n)
        self.visits = [0]*self.n
        self.dirty = {}
        self.seq = 0

    def index(self, x, y):
        ix=int((x+self.width/2)//self.CELL)
        iy=int((y+self.height/2)//self.CELL)
        return iy*self.nx+ix if 0<=ix<self.nx and 0<=iy<self.ny else None

    def center(self, idx):
        return ((idx%self.nx+.5)*self.CELL-self.width/2,
                (idx//self.nx+.5)*self.CELL-self.height/2)

    def update(self,idx,value,seen=True):
        value=int(clamp(value,-6,6))
        if self.log[idx]!=value or (seen and not self.seen[idx]):
            self.log[idx]=value
            if seen:self.seen[idx]=1
            self.seq+=1
            self.dirty[idx]=(self.log[idx],int(self.seen[idx]))

    def scan(self,xy,heading,values,angles,tick,ignore=()):
        idx=self.index(*xy)
        if idx is not None:
            self.visits[idx]=min(255,self.visits[idx]+1)
            self.update(idx,min(-3,self.log[idx]))
        if values is None or angles is None or tick%5:return
        for k in range(0,min(len(values),len(angles)),3):
            try:
                r, a=float(values[k]),float(angles[k])
                if not math.isfinite(r) or not math.isfinite(a):continue
            except (ValueError,TypeError):continue
            if any(abs(wrap(a-c))<.24 and r>=d-22 for c,d in ignore):
                continue
            r=clamp(r,0,230)
            direction=heading+a
            ca,sa=math.cos(direction),math.sin(direction)
            prev=None
            for t in range(1,int(max(0,r-14)/13)+1):
                cell=self.index(xy[0]+13*t*ca,xy[1]+13*t*sa)
                if cell is not None and cell!=prev:
                    self.update(cell,self.log[cell]-1)
                prev=cell
            if 12<r<222:
                end=self.index(xy[0]+r*ca,xy[1]+r*sa)
                if end is not None:self.update(end,self.log[end]+2)

    def merge(self,updates):
        if not isinstance(updates,list):return
        for item in updates[:160]:
            try:idx,val,seen=(int(i) for i in item)
            except (ValueError,TypeError):continue
            if not 0<=idx<self.n or not -6<=val<=6:continue
            old=self.log[idx]
            if (val>=3 and old<3) or (abs(val)>abs(old) and not(old>=3 and val<0)):
                self.update(idx,val,bool(seen))
            elif seen and not self.seen[idx]:self.update(idx,old)

    def export(self,limit=100):
        data=list(self.dirty.items())[:limit]
        for idx,_ in data:self.dirty.pop(idx,None)
        return [[i,*v] for i,v in data]

    def obstacle(self,idx):
        x,y=idx%self.nx,idx//self.nx
        return x<1 or y<1 or x>=self.nx-1 or y>=self.ny-1 or self.log[idx]>=3

    def clearance(self,idx):
        """Grid proximity to known obstacles, not simulator geometry."""
        x,y=idx%self.nx,idx//self.nx
        nearest=3
        for dy in range(-2,3):
            for dx in range(-2,3):
                xx,yy=x+dx,y+dy
                if not 0<=xx<self.nx or not 0<=yy<self.ny:continue
                other=yy*self.nx+xx
                if self.log[other]>=3:
                    nearest=min(nearest,math.hypot(dx,dy))
        return nearest*self.CELL

    def passable(self,idx,carrying=False):
        if self.obstacle(idx):return False
        # Avoid cutting through a known wall: moderate inflation, rather than
        # closing every narrow gate on a 24px discretization.
        return self.clearance(idx)>=(30 if carrying else 24)

    def neighbors(self,idx,carrying=False):
        x,y=idx%self.nx,idx//self.nx
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            nx,ny=x+dx,y+dy
            if not 0<=nx<self.nx or not 0<=ny<self.ny:continue
            nxt=ny*self.nx+nx
            if not self.passable(nxt,carrying):continue
            if dx and dy:
                # No diagonal squeezing between blocked cells.
                if not self.passable(y*self.nx+nx,carrying):continue
                if not self.passable(ny*self.nx+x,carrying):continue
            yield nxt,math.hypot(dx,dy)

    def astar(self,start,goal,carrying=False,max_expansions=8500):
        src,dst=self.index(*start),self.index(*goal)
        if src is None or dst is None:return []
        if src==dst:return [goal]
        # The actual target (a bomb / docking surface) may sit next to an
        # obstacle. Search to the closest safe cell instead of failing outright.
        if not self.passable(dst,carrying):
            good=[]
            tx,ty=dst%self.nx,dst//self.nx
            for radius in range(1,5):
                for dy in range(-radius,radius+1):
                    for dx in range(-radius,radius+1):
                        xx,yy=tx+dx,ty+dy
                        if 1<=xx<self.nx-1 and 1<=yy<self.ny-1:
                            k=yy*self.nx+xx
                            if self.passable(k,carrying):good.append(k)
                if good:break
            if not good:return []
            dst=min(good,key=lambda k:distance(self.center(k),goal))
        cost={src:0.};prev={}
        queue=[(0.,src)]
        visited=set()
        best=src;near=float('inf')
        for _ in range(max_expansions):
            if not queue:break
            _,here=heapq.heappop(queue)
            if here in visited:continue
            visited.add(here)
            hh=distance(self.center(here),self.center(dst))/self.CELL
            if hh<near:best,near=here,hh
            if here==dst:best=here;break
            for nxt,step in self.neighbors(here,carrying):
                unknown=not self.seen[nxt]
                proximity=max(0,54-self.clearance(nxt))/54
                val=self.log[nxt]
                risk=1.0+(.9 if unknown else 0.)+3.5*proximity**2+(2.0 if val>=1 else 0.)
                proposed=cost[here]+step*risk
                if proposed<cost.get(nxt,float('inf')):
                    cost[nxt]=proposed;prev[nxt]=here
                    hn=distance(self.center(nxt),self.center(dst))/self.CELL
                    heapq.heappush(queue,(proposed+hn,nxt))
        if best==src:return []
        route=[];node=best
        while node!=src and len(route)<self.n:
            route.append(self.center(node))
            node=prev.get(node,src)
        route.reverse()
        if best==dst and distance(route[-1],goal)<self.CELL*1.5:route.append(goal)
        return route

    def frontier(self,pos,home,ident,peers,avoided,tick):
        candidates=[]
        sector=-math.pi+(ident+.5)*2*math.pi/10
        for i in range(self.n):
            if not self.seen[i] or self.log[i]>-1 or not self.passable(i):continue
            x,y=i%self.nx,i//self.nx
            if x<2 or y<2 or x>self.nx-3 or y>self.ny-3:continue
            neighbors=[i-1,i+1,i-self.nx,i+self.nx]
            unk=sum(not self.seen[k] for k in neighbors)
            if not unk:continue
            point=self.center(i)
            travel=distance(pos,point)
            if travel<50 or travel>1000:continue
            bearing=math.atan2(point[1]-home[1],point[0]-home[0])
            penalty=48*abs(wrap(bearing-sector))
            penalty+=180 if avoided.get(i,-99999)>tick-300 else 0
            for peer in peers:
                g=finite_pair(peer.get('goal'))
                if g is not None and distance(point,g)<155:penalty+=145
            gain=0
            for yy in range(max(1,y-4),min(self.ny-1,y+5),2):
                for xx in range(max(1,x-4),min(self.nx-1,x+5),2):
                    if not self.seen[yy*self.nx+xx]:gain+=1
            candidates.append((gain*12+unk*36-.23*travel-penalty,i,point))
        if not candidates:return None,None
        for _,i,p in heapq.nlargest(12,candidates):
            if self.astar(pos,p):return i,p
        return None,None


class LocalPilot:
    """Velocity-error force control with swept-distance laser safety.

    The requested speed and the odometry are BOTH measured in px/step.
    """
    def __init__(self):
        self.last_dir=0.
        self.last_action=(0.,0.)

    @staticmethod
    def clearance(angles,ranges,direction,half_angle=.23):
        if angles is None or ranges is None:return 230.
        choices=[]
        for a,r in zip(angles,ranges):
            try:
                a,r=float(a),float(r)
                if math.isfinite(a) and math.isfinite(r) and abs(wrap(a-direction))<half_angle:
                    choices.append(r)
            except (ValueError,TypeError):pass
        return max(0,min(230,min(choices))) if choices else 230.

    def command(self,heading,xy,waypoint,ranges,angles,peers=(),carrying=False,
                docking=False,stuck=False,velocity=None,ignore_target=False,
                force_speed=None):
        dx,dy=waypoint[0]-xy[0],waypoint[1]-xy[1]
        dist=math.hypot(dx,dy)
        desired=wrap(math.atan2(dy,dx)-heading) if dist>1 else 0.
        # Simulator's measured_velocity derives from odometer and is px/step.
        v=finite_pair(velocity) or (0.,0.)
        cv,sv=math.cos(heading),math.sin(heading)
        vf=cv*v[0]+sv*v[1]
        vl=-sv*v[0]+cv*v[1]
        spd=math.hypot(vf,vl)
        safety=36 if carrying else 29
        if docking:safety=20
        # Stopping distance is proportional to current momentum and payload.
        braking=safety+18+spd*(19 if carrying else 13)
        choices=[desired]+[wrap(desired+d) for d in (-.32,.32,-.65,.65,-1.05,1.05,-1.48,1.48,-2.1,2.1,math.pi)]
        if stuck:choices.extend([wrap(desired+math.pi/2),wrap(desired-math.pi/2)])
        best=None
        for direction in choices:
            free=self.clearance(angles,ranges,direction)
            if docking and ignore_target and abs(wrap(direction-desired))<.22:
                # Center is a physical collider, but bomb-to-center contact
                # is desired. Only suppress the *center-facing* laser sector.
                free=max(free,85.)
            penalty=4.*max(0.,braking-free)/max(braking,1.)
            if free<safety:penalty+=5.+.18*(safety-free)
            for peer in peers:
                p=finite_pair(peer.get('pos'))
                if p is None:continue
                separation=distance(p,xy)
                if separation<95 and abs(wrap(math.atan2(p[1]-xy[1],p[0]-xy[0])-heading-direction))<.65:
                    penalty+=3.4*(95-separation)/95
            score=2.0*math.cos(wrap(direction-desired))-penalty
            if abs(wrap(direction-self.last_dir))<.3:score+=.12
            if best is None or score>best[0]:best=(score,direction,free)
        _,direction,free=best
        self.last_dir=direction
        max_speed=1.55 if carrying else 2.45
        if docking:max_speed=.9
        if force_speed is not None:max_speed=force_speed
        max_speed*=clamp(dist/(90 if carrying else 80),.0,1.)
        if free<braking:
            max_speed*=clamp((free-safety)/max(1,braking-safety),0,1)
        if free<safety:max_speed=0.
        if docking and ignore_target:
            max_speed=max(max_speed,.55 if dist>35 else .22)
        if dist<11 and not docking:max_speed=0.
        if stuck and max_speed<.2:
            # Avoid uncontrolled force side-stepping while already close to wall.
            direction=wrap(direction+(.8 if math.sin(direction-self.last_dir)>=0 else -.8))
            max_speed=.55 if free>safety+10 else 0.
        tx,ty=max_speed*math.cos(direction),max_speed*math.sin(direction)
        gain=.36 if carrying else .29
        fwd=gain*(tx-vf)
        lat=gain*(ty-vl)
        cap=.61 if carrying else .56
        if docking:cap=.42
        magnitude=math.hypot(fwd,lat)
        if magnitude>cap:fwd,lat=fwd*cap/magnitude,lat*cap/magnitude
        # Stopped/blocked state must actually brake; nonzero odometry leads
        # to sign-reversing force so the robot does not drift into a wall.
        rotation=clamp(1.0*desired,-.7,.7)
        if docking:rotation=clamp(.6*desired,-.4,.4)
        self.last_action=(fwd,lat)
        return clamp(fwd),clamp(lat),rotation

