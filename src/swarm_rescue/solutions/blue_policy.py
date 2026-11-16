"""Rule-compliant distributed blue-team rescue controller.

Every drone stores ONLY its own observations and received radio messages.
The public maps' *disposal center* locations are optional priors. Bomb locations
are NEVER hardcoded and are discovered through semantic sensing.
"""
from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple

from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor
from swarm_rescue.solutions.blue_nav import (
    LocalPilot, Occupancy, PoseTracker, clamp, distance, finite_pair, wrap,
)

Vec = Tuple[float, float]

# Public map definitions: (size, nominal starting center, disposal center, half-sizes).
# ONLY fixed disposal center coordinates are used (never bomb/wall geometry).
CENTER_PRIORS = (
    ((1250, 800), (-505, 98), (-505, 302), (113, 91)),
    ((1200, 900), (14, -277), (13, -402), (84, 42)),
    ((1113, 750), (-20, 93), (-20, -47), (105, 55)),
    ((1660, 1122), (-232, -463), (13, -476), (123, 80)),
    ((1200, 1200), (65, -15), (-122, 54), (69, 102)),
    ((1700, 1100), (612, 36), (781, 21), (66, 113)),
    ((1660, 1122), (-520, -411), (-720, -428), (105, 127)),
    ((1660, 1122), (586, 445), (746, 429), (78, 126)),
    ((1700, 1100), (574, -286), (755, -424), (89, 119)),
)


def safe_command(forward=0.0, lateral=0.0, rotation=0.0, grasper=0) -> CommandsDict:
    return dict(forward=clamp(forward), lateral=clamp(lateral),
                rotation=clamp(rotation), grasper=int(bool(grasper)))


class Track:
    __slots__ = ('xy', 'last_seen', 'state', 'owner', 'claimed_at', 'confidence')

    def __init__(self, xy: Vec, tick: int):
        self.xy = xy
        self.last_seen = tick
        self.state = 'free'
        self.owner = -1
        self.claimed_at = -1
        self.confidence = 1.0


class MyDroneRescue(DroneAbstract):
    """Ten fully independent agents with explicit communication-based consensus."""

    def __init__(self, identifier=None, misc_data=None, **kwargs):
        super().__init__(identifier=identifier, misc_data=misc_data, **kwargs)
        self.did = int(self.identifier if self.identifier is not None else 0)
        width, height = self.size_area or (1700, 1100)
        self.grid = Occupancy(int(width), int(height))
        self.pose = PoseTracker()
        self.pilot = LocalPilot()
        self.tracks: Dict[int, Track] = {}
        self.track_serial = 0
        self.peers: Dict[int, dict] = {}
        self.home: Optional[Vec] = None
        self.disposal: Optional[Vec] = None
        self.center_half = (85.0, 85.0)
        self.mode = 'EXPLORE'
        self.target: Optional[Vec] = None
        self.target_track: Optional[int] = None
        self.frontier_id: Optional[int] = None
        self.frontier_avoid: Dict[int, int] = {}
        self.path = []
        self.path_tick = -9999
        self.path_target: Optional[Vec] = None
        self.goal_tick = -9999
        self.was_carrying = False
        self.active_carry_track: Optional[int] = None
        self.prev_position: Optional[Vec] = None
        self.progress_tick = 0
        self.escape_left = 0
        self.escape_dir = 1 if self.did % 2 == 0 else -1
        self.latest_msg: Dict[str, Any] = {}
        self.last_tick = -1
        self.delivery_struggles = 0
        self._dock_stage = None
        self._dock_stage_tick = -1000
        self._last_known_heading = None

    def _select_center(self, xy: Vec):
        if self.disposal is not None:
            return
        matches = [(distance(xy, spawn), center, hs)
                   for dims, spawn, center, hs in CENTER_PRIORS
                   if tuple(self.size_area or ()) == dims]
        if matches:
            score, center, half = min(matches, key=lambda it: it[0])
            if score < 250:
                self.disposal = center
                self.center_half = half

    def _receive(self, tick: int):
        if self.communicator_is_disabled():
            return
        for _, msg in self.communicator.received_messages:
            if not isinstance(msg, dict) or msg.get('v') != 1:
                continue
            try:
                did = int(msg['id'])
                age = tick - int(msg['tick'])
            except (ValueError, TypeError, KeyError):
                continue
            if did == self.did or not (0 <= did < 100) or not (-3 <= age <= 240):
                continue
            self.peers[did] = msg
            if self.disposal is None:
                p = finite_pair(msg.get('home'))
                if p is not None:
                    self.disposal = p
            self.grid.merge(msg.get('cells'))
            for entry in msg.get('bombs', [])[:20]:
                try:
                    x, y, state, owner, seen, claimed = entry
                    xy = (float(x), float(y))
                    seen, claimed, owner = int(seen), int(claimed), int(owner)
                    if not (math.isfinite(xy[0]) and math.isfinite(xy[1])):
                        continue
                except (TypeError, ValueError):
                    continue
                if tick - seen > 750:
                    continue
                key = self._nearest_track(xy, radius=22.0)
                if key is None:
                    key = self._new_track(xy, seen)
                track = self.tracks[key]
                if state == 'done':
                    track.state, track.owner = 'done', owner
                    track.last_seen = max(track.last_seen, seen)
                elif track.state != 'done':
                    # Physical pickup dominates simultaneous claims. Claims use
                    # deterministic owner ID to avoid a last-message-wins race.
                    incoming = state if state in ('free','claimed','carried') else 'free'
                    accept = incoming == 'carried' or (incoming == 'claimed' and
                        track.state != 'carried' and (track.owner < 0 or
                        owner < track.owner or tick-track.claimed_at > 180))
                    if accept:
                        track.state,track.owner,track.claimed_at=incoming,owner,claimed
                        if incoming=='free':track.xy=xy
                track.last_seen = max(track.last_seen, seen)
        self.peers = {k: p for k, p in self.peers.items()
                      if tick - int(p.get('tick', -99999)) < 150}

    def _new_track(self, xy: Vec, tick: int) -> int:
        key = self.track_serial
        self.track_serial += 1
        self.tracks[key] = Track(xy, tick)
        return key

    def _nearest_track(self, xy: Vec, radius: float = 22.0) -> Optional[int]:
        choices = [(distance(t.xy, xy), k) for k, t in self.tracks.items()
                   if distance(t.xy, xy) < radius]
        return min(choices)[1] if choices else None

    def _observe(self, tick: int, xy: Vec, heading: float):
        visible = []
        detections = self.semantic_values() or []
        for det in detections:
            try:
                dist = float(det.distance)
                ang = float(det.angle)
                if not math.isfinite(dist) or not math.isfinite(ang):
                    continue
            except (TypeError, ValueError):
                continue
            kind = det.entity_type
            world = (xy[0] + dist * math.cos(heading + ang),
                     xy[1] + dist * math.sin(heading + ang))
            if kind == DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER:
                # The semantic sensor detects the VISIBLE SURFACE, not the center.
                # If public map prior exists, keep it, else use surface as an approach target.
                if self.disposal is None and dist < 125:
                    self.disposal = world
            elif kind == DroneSemanticSensor.TypeEntity.BOMB:
                key = self._nearest_track(world)
                if key is None:
                    key = self._new_track(world, tick)
                track = self.tracks[key]
                # Noisy GPS and moving bomb: fuse, but never resurrect a disposed bomb.
                if track.state != 'done':
                    track.xy = (0.72 * track.xy[0] + 0.28 * world[0],
                                0.72 * track.xy[1] + 0.28 * world[1])
                    track.last_seen = tick
                    if bool(det.grasped):
                        track.state = 'carried'
                    elif track.state == 'carried':
                        # Direct evidence of an ungrasped bomb overrides a stale peer report.
                        track.state, track.owner = 'free', -1
                    if not bool(det.grasped):
                        visible.append((dist, key, det))
        visible.sort(key=lambda item: item[0])
        return visible

    def _is_claimed_by_peer(self, key: int, tick: int, xy: Vec) -> bool:
        tr = self.tracks[key]
        if tr.state == 'done' or tr.state == 'carried':
            return True
        if tr.owner not in (-1, self.did) and tick - tr.claimed_at < 160:
            peer = self.peers.get(tr.owner)
            if peer is not None and not peer.get('carrying'):
                peer_xy = finite_pair(peer.get('pos'))
                if peer_xy is not None and distance(peer_xy, tr.xy) + 12 < distance(xy, tr.xy):
                    return True
        for pid, peer in self.peers.items():
            if peer.get('mode') != 'CHASE' or peer.get('carrying'):
                continue
            dest = finite_pair(peer.get('goal'))
            other = finite_pair(peer.get('pos'))
            if dest is None or other is None or distance(dest, tr.xy) > 22:
                continue
            own_cost = distance(xy, tr.xy)
            theirs = distance(other, tr.xy)
            if theirs + 15 < own_cost or (abs(theirs-own_cost) <= 15 and pid < self.did):
                return True
        return False

    def _choose_bomb(self, tick: int, xy: Vec, visible):
        candidates = []
        for key, tr in self.tracks.items():
            if tr.state in ('done', 'carried'):
                continue
            if tick - tr.last_seen > 420:
                continue
            if self._is_claimed_by_peer(key, tick, xy):
                continue
            # Locally observed targets are less likely to be stale.
            local_bonus = 75 if any(v[1] == key for v in visible) else 0
            cost = distance(xy, tr.xy) - local_bonus
            if self.target_track == key:
                cost -= 80.0
            candidates.append((cost, key))
        if not candidates:
            return None
        return min(candidates)[1]

    def _make_goal(self, tick: int, xy: Vec, visible, carrying: bool):
        if carrying:
            self.mode = 'DELIVER'
            self.target_track = None
            self.target = self.disposal
            return
        target_key = self._choose_bomb(tick, xy, visible)
        if target_key is not None:
            self.mode = 'CHASE'
            self.target_track = target_key
            tr = self.tracks[target_key]
            tr.owner, tr.claimed_at, tr.state = self.did, tick, 'claimed'
            self.target = tr.xy
            return
        self.target_track = None
        self.mode = 'EXPLORE'
        if (self.target is not None and self.frontier_id is not None
                and distance(xy, self.target) > 32 and tick - self.goal_tick < 135):
            return
        peers = list(self.peers.values())
        idx, frontier = self.grid.frontier(xy, self.home or xy,
                                            self.did, peers, self.frontier_avoid, tick)
        if frontier is not None:
            self.target, self.frontier_id = frontier, idx
            self.goal_tick = tick
        else:
            self.frontier_id = None
            # No frontier yet: push toward a deterministic id-specific spoke.
            a = 2 * math.pi * (self.did + 0.5) / 10.0
            tx = xy[0] + 120 * math.cos(a)
            ty = xy[1] + 120 * math.sin(a)
            w, h = self.size_area or (1700, 1100)
            self.target = (max(-w/2 + 55, min(w/2 - 55, tx)),
                           max(-h/2 + 55, min(h/2 - 55, ty)))
            self.goal_tick = tick

    def _route(self, tick: int, xy: Vec, target: Vec):
        changed = self.path_target is None or distance(target, self.path_target) > 34
        if changed or tick - self.path_tick >= 37 or not self.path:
            self.path = self.grid.astar(xy, target, carrying=self.mode=='DELIVER')
            self.path_target = target
            self.path_tick = tick
        # Pop passed waypoints, then safely look ahead along traversable grid.
        while len(self.path) > 1 and distance(xy, self.path[0]) < 32:
            self.path.pop(0)
        if self.path:
            candidate = self.path[0]
            for p in self.path[1:9]:
                if distance(xy, p) > 145 or not self._straight_clear(xy, p):
                    break
                candidate = p
            return candidate
        return target

    def _straight_clear(self, a: Vec, b: Vec) -> bool:
        d = distance(a, b)
        for k in range(1, int(d // 12) + 1):
            fraction = min(1.0, k * 12 / max(d, 1))
            p = (a[0] + fraction * (b[0]-a[0]), a[1] + fraction*(b[1]-a[1]))
            idx = self.grid.index(*p)
            if idx is None or not self.grid.passable(idx,carrying=self.mode=='DELIVER'):
                return False
        return True

    def _update_stuck(self, tick: int, xy: Vec, goal: Vec):
        if self.prev_position is None:
            self.prev_position, self.progress_tick = xy, tick
            return
        if distance(xy, self.prev_position) >= 24:
            self.prev_position, self.progress_tick = xy, tick
        elif tick - self.progress_tick > 65 and distance(xy, goal) > 36:
            self.escape_left = max(self.escape_left, 22)
            self.escape_dir *= -1
            self.progress_tick = tick
            self.prev_position = xy
            if self.frontier_id is not None:
                self.frontier_avoid[self.frontier_id] = tick
                self.goal_tick = -9999
            self.path = []

    def _finish_previous_carry(self, tick: int, xy: Optional[Vec], carrying: bool):
        if carrying and not self.was_carrying:
            # The task selected immediately before pickup is the only bomb
            # to mark as carried; other discovered bombs remain available.
            key = self.target_track
            if key is None and xy is not None:
                key = self._nearest_track(xy, radius=65.0)
            self.active_carry_track = key
            if key is not None and key in self.tracks:
                tr = self.tracks[key]
                tr.state, tr.owner, tr.claimed_at = 'carried', self.did, tick
        elif self.was_carrying and not carrying:
            key = self.active_carry_track
            if key is not None and key in self.tracks:
                tr = self.tracks[key]
                # Empty grasper close to the disposal center normally means
                # the engine auto-disposed the object after physical contact.
                # Do not report 'done' for a remote accidental drop.
                near_center = (xy is not None and self.disposal is not None and
                               distance(xy, self.disposal) < max(self.center_half) + 80)
                tr.state = 'done' if near_center else 'free'
                tr.owner = self.did if near_center else -1
                tr.last_seen = tick
            self.active_carry_track = None
        self.was_carrying = carrying

    def define_message_for_all(self) -> Any:
        if self.communicator_is_disabled():
            return None
        # Called by official framework; never invokes global state or true_*.
        return self.latest_msg or {
            'v': 1, 'id': self.did, 'tick': int(self.elapsed_timestep),
            'pos': list(self.pose.xy) if self.pose.xy else None,
            'home': list(self.disposal) if self.disposal else None,
            'mode': self.mode, 'goal': list(self.target) if self.target else None,
            'carrying': self.was_carrying, 'bombs': [], 'cells': []
        }

    def _prepare_message(self, tick: int, carrying: bool):
        tracks = sorted(self.tracks.values(), key=lambda t: -t.last_seen)[:25]
        self.latest_msg = {
            'v': 1, 'id': self.did, 'tick': tick,
            'pos': list(self.pose.xy) if self.pose.xy else None,
            'home': list(self.disposal) if self.disposal else None,
            'goal': list(self.target) if self.target else None,
            'mode': self.mode, 'carrying': carrying,
            'bombs': [[round(t.xy[0], 1), round(t.xy[1], 1), t.state,
                       t.owner, t.last_seen, t.claimed_at] for t in tracks],
            'cells': self.grid.export(110) if tick % 9 == self.did % 9 else [],
        }

    def _target_avoidance_mask(self):
        """Avoid falsely marking bombs/drones/center as static walls."""
        mask=[]
        for d in self.semantic_values() or []:
            try:
                if d.entity_type in (DroneSemanticSensor.TypeEntity.BOMB,
                                     DroneSemanticSensor.TypeEntity.DRONE,
                                     DroneSemanticSensor.TypeEntity.DISPOSAL_CENTER):
                    mask.append((float(d.angle),float(d.distance)))
            except (TypeError,ValueError,AttributeError):pass
        return mask

    def _dock_target(self,xy,tick):
        """Plan to a reachable face before contacting the physical center.

        No simulation object is accessed; center dimensions come from the
        public map definitions already used by this strategy.
        """
        if self.disposal is None:return None,False
        cx,cy=self.disposal
        hx,hy=self.center_half
        dx,dy=xy[0]-cx,xy[1]-cy
        inside=max(abs(dx)-hx,abs(dy)-hy)<48
        if inside:
            return self.disposal,True
        if self._dock_stage is None or tick-self._dock_stage_tick>90:
            margin=35.
            candidates=[(cx+hx+margin,cy),(cx-hx-margin,cy),
                        (cx,cy+hy+margin),(cx,cy-hy-margin)]
            candidates=[p for p in candidates if self.grid.index(*p) is not None]
            ranked=[]
            for pt in candidates:
                path=self.grid.astar(xy,pt,carrying=True,max_expansions=3800)
                if path and distance(path[-1],pt)<56:
                    length=sum(distance(a,b) for a,b in zip([xy]+path,path))
                    ranked.append((length,pt))
            if ranked:
                self._dock_stage=min(ranked)[1]
            elif candidates:
                self._dock_stage=min(candidates,key=lambda p:distance(xy,p))
            self._dock_stage_tick=tick
        if self._dock_stage is not None and distance(xy,self._dock_stage)>39:
            return self._dock_stage,False
        return self.disposal,True

    def control(self) -> CommandsDict:
        tick=int(self.elapsed_timestep)
        self.last_tick=tick
        try:
            self.pose.update(tick,self.measured_gps_position(),
                             self.measured_compass_angle(),self.odometer_values())
            xy,heading=self.pose.xy,self.pose.heading
            carrying=bool(self.grasped_bombs())
            self._receive(tick)
            if xy is None or heading is None:
                self._prepare_message(tick,carrying)
                return safe_command(rotation=.25,grasper=int(carrying))
            if self.home is None:self.home=xy
            self._select_center(xy)
            self._finish_previous_carry(tick,xy,carrying)
            lidar,angles=self.lidar_values(),self.lidar_rays_angles()
            if self.pose.uncertainty<38.:
                self.grid.scan(xy,heading,lidar,angles,tick,ignore=self._target_avoidance_mask())
            visible=self._observe(tick,xy,heading)
            for key,tr in self.tracks.items():
                if tr.state=='claimed' and tr.owner==self.did and key!=self.target_track:
                    tr.state,tr.owner='free',-1
            self._make_goal(tick,xy,visible,carrying)
            target=self.target
            if target is None:
                self._prepare_message(tick,carrying)
                return safe_command(rotation=.22,grasper=int(carrying))
            # Stagger launch to avoid initial dense formation collision.
            motion=self.pose.motion
            if tick<8+self.did*5 and not carrying:
                self._prepare_message(tick,carrying)
                return safe_command(rotation=.08 if self.did%2 else -.08)
            self._update_stuck(tick,xy,target)
            docking=False
            if carrying and self.disposal is not None:
                target,docking=self._dock_target(xy,tick)
            else:
                self._dock_stage=None
            if self.escape_left>0 and not docking:
                self.escape_left-=1
                self._prepare_message(tick,carrying)
                sign=float(self.escape_dir)
                rear=self.pilot.clearance(angles,lidar,math.pi)
                side=self.pilot.clearance(angles,lidar,sign*math.pi/2)
                if rear>48:
                    return safe_command(forward=-.28,rotation=.12*sign,grasper=int(carrying))
                if side>52:
                    return safe_command(lateral=.25*sign,rotation=.12*sign,grasper=int(carrying))
                return safe_command(rotation=.28*sign,grasper=int(carrying))
            waypoint=target if docking else self._route(tick,xy,target)
            peers=[p for p in self.peers.values()
                   if tick-int(p.get('tick',-99999))<75]
            # Use odometer displacement, not measured_velocity's mislabeled
            # px/s interpretation. Engine's motion is per step.
            fwd,lat,rot=self.pilot.command(
                heading,xy,waypoint,lidar,angles,peers=peers,carrying=carrying,
                docking=docking,velocity=motion,
                ignore_target=docking,stuck=self.escape_left>0)
            if docking:
                # Preserve velocity feedback while allowing center contact;
                # a blind constant push caused the repeated delivery deaths.
                self.delivery_struggles+=1
                a=wrap(math.atan2(self.disposal[1]-xy[1],
                                  self.disposal[0]-xy[0])-heading)
                if self.delivery_struggles>90:
                    # A slow tangent sweep can bring a trailing bomb across
                    # the center boundary. Keep component pushing inward.
                    sign=1 if (self.did+(self.delivery_struggles//90))%2 else -1
                    fwd+=.11*math.cos(a+sign*math.pi/2)
                    lat+=.11*math.sin(a+sign*math.pi/2)
                # Other drones cannot be pushed through; yield instead.
                for obj in self.semantic_values() or []:
                    if obj.entity_type!=DroneSemanticSensor.TypeEntity.DRONE:continue
                    try:
                        if float(obj.distance)<41 and abs(wrap(float(obj.angle)-a))<.6:
                            fwd,lat=0.,0.
                            break
                    except (ValueError,TypeError):continue
            else:
                self.delivery_struggles=0
            if self.mode=='CHASE' and self.target_track in self.tracks:
                tr=self.tracks[self.target_track]
                near=distance(xy,tr.xy)
                if near<34:
                    a=wrap(math.atan2(tr.xy[1]-xy[1],tr.xy[0]-xy[0])-heading)
                    rot=clamp(1.0*a,-.65,.65)
                    if math.hypot(fwd,lat)<.15 and near>11:
                        fwd+=.16*math.cos(a)
                        lat+=.16*math.sin(a)
            if carrying and self.active_carry_track in self.tracks:
                tr=self.tracks[self.active_carry_track]
                tr.state,tr.owner,tr.claimed_at='carried',self.did,tick
            self._prepare_message(tick,carrying)
            return safe_command(fwd,lat,rot,int(carrying or self.mode=='CHASE'))
        except (TypeError,ValueError,ArithmeticError,IndexError,AttributeError):
            return safe_command(grasper=int(bool(self.grasped_bombs())))

