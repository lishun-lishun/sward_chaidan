"""Blue-team navigation and perception. Only measured sensor data may enter here.

Occupancy grid + weighted A*, frontier utility, holonomic LiDAR velocity sampling.
No simulator objects, static walls, bomb lists, true poses, or shared class state.
"""
from __future__ import annotations

import heapq
import math
from collections import deque
from typing import Dict, List, Optional, Tuple

Vec = Tuple[float, float]


def clamp(x: float, low: float = -1.0, high: float = 1.0) -> float:
    return max(low, min(high, float(x)))


def wrap(a: float) -> float:
    return (float(a) + math.pi) % (2.0 * math.pi) - math.pi


def distance(a: Vec, b: Vec) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def finite_pair(v) -> Optional[Vec]:
    try:
        x, y = float(v[0]), float(v[1])
        return (x, y) if math.isfinite(x) and math.isfinite(y) else None
    except (TypeError, ValueError, IndexError):
        return None


class PoseTracker:
    """GPS+compass corrected dead reckoning with odometer during NO_GPS_ZONE.

    Uses odometer displacement as defined by official engine (travel, alpha, theta).
    The odometer displacement orientation is referenced to the PREVIOUS heading.
    """

    def __init__(self):
        self.xy: Optional[Vec] = None
        self.heading: Optional[float] = None
        self.uncertainty = float('inf')
        self.last_tick = -1
        self.gps_lost_steps = 0

    def update(self, tick: int, gps, compass, odometer):
        if tick == self.last_tick:
            return
        self.last_tick = tick
        gps_pos = finite_pair(gps)
        compass_angle = None
        try:
            if compass is not None and math.isfinite(float(compass)):
                compass_angle = wrap(float(compass))
        except (TypeError, ValueError):
            pass
        if self.heading is None and compass_angle is not None:
            self.heading = compass_angle
        if self.xy is None and gps_pos is not None:
            self.xy = gps_pos
            self.uncertainty = 8.0
        if odometer is not None and self.heading is not None and self.xy is not None:
            try:
                d, alpha, theta = (float(z) for z in odometer[:3])
                if all(math.isfinite(v) for v in (d, alpha, theta)):
                    d = clamp(d, 0.0, 10.0)
                    direction = self.heading + alpha
                    self.xy = (self.xy[0] + d * math.cos(direction),
                               self.xy[1] + d * math.sin(direction))
                    self.heading = wrap(self.heading + clamp(theta, -0.20, 0.20))
                    self.uncertainty = min(220.0, self.uncertainty + 0.018 + 0.095 * d)
            except (TypeError, ValueError, IndexError):
                pass
        if compass_angle is not None:
            if self.heading is None:
                self.heading = compass_angle
            else:
                self.heading = wrap(self.heading + 0.22 * wrap(compass_angle - self.heading))
        if gps_pos is not None:
            if self.xy is None:
                self.xy = gps_pos
            else:
                # Smooth AR(1) noise; correct accumulated odometry drift.
                a = 0.16 if self.gps_lost_steps == 0 else 0.36
                if distance(gps_pos, self.xy) > 100:
                    a = 0.50
                self.xy = (self.xy[0] * (1 - a) + gps_pos[0] * a,
                           self.xy[1] * (1 - a) + gps_pos[1] * a)
            self.gps_lost_steps = 0
            self.uncertainty = max(5.0, self.uncertainty * 0.67)
        else:
            self.gps_lost_steps += 1


class Occupancy:
    """Grid in world coordinates; never initialized from map/engine truth.

    Values (-6..6) model log odds. `seen` is deliberately separate from occupied:
    it prevents stale visited places being re-explored when no radio is available.
    """

    CELL = 24

    def __init__(self, width: int, height: int):
        self.width = int(width)
        self.height = int(height)
        self.nx = int(math.ceil(self.width / self.CELL))
        self.ny = int(math.ceil(self.height / self.CELL))
        self.n = self.nx * self.ny
        self.log = [0] * self.n
        self.seen = bytearray(self.n)
        self.visits = [0] * self.n
        self.dirty: Dict[int, Tuple[int, int]] = {}
        self.counter = 0

    def index(self, x: float, y: float) -> Optional[int]:
        ix = int((x + self.width * 0.5) // self.CELL)
        iy = int((y + self.height * 0.5) // self.CELL)
        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            return iy * self.nx + ix
        return None

    def center(self, idx: int) -> Vec:
        return (((idx % self.nx) + 0.5) * self.CELL - self.width * 0.5,
                ((idx // self.nx) + 0.5) * self.CELL - self.height * 0.5)

    def _update(self, idx: int, new: int, present: bool = True):
        old = self.log[idx]
        self.log[idx] = max(-6, min(6, new))
        if present:
            self.seen[idx] = 1
        if self.log[idx] != old or present:
            self.dirty[idx] = (self.log[idx], int(self.seen[idx]))

    def visit(self, xy: Vec):
        i = self.index(*xy)
        if i is None:
            return
        self.visits[i] = min(255, self.visits[i] + 1)
        self._update(i, min(self.log[i], -3))

    def ray(self, origin: Vec, heading: float, angle: float, measured: float,
            sensor_range: float = 230.0):
        if not math.isfinite(measured):
            return
        r = max(0.0, min(sensor_range, measured))
        ca, sa = math.cos(heading + angle), math.sin(heading + angle)
        prev = None
        # Stop short of an observed endpoint. Avoid carving the endpoint as free.
        for step in range(1, int(max(0.0, r - 10.0) // (self.CELL * 0.60)) + 1):
            s = step * (self.CELL * 0.60)
            idx = self.index(origin[0] + s * ca, origin[1] + s * sa)
            if idx is not None and idx != prev:
                self._update(idx, self.log[idx] - 1)
            prev = idx
        if 9.0 < r < sensor_range - 6.0:
            hit = self.index(origin[0] + r * ca, origin[1] + r * sa)
            if hit is not None:
                self._update(hit, self.log[hit] + 2)

    def scan(self, xy: Vec, heading: float, values, angles, tick: int):
        self.visit(xy)
        if values is None or angles is None or tick % 5 != 0:
            return
        for j in range(0, min(len(values), len(angles)), 3):
            try:
                self.ray(xy, heading, float(angles[j]), float(values[j]))
            except (ValueError, TypeError):
                continue

    def merge(self, updates):
        if not isinstance(updates, list):
            return
        for record in updates[:220]:
            try:
                idx, val, seen = (int(z) for z in record)
            except (ValueError, TypeError):
                continue
            if not (0 <= idx < self.n and -6 <= val <= 6):
                continue
            # Do not overwrite a confident local obstacle with a remote free ray.
            old_log, old_seen = self.log[idx], self.seen[idx]
            if abs(val) > abs(self.log[idx]) or (val < 0 and self.log[idx] < 3):
                self.log[idx] = val
            if seen:
                self.seen[idx] = 1
            if old_log != self.log[idx] or old_seen != self.seen[idx]:
                self.dirty[idx] = (self.log[idx], int(self.seen[idx]))

    def export(self, limit: int = 110):
        if not self.dirty:
            return []
        items = list(self.dirty.items())[:limit]
        for i, _ in items:
            self.dirty.pop(i, None)
        return [[i, v, seen] for i, (v, seen) in items]

    def _adj(self, idx: int):
        x, y = idx % self.nx, idx // self.nx
        for dx, dy, c in ((1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
                          (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)):
            tx, ty = x + dx, y + dy
            if 0 <= tx < self.nx and 0 <= ty < self.ny:
                yield ty * self.nx + tx, c

    def passable(self, idx: int) -> bool:
        x, y = idx % self.nx, idx // self.nx
        if x == 0 or y == 0 or x == self.nx - 1 or y == self.ny - 1:
            return False
        return self.log[idx] < 4

    def astar(self, start: Vec, goal: Vec, max_expansions: int = 8500) -> List[Vec]:
        src = self.index(*start)
        dst = self.index(*goal)
        if src is None or dst is None:
            return []
        if src == dst:
            return [goal]
        q = [(0.0, src)]
        best = {src: 0.0}
        came = {}
        end = src
        near_d = float('inf')
        expansions = 0
        while q and expansions < max_expansions:
            _, here = heapq.heappop(q)
            expansions += 1
            h = distance(self.center(here), goal) / self.CELL
            if h < near_d:
                near_d, end = h, here
            if here == dst:
                end = here
                break
            for there, step in self._adj(here):
                if not self.passable(there):
                    continue
                # Allow unknown cells, but prefer verified free corridors.
                val = self.log[there]
                cost = step * (1.0 + (1.1 if not self.seen[there] else 0.0)
                               + (5.0 if val >= 2 else 0.0))
                cand = best[here] + cost
                if cand < best.get(there, float('inf')):
                    best[there] = cand
                    came[there] = here
                    gx, gy = self.center(there)
                    hh = math.hypot(goal[0] - gx, goal[1] - gy) / self.CELL
                    heapq.heappush(q, (cand + hh, there))
        path = []
        while end in came and len(path) < self.n:
            path.append(self.center(end))
            end = came[end]
        path.reverse()
        if end == src and path and distance(path[-1], goal) < 2 * self.CELL:
            path.append(goal)
        return path

    def frontier(self, pos: Vec, home: Vec, ident: int, peers, avoided: Dict[int, int], tick: int):
        """Known-free cell at the boundary of unexplored space, with utility."""
        options = []
        nx = self.nx
        my_bearing = -math.pi + (ident + 0.5) * 2 * math.pi / 10.0
        for i in range(self.n):
            if not self.seen[i] or self.log[i] > -1:
                continue
            x, y = i % nx, i // nx
            if x < 2 or y < 2 or x > nx - 3 or y > self.ny - 3:
                continue
            near = [i - 1, i + 1, i - nx, i + nx]
            unexplored = sum(not self.seen[j] for j in near)
            if unexplored == 0:
                continue
            xy = self.center(i)
            travel = distance(pos, xy)
            if travel < 45.0 or travel > 850.0:
                continue
            bearing = math.atan2(xy[1] - home[1], xy[0] - home[0])
            sector_penalty = 42.0 * abs(wrap(bearing - my_bearing))
            peer_penalty = 0.0
            for peer in peers:
                goal = peer.get('goal')
                if goal and finite_pair(goal) is not None and distance(goal, xy) < 130:
                    peer_penalty += 165.0
            penalty = 220.0 if avoided.get(i, -10000) > tick - 280 else 0.0
            gain = 0
            for yy in range(max(1, y-3), min(self.ny-1, y+4), 2):
                for xx in range(max(1, x-3), min(self.nx-1, x+4), 2):
                    if not self.seen[yy * nx + xx]:
                        gain += 1
            utility = gain * 14.0 + unexplored * 30.0 - travel * 0.27 - sector_penalty - peer_penalty - penalty
            options.append((utility, i, xy))
        if not options:
            return None, None
        top = heapq.nlargest(8, options)
        for _, idx, xy in top:
            if self.astar(pos, xy):
                return idx, xy
        return top[0][1], top[0][2]


class LocalPilot:
    """Holonomic direction sampling (ORCA-inspired, not a full ORCA solver)."""

    def __init__(self):
        self.last_dir = 0.0
        self.last_action = (0.0, 0.0)

    @staticmethod
    def clearance(angles, ranges, direction: float, half_angle: float = 0.25):
        if angles is None or ranges is None:
            return 230.0
        val = 230.0
        count = 0
        for j in range(0, min(len(angles), len(ranges)), 2):
            a, r = float(angles[j]), float(ranges[j])
            if math.isfinite(r) and abs(wrap(a - direction)) < half_angle:
                val = min(val, r)
                count += 1
        return val if count else 230.0

    def command(self, heading: float, xy: Vec, waypoint: Vec, ranges, angles,
                peers=(), carrying=False, docking=False, stuck=False,
                velocity: Optional[Vec] = None):
        dx, dy = waypoint[0] - xy[0], waypoint[1] - xy[1]
        dist = math.hypot(dx, dy)
        theta = wrap(math.atan2(dy, dx) - heading)
        if dist < 12 and not docking:
            return (0.0, 0.0, 0.0)
        # The drone has considerable inertia (commands are forces, not
        # velocities), so clearance must include a braking margin.
        safe = 44.0 if carrying else 38.0
        if docking:
            safe = 24.0
        preferred = theta
        candidates = [preferred] + [wrap(preferred + t) for t in
                                    (-0.30, .30, -.60, .60, -.95, .95, -1.35, 1.35, -1.85, 1.85, math.pi)]
        if stuck:
            candidates.extend([wrap(preferred + math.pi/2), wrap(preferred-math.pi/2)])
        best_dir, best_score, best_clear = preferred, -float('inf'), 230.0
        for a in candidates:
            free = self.clearance(angles, ranges, a)
            # Steering samples only; use local reactive collision margin.
            hazard = max(0.0, safe + 40.0 - free) / (safe + 40.0)
            score = 2.0 * math.cos(wrap(a - preferred)) - 4.1 * hazard ** 2
            if free < safe:
                score -= 3.5 + 0.20 * (safe - free)
            # Peers only from received radio messages, never global object list.
            for peer in peers:
                p = finite_pair(peer.get('pos'))
                if p is None:
                    continue
                dd = distance(xy, p)
                if dd > 120:
                    continue
                peer_dir = wrap(math.atan2(p[1]-xy[1],p[0]-xy[0]) - heading)
                if abs(wrap(a - peer_dir)) < 0.55:
                    score -= 4.0 * max(0.0, (85.0 - dd) / 85.0)
            if abs(wrap(a - self.last_dir)) < 0.28:
                score += 0.12
            if score > best_score:
                best_score, best_dir, best_clear = score, a, free
        self.last_dir = best_dir
        scale = 1.0
        if best_clear < safe + 55.0:
            scale = clamp((best_clear - safe + 10.0) / 55.0, 0.0, 1.0)
        if best_clear < safe:
            scale = 0.0
        if dist < 65.0 and not docking:
            scale = min(scale, max(0.18, dist / 65.0))
        if stuck and scale < 0.2:
            # Prefer retreat/side-step instead of blindly charging an obstacle.
            best_dir = wrap(best_dir + math.pi / 2)
            scale = 0.40
        if docking and dist < 85.0:
            scale = min(scale, 0.56)
        # Engine command is FORCE, not velocity. Close the loop on the legal
        # odometer-derived velocity instead of continuously accelerating.
        vx = vy = 0.0
        if velocity is not None:
            try:
                wx, wy = float(velocity[0]), float(velocity[1])
                if math.isfinite(wx) and math.isfinite(wy):
                    vx = wx * math.cos(heading) + wy * math.sin(heading)
                    vy = -wx * math.sin(heading) + wy * math.cos(heading)
            except (TypeError, ValueError, IndexError):
                pass
        speed = math.hypot(vx, vy)
        desired_speed = 48.0 if carrying else 58.0
        if docking:
            desired_speed = 16.0
        if dist < 90.0:
            desired_speed *= clamp(dist / 90.0, 0.28, 1.0)
        # Stop requesting forward speed when the current momentum cannot be
        # shed inside the observed free corridor.
        brake_margin = safe + 12.0 + 0.55 * speed
        if best_clear < brake_margin:
            desired_speed *= clamp((best_clear - safe) / max(1.0, brake_margin - safe), 0.0, 1.0)
        desired_speed *= scale
        # A grasped bomb has twice the drone's mass and is coupled by a pivot;
        # it needs substantially more force to achieve the same translation.
        gain = 0.024 if carrying else 0.014
        fwd = gain * (desired_speed * math.cos(best_dir) - vx)
        lat = gain * (desired_speed * math.sin(best_dir) - vy)
        # Nearby radio peers contribute a gentle repulsive force. This remains
        # best-effort under partitions and uses no shared process state.
        for peer in peers:
            p = finite_pair(peer.get('pos'))
            if p is None:
                continue
            dd = distance(xy, p)
            if 1.0 < dd < 52.0:
                away = wrap(math.atan2(xy[1] - p[1], xy[0] - p[0]) - heading)
                repulse = 0.22 * (52.0 - dd) / 52.0
                fwd += repulse * math.cos(away)
                lat += repulse * math.sin(away)
        rot = clamp(1.45 * theta, -0.90, 0.90)
        if docking:
            rot = clamp(0.8 * theta, -0.5, .5)
        # Keep force commands conservative; the velocity loop supplies braking
        # by changing sign when momentum exceeds the requested speed.
        norm = math.hypot(fwd, lat)
        cap = 0.72 if carrying else 0.62
        if docking:
            cap = 0.24
        if norm > cap:
            fwd, lat = fwd * cap / norm, lat * cap / norm
        self.last_action = (fwd, lat)
        return clamp(fwd), clamp(lat), rot

