"""The team frame: where each robot of a team stands, in one planar frame the robots share.

No robot here measures its position in the room. The frame is a ledger: the operator gives each robot's starting
pose (x, y in metres, yaw in degrees; any fixed frame of the room), every base motion a robot reports moves its
entry, and the agent corrects an entry from what it perceives (team_set_pose). Each entry carries an uncertainty that
grows with every motion and that the separation check adds to the robot's radius.

The separation check is a planning check, not a guard: it refuses a base motion whose path would bring two robots'
bodies closer than `min_gap_m`, from the ledger alone. It does not see arms, the steps a robot takes on its own while
reaching, or anything else in the room.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class Pose:
    x: float
    y: float
    yaw: float              # radians
    uncertainty_m: float
    source: str
    updated: float

    def report(self):
        return {'x_m': round(self.x, 3), 'y_m': round(self.y, 3), 'yaw_deg': round(math.degrees(self.yaw), 1),
                'uncertainty_m': round(self.uncertainty_m, 3), 'source': self.source, 'updated': round(self.updated, 2)}


def _segment_distance(a0, a1, b0, b1):
    """Shortest distance between two planar segments (either may be a point)."""
    def point_segment(p, s0, s1):
        dx, dy = s1[0] - s0[0], s1[1] - s0[1]
        length2 = dx * dx + dy * dy
        t = 0.0 if length2 < 1e-12 else max(0.0, min(1.0, ((p[0] - s0[0]) * dx + (p[1] - s0[1]) * dy) / length2))
        return math.hypot(p[0] - (s0[0] + t * dx), p[1] - (s0[1] + t * dy))

    def cross(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])

    d1, d2, d3, d4 = cross(b0, b1, a0), cross(b0, b1, a1), cross(a0, a1, b0), cross(a0, a1, b1)
    if d1 * d2 < 0 and d3 * d4 < 0:
        return 0.0  # they cross
    return min(point_segment(a0, b0, b1), point_segment(a1, b0, b1), point_segment(b0, a0, a1), point_segment(b1, a0, a1))


class TeamFrame:
    def __init__(self, radius_m, *, min_gap_m=0.3, drift_per_m=0.1, drift_per_motion_m=0.03, wall=None):
        import time
        self.radius = dict(radius_m)          # member -> body radius (m) in the plane, arms at the sides
        self.min_gap_m = float(min_gap_m)
        self.drift_per_m = float(drift_per_m)
        self.drift_per_motion_m = float(drift_per_motion_m)
        self.wall = wall or time.time
        self.poses = {}

    def set(self, member, x, y, yaw_deg, uncertainty_m=0.05, source='operator'):
        if member not in self.radius:
            raise ValueError(f'unknown robot {member!r}; the team has {sorted(self.radius)}')
        self.poses[member] = Pose(float(x), float(y), math.radians(float(yaw_deg)), float(uncertainty_m), source, self.wall())

    def _ends(self, pose, motion):
        """Where a base motion (forward m, left m, turn deg) may end: turning first, or turning last."""
        forward, left, turn = motion
        ends = []
        for heading in {pose.yaw, pose.yaw + math.radians(turn)}:
            c, s = math.cos(heading), math.sin(heading)
            ends.append((pose.x + forward * c - left * s, pose.y + forward * s + left * c))
        return ends

    def advance(self, member, forward_m, left_m, turn_deg, *, measured, extra_uncertainty_m=0.0):
        """Move a robot's entry by a base motion it reported (in the heading it started with, then the turn)."""
        pose = self.poses.get(member)
        if pose is None:
            return
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        pose.x += forward_m * c - left_m * s
        pose.y += forward_m * s + left_m * c
        pose.yaw = math.atan2(math.sin(pose.yaw + math.radians(turn_deg)), math.cos(pose.yaw + math.radians(turn_deg)))
        span = math.hypot(forward_m, left_m)
        pose.uncertainty_m += (self.drift_per_motion_m + self.drift_per_m * span * (1.0 if measured else 2.0)
                               + abs(extra_uncertainty_m))
        pose.source = 'dead reckoning (measured motion)' if measured else 'dead reckoning (commanded motion)'
        pose.updated = self.wall()

    def blur(self, member, reach_m):
        """A robot moved its base by an amount nobody measured: its entry may be off by up to reach_m more."""
        pose = self.poses.get(member)
        if pose is not None:
            pose.uncertainty_m += abs(reach_m)
            pose.source = 'dead reckoning (unmeasured motion)'
            pose.updated = self.wall()

    def gap(self, a, b, motions=None):
        """Smallest body-to-body gap (m) between two robots while each makes its base motion (None: stands)."""
        motions = motions or {}
        pa, pb = self.poses.get(a), self.poses.get(b)
        if pa is None or pb is None:
            return None
        ends_a = self._ends(pa, motions[a]) if motions.get(a) else [(pa.x, pa.y)]
        ends_b = self._ends(pb, motions[b]) if motions.get(b) else [(pb.x, pb.y)]
        centre = min(_segment_distance((pa.x, pa.y), ea, (pb.x, pb.y), eb) for ea in ends_a for eb in ends_b)
        return centre - self.radius[a] - self.radius[b] - pa.uncertainty_m - pb.uncertainty_m

    def conflicts(self, motions, allow_close=()):
        """Pairs a set of base motions would bring closer than min_gap_m. motions: {member: (forward, left, turn)}.
        allow_close: pairs (frozensets) that are meant to work within reach of each other."""
        found, unknown = [], []
        members = sorted(self.radius)
        for i, a in enumerate(members):
            for b in members[i + 1:]:
                if not (motions.get(a) or motions.get(b)) or frozenset((a, b)) in allow_close:
                    continue
                gap = self.gap(a, b, motions)
                if gap is None:
                    unknown.append((a, b))
                elif gap < self.min_gap_m:
                    found.append({'robots': [a, b], 'gap_m': round(gap, 3), 'min_gap_m': self.min_gap_m})
        return found, unknown

    def report(self):
        members = sorted(self.radius)
        gaps = {}
        for i, a in enumerate(members):
            for b in members[i + 1:]:
                gap = self.gap(a, b)
                gaps[f'{a}-{b}'] = None if gap is None else round(gap, 3)
        return {'poses': {m: (self.poses[m].report() if m in self.poses else None) for m in members},
                'body_radius_m': dict(self.radius), 'gap_m': gaps, 'min_gap_m': self.min_gap_m,
                'note': 'a ledger, not a measurement: starting poses from the operator, moved by each reported base '
                        'motion, corrected with team_set_pose; gaps subtract both radii and both uncertainties'}
