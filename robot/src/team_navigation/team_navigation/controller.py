"""Floor trail -> (forward m/s, left-turn rad/s) using pure pursuit. No ROS imports.

Input is the trail from geometry.pixel_to_floor: an N x 2 array of (x, y)
metres in the robot frame (x forward, y left), ordered near -> far.
"""
import math
import numpy as np


def lookahead_point(trail, lookahead_m):
    """Return the (x, y) trail point about lookahead_m from the robot, or None.

    Interpolates between the two trail points either side of the lookahead
    distance. If the whole visible trail is farther than lookahead_m, uses its
    nearest point; if it is all closer, uses its farthest point (path ending).
    """
    trail = np.asarray(trail, np.float64).reshape(-1, 2)
    trail = trail[trail[:, 0] > 0]  # ignore anything at or behind the robot
    if len(trail) == 0:
        return None
    dist = np.hypot(trail[:, 0], trail[:, 1])
    beyond = np.flatnonzero(dist >= lookahead_m)
    if len(beyond) == 0:
        return tuple(trail[-1])
    i = int(beyond[0])
    if i == 0:
        return tuple(trail[0])
    d0, d1 = dist[i - 1], dist[i]
    t = (lookahead_m - d0) / (d1 - d0) if d1 != d0 else 0.
    return tuple(trail[i - 1] + t * (trail[i] - trail[i - 1]))


def pure_pursuit(trail, speed_m_s=0.05, lookahead_m=0.30, max_yaw_rad_s=1.0,
                 min_points=3):
    """Return (forward m/s, yaw rad/s, target) for the given floor trail.

    Steers along the circular arc through the lookahead point:
    yaw = speed * 2y / d^2, where d is the point's distance. Positive yaw
    turns left. Returns zero motion and target None when the path is lost
    (fewer than min_points trail points).
    """
    trail = np.asarray(trail, np.float64).reshape(-1, 2)
    if len(trail) < min_points:
        return 0., 0., None
    target = lookahead_point(trail, lookahead_m)
    if target is None:
        return 0., 0., None
    return pursue(target, speed_m_s, max_yaw_rad_s)


def pursue(target, speed_m_s=0.05, max_yaw_rad_s=1.0):
    """Return (forward, yaw, target) driving the arc through floor point target.

    yaw = speed * 2y / d^2, where d is the point's distance; positive turns
    left. Zero motion if the point is not ahead of the robot.
    """
    x, y = float(target[0]), float(target[1])
    if not x > 0:
        return 0., 0., None
    yaw = speed_m_s * 2. * y / (x * x + y * y)
    yaw = max(-max_yaw_rad_s, min(max_yaw_rad_s, yaw))
    if not (math.isfinite(speed_m_s) and math.isfinite(yaw)):
        return 0., 0., None
    return float(speed_m_s), float(yaw), (x, y)
