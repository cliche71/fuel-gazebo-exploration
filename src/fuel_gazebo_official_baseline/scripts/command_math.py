"""Frame conversion and the official demonstration's vertical startup path."""
import math
import numpy as np


def rotation_xyzw(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError('Invalid measured orientation')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def world_twist(q, linear, angular, covariance):
    """RotorS odometry twist is in child/body axes; FUEL expects world axes."""
    rot = rotation_xyzw(q)
    values = np.asarray([linear, angular], dtype=float)
    cov = np.asarray(covariance, dtype=float).reshape(6, 6)
    if not np.isfinite(values).all() or not np.isfinite(cov).all():
        raise ValueError('Invalid odometry twist')
    transform = np.zeros((6, 6))
    transform[:3, :3] = transform[3:, 3:] = rot
    return rot @ values[0], rot @ values[1], transform @ cov @ transform.T


def startup_height(elapsed, init_z):
    """Match upstream traj_server: 100 increments and 100 decrements of .01 m.

    Each sample lasts .01 simulation seconds. Upstream sets zero velocity and
    acceleration while sending these position commands; the adapter does too.
    At a slower control/physics update only the current sample is published.
    """
    if elapsed < 0:
        raise ValueError('Startup time went backwards')
    sample = min(200, int(math.floor((elapsed+1e-9)/.01))+1)
    return init_z + .01 * (sample if sample <= 100 else 200-sample)


def validate_command(position, velocity, acceleration, yaw, yaw_dot, bounds,
                     max_speed=3.0, max_acceleration=6.0, max_yaw_rate=3.0):
    data = np.asarray(list(position)+list(velocity)+list(acceleration)+[yaw, yaw_dot],
                      dtype=float)
    if not np.isfinite(data).all():
        raise ValueError('Non-finite FUEL command')
    p = np.asarray(position)
    if np.any(p < np.asarray(bounds[0])) or np.any(p > np.asarray(bounds[1])):
        raise ValueError('FUEL command leaves the configured scene bounds')
    if np.linalg.norm(velocity) > max_speed + 1e-6:
        raise ValueError('FUEL command exceeds the command speed limit')
    if np.linalg.norm(acceleration) > max_acceleration + 1e-6:
        raise ValueError('FUEL command exceeds the command acceleration limit')
    if abs(yaw_dot) > max_yaw_rate + 1e-6:
        raise ValueError('FUEL command exceeds the command yaw rate limit')
