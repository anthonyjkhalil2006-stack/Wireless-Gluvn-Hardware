"""Gyro yaw by quaternion integration (keeps the full tilt, unlike summing gz).

Each 20 ms: w = (gyro - bias) * dt, then q <- q (x) [cos(th/2), (w/th) sin(th/2)], th = |w|.
Yaw is read from q with atan2.
"""
import numpy as np
from common import DT


def bias_from_rest(gyro, mask):
    """Gyro bias = median over a still window (median ignores short bumps)."""
    return np.median(gyro[mask], axis=0)


def _qmul(a, b):
    w0, x0, y0, z0 = a
    w1, x1, y1, z1 = b
    return np.array([w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
                     w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
                     w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
                     w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1])


def integrate_yaw(gyro, bias, accel_rest, dt=DT):
    """Yaw in degrees (unwrapped, relative to the start orientation).
    accel_rest: accelerometer vector at rest, used for the starting roll and pitch."""
    a = accel_rest / np.linalg.norm(accel_rest)
    roll = np.arctan2(a[1], a[2])
    pitch = np.arctan2(-a[0], np.hypot(a[1], a[2]))
    cr, sr, cp, sp = np.cos(roll / 2), np.sin(roll / 2), np.cos(pitch / 2), np.sin(pitch / 2)
    q = np.array([cr * cp, sr * cp, cr * sp, -sr * sp])
    yaw = np.zeros(len(gyro))
    for i, g in enumerate(gyro):
        w = (g - bias) * dt
        th = np.linalg.norm(w)
        if th > 0:
            q = _qmul(q, np.r_[np.cos(th / 2), w / th * np.sin(th / 2)])
            q /= np.linalg.norm(q)
        yaw[i] = np.degrees(np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2)))
    return np.degrees(np.unwrap(np.radians(yaw)))
