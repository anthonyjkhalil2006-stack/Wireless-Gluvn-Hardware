"""Gated complementary filter: gyro carries fast motion, the compass slowly pulls out drift.

    f[i] = f[i-1] + (gyro_yaw[i] - gyro_yaw[i-1])
    if compass trusted:  f[i] += (1 - alpha) * wrap(compass[i] - f[i])        alpha = 0.995 at 50 Hz (tau ~ 4 s)

The compass is trusted only when the accelerometer reads gravity alone and the heading is well defined:
    | |a| - 9.81 | < 0.5   and   |u_x| < 0.7.
Usage:  python fusion.py LOG.csv [START_A START_B]     (start window in s, default 0.5 8: board still, level)
"""
import sys
import numpy as np
from common import G0, load_log, window, wrap
import accel_calibration, compass_calibration, tilt_compensation, gyro_quaternion

ALPHA = 0.995


def fuse(gyro_yaw, compass, trusted, alpha=ALPHA):
    f = np.zeros(len(gyro_yaw))
    f[0] = gyro_yaw[0]
    for i in range(1, len(f)):
        f[i] = f[i - 1] + (gyro_yaw[i] - gyro_yaw[i - 1])
        if trusted[i]:
            f[i] += (1 - alpha) * wrap(compass[i] - f[i])
    return f


def run(path, start=(0.5, 8.0)):
    log = load_log(path)
    t = log["t"]
    rest = window(t, *start)
    acc = accel_calibration.apply(log["accel"])
    u = acc / np.linalg.norm(acc, axis=1, keepdims=True)
    mb = tilt_compensation.to_board(compass_calibration.apply(log["mag"]))
    comp = tilt_compensation.unwrap_deg(tilt_compensation.tilt_heading(mb, acc))
    comp -= comp[rest].mean()                                   # start heading = 0
    bias = gyro_quaternion.bias_from_rest(log["gyro"], rest)
    gyro_yaw = gyro_quaternion.integrate_yaw(log["gyro"], bias, np.median(acc[rest], axis=0))
    gyro_yaw -= gyro_yaw[rest].mean()
    trusted = (np.abs(np.linalg.norm(acc, axis=1) - G0) < 0.5) & (np.abs(u[:, 0]) < 0.7)
    fused = fuse(gyro_yaw, comp, trusted)
    return t, gyro_yaw, comp, fused, trusted


if __name__ == "__main__":
    start = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) >= 4 else (0.5, 8.0)
    t, g, c, f, ok = run(sys.argv[1], start)
    print(f"{len(t)} samples, {t[-1]:.0f} s, compass trusted for {ok.mean() * 100:.0f}% of samples")
    print("final heading (deg): gyro %.1f   compass %.1f   fused %.1f" % (wrap(g[-1]), wrap(c[-1]), wrap(f[-1])))
    np.savetxt(sys.argv[1].replace(".csv", "_fused.csv"), np.c_[t, g, c, f], delimiter=",",
               header="t_s,gyro_yaw_deg,compass_deg,fused_deg", comments="")
