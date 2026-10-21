"""Shared helpers for the Gluvn rig analysis (50 Hz CSV log from firmware/gluvn_logger.ino)."""
import numpy as np

G0 = 9.80665      # standard gravity, m/s^2
DT = 0.02         # sample period, s (50 Hz)


def load_log(path):
    """Read a log. Columns: t_ms, ax, ay, az, gx, gy, gz, mx, my, mz, fsr, mark.
    Accel in m/s^2, gyro in rad/s, mag in raw counts. Lines starting with '#' are skipped."""
    rows = []
    for line in open(path):
        if line.startswith("#") or line.startswith("t_ms") or not line.strip():
            continue
        p = line.strip().split(",")
        if len(p) == 12:
            try:
                rows.append([float(x) for x in p])
            except ValueError:
                pass
    d = np.array(rows)
    return {"t": (d[:, 0] - d[0, 0]) / 1000.0, "accel": d[:, 1:4], "gyro": d[:, 4:7], "mag": d[:, 7:10]}


def window(t, a, b):
    """Boolean mask for a time window [a, b] seconds."""
    return (t >= a) & (t <= b)


def wrap(deg):
    """Wrap angles to [-180, 180)."""
    return (np.asarray(deg) + 180.0) % 360.0 - 180.0
