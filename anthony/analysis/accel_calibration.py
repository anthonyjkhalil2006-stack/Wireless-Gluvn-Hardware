"""Accelerometer offset and scale from six-face rests.

Put the board still on each face (x up, x down, y up, y down, z up, z down), ~10 s each.
For each axis the 'up' and 'down' readings are +g and -g plus the same offset, so
    offset = (up + down) / 2        scale = (up - down) / 2 / g
Usage:  python accel_calibration.py LOG.csv x_up=75-80 x_down=26-30 y_up=95-100 y_down=50-60 z_up=6-15 z_down=8-16
(windows in seconds; when the faces are in different logs, run once per log and combine by hand)
"""
import sys
import numpy as np
from common import G0, load_log, window

# Result of the Oct 21 2026 six-face test on the breadboard rig (MPU-6050, +/-8 g range).
OFFSET = np.array([0.260, -0.065, 1.427])      # m/s^2   (z is ~146 mg; datasheet allows +/-80 mg)
SCALE = np.array([0.9947, 1.0049, 1.0137])     # g per g (datasheet tolerance +/-3 %)


def apply(accel, offset=OFFSET, scale=SCALE):
    """Return accel in m/s^2 with offset removed and scale corrected."""
    return (np.asarray(accel) - offset) / scale


def from_faces(up, down):
    """up, down: median accel vectors (3,) for the +axis and -axis faces of ONE axis -> (offset, scale)."""
    return (up + down) / 2.0, (up - down) / 2.0 / G0


if __name__ == "__main__":
    log = load_log(sys.argv[1])
    med = {}
    for arg in sys.argv[2:]:
        name, w = arg.split("=")
        a, b = map(float, w.split("-"))
        med[name] = np.median(log["accel"][window(log["t"], a, b)], axis=0)
    for i, ax in enumerate("xyz"):
        if f"{ax}_up" in med and f"{ax}_down" in med:
            off = (med[f"{ax}_up"][i] + med[f"{ax}_down"][i]) / 2
            sc = (med[f"{ax}_up"][i] - med[f"{ax}_down"][i]) / 2 / G0
            print(f"{ax}: offset {off:+.3f} m/s^2 ({off / G0 * 1000:+.0f} mg)   scale {sc:.4f} g")
