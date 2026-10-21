"""Tilt-compensated compass heading.

The compass is mounted rotated relative to the IMU, so the calibrated field is mapped to the
board frame as Mb = [-My, Mx, Mz]. Using the accelerometer's 'up' u, remove the vertical part of
the field and of the board's x axis ("flatten both onto the table"), then take the angle between them:
    heading = -atan2( u . (xh x mh), xh . mh )
"""
import numpy as np


def to_board(mc):
    mc = np.asarray(mc)
    return np.c_[-mc[:, 1], mc[:, 0], mc[:, 2]]


def flat_heading(mb):
    """Heading assuming the board is level (degrees)."""
    return -np.degrees(np.arctan2(mb[:, 1], mb[:, 0]))


def tilt_heading(mb, accel):
    """Tilt-compensated heading in degrees. accel can be in any units (only its direction is used)."""
    u = accel / np.linalg.norm(accel, axis=1, keepdims=True)
    xb = np.array([1.0, 0.0, 0.0])
    xh = xb - (u @ xb)[:, None] * u
    mh = mb - np.sum(mb * u, axis=1)[:, None] * u
    return -np.degrees(np.arctan2(np.sum(u * np.cross(xh, mh), axis=1), np.sum(xh * mh, axis=1)))


def unwrap_deg(a):
    return np.degrees(np.unwrap(np.radians(a)))
