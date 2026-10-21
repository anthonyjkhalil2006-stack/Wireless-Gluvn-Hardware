"""Magnetometer calibration: fit an axis-aligned ellipsoid to readings taken while turning the board
through many orientations. Centre = hard-iron offset (the board's own constant field);
radii = per-axis scale (soft iron). Corrected = (raw - centre) / radii, so Earth's field has length 1.
Usage:  python compass_calibration.py LOG.csv
"""
import sys
import numpy as np
from common import load_log

# Fit on the Oct 7 2026 motion log (12,780 samples), QMC5883L clone at +/-8 G.
CENTER = np.array([-428.2, 2660.6, -814.3])    # counts
RADII = np.array([1471.1, 1471.9, 1384.9])     # counts


def fit_ellipsoid(m):
    """Least-squares fit of a*x^2 + b*y^2 + c*z^2 + d*x + e*y + f*z = 1 -> (centre, radii)."""
    A = np.c_[m[:, 0] ** 2, m[:, 1] ** 2, m[:, 2] ** 2, m[:, 0], m[:, 1], m[:, 2]]
    a, b, c, d, e, f = np.linalg.lstsq(A, np.ones(len(m)), rcond=None)[0]
    centre = -np.array([d / (2 * a), e / (2 * b), f / (2 * c)])
    k = 1 + a * centre[0] ** 2 + b * centre[1] ** 2 + c * centre[2] ** 2
    return centre, np.sqrt(k / np.array([a, b, c]))


def apply(mag, centre=CENTER, radii=RADII):
    return (np.asarray(mag, float) - centre) / radii


if __name__ == "__main__":
    m = load_log(sys.argv[1])["mag"]
    c, r = fit_ellipsoid(m)
    n = np.linalg.norm(apply(m, c, r), axis=1)
    print("centre (counts):", np.round(c, 1))
    print("radii  (counts):", np.round(r, 1))
    print(f"|m| after correction: {n.mean():.3f} +/- {n.std():.3f}  (should be 1)")
