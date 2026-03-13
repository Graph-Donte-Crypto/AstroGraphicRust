"""Compare atan2 projection trick vs gradient-based E2 estimate.

The gradient approach solves df/dE2 = 0 approximately by dropping the
sin E2 cos E2 term (exact for circular planet orbits).
"""

import numpy as np
from scipy.optimize import minimize_scalar

def rotation_matrix(inc, omega, Omega):
    ci, si = np.cos(inc), np.sin(inc)
    co, so = np.cos(omega), np.sin(omega)
    cO, sO = np.cos(Omega), np.sin(Omega)
    return np.array([
        [cO*co - sO*so*ci, -cO*so - sO*co*ci],
        [sO*co + cO*so*ci, -sO*so + cO*co*ci],
        [so*si,             co*si            ],
    ])

def f_value(E1, E2, a1, e1, b1, a2, e2, b2, M_mat, r_soi):
    p1 = np.array([np.cos(E1) - e1, np.sin(E1)])
    p2 = np.array([np.cos(E2) - e2, np.sin(E2)])
    r1_sq = (a1 * (1 - e1 * np.cos(E1)))**2
    r2_sq = (a2 * (1 - e2 * np.cos(E2)))**2
    return r1_sq + r2_sq - p1 @ M_mat @ p2 - r_soi**2

def atan2_projection(E1, a1, e1, b1, a2, e2, b2, A, B):
    r1_2d = np.array([a1 * (np.cos(E1) - e1), b1 * np.sin(E1)])
    q = B.T @ (A @ r1_2d)
    return np.arctan2(q[1] / b2, q[0] / a2 + e2)

def gradient_estimate(E1, a1, e1, b1, a2, e2, b2, M_mat, r_soi):
    """Solve df/dE2 ≈ 0 by dropping sin*cos term (exact for e2=0).
    Check both critical points and pick the one with smaller f."""
    p1 = np.array([np.cos(E1) - e1, np.sin(E1)])
    v = M_mat.T @ p1
    A_coeff = 2 * a2**2 * e2 + v[0]
    B_coeff = v[1]
    E2a = np.arctan2(B_coeff, A_coeff)
    E2b = E2a + np.pi
    if E2b > np.pi:
        E2b -= 2 * np.pi
    fa = f_value(E1, E2a, a1, e1, b1, a2, e2, b2, M_mat, r_soi)
    fb = f_value(E1, E2b, a1, e1, b1, a2, e2, b2, M_mat, r_soi)
    return E2a if fa < fb else E2b

def true_E2_min_f(E1, a1, e1, b1, a2, e2, b2, M_mat, r_soi):
    result = minimize_scalar(
        lambda E2: f_value(E1, E2, a1, e1, b1, a2, e2, b2, M_mat, r_soi),
        bounds=(-np.pi, np.pi), method='bounded',
    )
    return result.x

def compare_methods(a1, e1, inc2_deg, e2, a2, r_soi, Omega2=0.0, omega1=0.0, omega2=0.0):
    inc2 = np.radians(inc2_deg)
    b1 = abs(a1) * np.sqrt(abs(1 - e1**2))
    b2 = a2 * np.sqrt(1 - e2**2)
    A = rotation_matrix(0, omega1, 0)
    B = rotation_matrix(inc2, omega2, Omega2)
    C = 2 * A.T @ B
    M_mat = np.diag([a1, b1]) @ C @ np.diag([a2, b2])

    max_err_proj = 0
    max_err_grad = 0
    for E1 in np.linspace(-np.pi, np.pi, 3600):
        E2_true = true_E2_min_f(E1, a1, e1, b1, a2, e2, b2, M_mat, r_soi)

        E2_proj = atan2_projection(E1, a1, e1, b1, a2, e2, b2, A, B)
        err_proj = min(abs(E2_proj - E2_true), 2*np.pi - abs(E2_proj - E2_true))
        max_err_proj = max(max_err_proj, err_proj)

        E2_grad = gradient_estimate(E1, a1, e1, b1, a2, e2, b2, M_mat, r_soi)
        err_grad = min(abs(E2_grad - E2_true), 2*np.pi - abs(E2_grad - E2_true))
        max_err_grad = max(max_err_grad, err_grad)

    return np.degrees(max_err_proj), np.degrees(max_err_grad)

# Pluto
a2 = 39.5
e2 = 0.25
r_soi = 0.05
a1_h = (1 + a2) / 2
e1_h = (a2 - 1) / (a2 + 1)

print(f"Earth-Pluto Hohmann: a1 = {a1_h:.2f} AU, e1 = {e1_h:.3f}")
print()

print("=== Varying eccentricity (Pluto i=17°) ===")
print(f"{'e1':>6s}  {'a1':>6s}  {'proj err (°)':>12s}  {'grad err (°)':>12s}")
print("-" * 50)
for e1 in [0.0, 0.2, 0.5, 0.8, e1_h]:
    a1 = (1 + a2) / (1 + e1) if e1 > 0 else a2
    ep, eg = compare_methods(a1, e1, 17, e2, a2, r_soi)
    print(f"{e1:6.3f}  {a1:6.1f}  {ep:12.2f}  {eg:12.2f}")

print()
print(f"=== Varying inclination (Hohmann e1={e1_h:.3f}) ===")
print(f"{'inc (°)':>7s}  {'proj err (°)':>12s}  {'grad err (°)':>12s}")
print("-" * 35)
for inc in [1, 5, 10, 17, 30, 45, 60, 90]:
    ep, eg = compare_methods(a1_h, e1_h, inc, e2, a2, r_soi)
    print(f"{inc:7d}  {ep:12.2f}  {eg:12.2f}")

print()
print(f"=== Varying Omega2 (Hohmann, i=17°) ===")
print(f"{'Omega2 (°)':>10s}  {'proj err (°)':>12s}  {'grad err (°)':>12s}")
print("-" * 38)
for Om in [0, 30, 60, 90, 120, 150, 180]:
    ep, eg = compare_methods(a1_h, e1_h, 17, e2, a2, r_soi, Omega2=np.radians(Om))
    print(f"{Om:10d}  {ep:12.2f}  {eg:12.2f}")

print()
print("=== Varying planet eccentricity (Hohmann-like, i=17°) ===")
print(f"{'e2':>6s}  {'proj err (°)':>12s}  {'grad err (°)':>12s}")
print("-" * 34)
for e2_test in [0.0, 0.01, 0.05, 0.1, 0.25, 0.5]:
    a1 = (1 + a2*(1-e2_test)) / 2  # apoapsis reaches periapsis of planet
    e1 = 1 - 1/a1  # periapsis at 1 AU
    if e1 <= 0 or e1 >= 1:
        continue
    ep, eg = compare_methods(a1, e1, 17, e2_test, a2, r_soi)
    print(f"{e2_test:6.2f}  {ep:12.2f}  {eg:12.2f}")
