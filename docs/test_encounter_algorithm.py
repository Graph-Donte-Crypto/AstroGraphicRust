"""Test the encounter-finding algorithm from §8 of soi_encounter_numerical.typ.

Applies the algorithm to Voyager 2 / Jupiter.
Expected SOI entry: 1979-07-09 around 01:05 UTC (actual closest approach 22:29 UTC).
"""

import numpy as np
from datetime import datetime, timedelta

# --- Orbital elements ---
# Voyager 2 at spacecraft epoch 1977-08-23 11:29:11 UTC
# Source: JPL Horizons state vectors just before Jupiter arrival
a1 = 546095282.4529687         # km
e1 = 0.7331215656362654
i1 = np.radians(5.015)
Om1 = np.radians(-32.9834163461)
w1 = np.radians(12.2919523634)
M01_at_sc_epoch = 6.25059      # rad

mu_sun = 132712440042.0
mu_jup = 1.266865319e8
mu_jup_eff = mu_sun + mu_jup           # heliocentric motion of Jupiter
AU_KM = 149597870.7

# --- Jupiter elements: JPL approx_pos (Standish & Williams, Table 1) ---
JUP_TABLE1 = dict(
    a0=5.20288700,  a_dot=-0.00011607,
    e0=0.04838624,  e_dot=-0.00013253,
    I0=1.30439695,  I_dot=-0.00183714,
    L0=34.39644051, L_dot=3034.74612775,
    lp0=14.72847983,  lp_dot=0.21252668,       # longitude of perihelion
    ln0=100.47390909, ln_dot=0.20469106,       # longitude of ascending node
)

# Evaluate at an epoch near the expected encounter (mid-1979)
# so the fixed-Kepler approximation is centered on the encounter epoch.
solar_t0 = datetime(2000, 1, 1, 12, 0, 0)
sc_t0 = datetime(1977, 8, 23, 11, 29, 11)
T_REF_DATE = datetime(1979, 7, 9)                                       # ~ expected perijove
T_REF = (T_REF_DATE - solar_t0).total_seconds() / (86400.0 * 36525.0)   # Julian centuries

el = JUP_TABLE1
a2 = (el['a0'] + el['a_dot']*T_REF) * AU_KM
e2 = el['e0'] + el['e_dot']*T_REF
i2 = np.radians(el['I0'] + el['I_dot']*T_REF)
L_ref = el['L0'] + el['L_dot']*T_REF
lp_ref = el['lp0'] + el['lp_dot']*T_REF
ln_ref = el['ln0'] + el['ln_dot']*T_REF
w2 = np.radians(lp_ref - ln_ref)
Om2 = np.radians(ln_ref)

# M_ref = M at T_REF. Back-propagate to J2000 under pure Kepler with μ_sun + μ_jup.
M_ref_deg = (L_ref - lp_ref) % 360
if M_ref_deg > 180: M_ref_deg -= 360
M_ref = np.radians(M_ref_deg)
n_jup_helio = np.sqrt(mu_jup_eff / a2**3)
t_ref_sec = T_REF * 36525.0 * 86400.0
M02 = M_ref - n_jup_helio * t_ref_sec
M02 = ((M02 + np.pi) % (2*np.pi)) - np.pi

r_soi = a2 * (mu_jup / mu_sun) ** 0.4

b1 = a1 * np.sqrt(1 - e1**2)
b2 = a2 * np.sqrt(1 - e2**2)
T1 = 2 * np.pi * np.sqrt(a1**3 / mu_sun)
T2 = 2 * np.pi * np.sqrt(a2**3 / mu_jup_eff)

# Use solar-system epoch (J2000 per solar.yml) as the reference.
# Propagate M01 from spacecraft epoch (1977-08-23) forward to J2000.
J2000 = solar_t0
t_launch = datetime(1977, 8, 20, 14, 29, 0)  # Voyager 2 launch
M10 = M01_at_sc_epoch + (2 * np.pi / T1) * (J2000 - sc_t0).total_seconds()
M20 = M02
t_start = (t_launch - J2000).total_seconds()  # negative

# --- Rotation matrices (orbital plane → ecliptic) ---
def orb_to_ecl(Om, w, i):
    cO, sO = np.cos(Om), np.sin(Om)
    cw, sw = np.cos(w), np.sin(w)
    ci, si = np.cos(i), np.sin(i)
    col1 = np.array([cO*cw - sO*sw*ci, sO*cw + cO*sw*ci, sw*si])
    col2 = np.array([-cO*sw - sO*cw*ci, -sO*sw + cO*cw*ci, cw*si])
    return np.column_stack([col1, col2])

A = orb_to_ecl(Om1, w1, i1)
B = orb_to_ecl(Om2, w2, i2)
C = 2.0 * A.T @ B
Mmx = np.array([[C[0,0]*a1*a2, C[0,1]*a1*b2],
                [C[1,0]*b1*a2, C[1,1]*b1*b2]])

# --- f, gradient, Hessian ---
def f_val(E1, E2):
    p1 = np.array([np.cos(E1) - e1, np.sin(E1)])
    p2 = np.array([np.cos(E2) - e2, np.sin(E2)])
    r1 = a1 * (1 - e1*np.cos(E1))
    r2 = a2 * (1 - e2*np.cos(E2))
    return r1**2 + r2**2 - p1 @ Mmx @ p2 - r_soi**2

def f_gh(E1, E2):
    p1 = np.array([np.cos(E1) - e1, np.sin(E1)])
    p2 = np.array([np.cos(E2) - e2, np.sin(E2)])
    w1 = np.array([-np.sin(E1), np.cos(E1)])
    w2 = np.array([-np.sin(E2), np.cos(E2)])
    u1 = np.array([np.cos(E1), np.sin(E1)])
    u2 = np.array([np.cos(E2), np.sin(E2)])
    g1 = 2*a1**2*e1*np.sin(E1)*(1 - e1*np.cos(E1)) - w1 @ Mmx @ p2
    g2 = 2*a2**2*e2*np.sin(E2)*(1 - e2*np.cos(E2)) - p1 @ Mmx @ w2
    H11 = 2*a1**2*e1*(np.cos(E1) - e1 + 2*e1*np.sin(E1)**2) + u1 @ Mmx @ p2
    H22 = 2*a2**2*e2*(np.cos(E2) - e2 + 2*e2*np.sin(E2)**2) + p1 @ Mmx @ u2
    H12 = -w1 @ Mmx @ w2
    return np.array([g1, g2]), np.array([[H11, H12], [H12, H22]])

def kepler_M_to_E(M, e):
    E = M if e < 0.8 else np.pi
    for _ in range(50):
        dE = (E - e*np.sin(E) - M) / (1 - e*np.cos(E))
        E -= dE
        if abs(dE) < 1e-14:
            break
    return E

# --- STEP 1: Geometric minimum (§6, seeded from coarse grid) ---
E1c, E2c = np.meshgrid(np.linspace(1.5, 2.8, 60), np.linspace(1.5, 2.5, 60))
Fc = np.vectorize(f_val)(E1c, E2c)
idx = np.unravel_index(np.argmin(Fc), Fc.shape)
E1s, E2s = E1c[idx], E2c[idx]
for _ in range(50):
    g, H = f_gh(E1s, E2s)
    delta = np.linalg.solve(H, -g)
    E1s += delta[0]
    E2s += delta[1]
    if np.linalg.norm(delta) < 1e-14:
        break
f_star = f_val(E1s, E2s)
_, H_star = f_gh(E1s, E2s)
M1s = E1s - e1*np.sin(E1s)
M2s = E2s - e2*np.sin(E2s)

print("=== Step 1: Geometric minimum ===")
print(f"  E1* = {E1s:.6f} rad   M1* = {np.degrees(M1s):.4f}°")
print(f"  E2* = {E2s:.6f} rad   M2* = {np.degrees(M2s):.4f}°")
min_dist = np.sqrt(f_star + r_soi**2)
print(f"  min |Δr| = {min_dist:,.0f} km   r_SOI = {r_soi:,.0f} km   f* = {f_star:.3e}")
assert f_star < 0, "No geometric encounter possible — f* >= 0"

# --- STEP 2-4: pick (k1, k2), intersect, refine; retry k1+1 if needed ---
alpha_star = (T2*(M2s - M20) - T1*(M1s - M10)) / (2*np.pi)
slope = T1 / T2

# Transform Hessian E → M via diagonal Jacobian (reused across k1 attempts)
dM1dE1 = 1 - e1*np.cos(E1s)
dM2dE2 = 1 - e2*np.cos(E2s)
H_M = H_star / np.array([[dM1dE1**2, dM1dE1*dM2dE2],
                         [dM1dE1*dM2dE2, dM2dE2**2]])

def try_k1(k1):
    """Return (t_enc, E1, E2, diagnostics_dict) or None if this k1 fails."""
    k2 = int(round((T1*k1 - alpha_star) / T2))
    alpha = T1*k1 - T2*k2
    c_line = -slope*M10 + M20 + 2*np.pi*alpha/T2

    d0 = slope*M1s + c_line - M2s
    A_q = H_M[0,0] + 2*slope*H_M[0,1] + slope**2*H_M[1,1]
    B_q = 2*d0*(H_M[0,1] + slope*H_M[1,1])
    C_q = d0**2 * H_M[1,1] + 2*f_star
    disc = B_q**2 - 4*A_q*C_q
    diag = {"k1": k1, "k2": k2, "alpha": alpha, "disc": disc}
    if disc <= 0:
        return None, diag

    u_low = (-B_q - np.sqrt(disc)) / (2*A_q)
    M1_guess = M1s + u_low
    M2_guess = slope*M1_guess + c_line
    E1 = kepler_M_to_E(M1_guess, e1)

    def f_along_line(E1):
        M1 = E1 - e1*np.sin(E1)
        M2 = slope*M1 + c_line
        E2 = kepler_M_to_E(M2, e2)
        return f_val(E1, E2), E2

    for _ in range(50):
        fv, _ = f_along_line(E1)
        h = 1e-7
        fp, _ = f_along_line(E1 + h)
        fm, _ = f_along_line(E1 - h)
        dfdE1 = (fp - fm) / (2*h)
        dE1 = -fv / dfdE1
        E1 += dE1
        if abs(dE1) < 1e-13:
            break
    _, E2 = f_along_line(E1)
    M1_enc = E1 - e1*np.sin(E1)
    t_enc = T1/(2*np.pi) * (M1_enc + 2*np.pi*k1 - M10)
    diag["t_enc"] = t_enc
    if t_enc < t_start:
        return None, diag
    return (t_enc, E1, E2, c_line), diag

k1 = int(np.ceil(t_start/T1 - (M1s - M10)/(2*np.pi)))
print("\n=== Steps 2–4: revolution selection + refinement ===")
print(f"  α* = {alpha_star:.3e} s   ceil k1 = {k1}")
result, diag = try_k1(k1)
assert result is not None, f"k1={k1} failed: {diag}"
print(f"  k1={diag['k1']}, k2={diag['k2']}: OK")
t_enc, E1_enc, E2_enc, c_line = result

dist_check = np.linalg.norm(A @ np.array([a1*(np.cos(E1_enc)-e1), b1*np.sin(E1_enc)])
                            - B @ np.array([a2*(np.cos(E2_enc)-e2), b2*np.sin(E2_enc)]))
print(f"\n  selected k1 = {diag['k1']}, k2 = {diag['k2']}")
print(f"  E1 = {E1_enc:.8f}   E2 = {E2_enc:.8f}")
print(f"  ‖Ar1 - Br2‖ = {dist_check:,.3f} km   r_SOI = {r_soi:,.3f} km")
print(f"  t_enc - t_start = {(t_enc - t_start)/86400:.2f} days")

encounter_date = J2000 + timedelta(seconds=t_enc)
print(f"\n  Computed SOI entry: {encounter_date.isoformat()} UTC")
print(f"  Launch:             {t_launch.isoformat()} UTC")

# ====================================================================
# PERIJOVE TIME from state vector at SOI entry
# ====================================================================
# Treat the spacecraft as a Keplerian two-body orbit around Jupiter,
# seeded with the relative state (r, v) at SOI entry.

def state_at(E, a, b, e, A_rot, T):
    """Heliocentric 3D position and velocity for orbit with given rotation."""
    r_plane = np.array([a*(np.cos(E) - e), b*np.sin(E)])
    # dE/dt = n / (1 - e cos E), n = 2π/T
    dEdt = (2*np.pi/T) / (1 - e*np.cos(E))
    v_plane = np.array([-a*np.sin(E), b*np.cos(E)]) * dEdt
    return A_rot @ r_plane, A_rot @ v_plane

r_sc, v_sc = state_at(E1_enc, a1, b1, e1, A, T1)
r_jup, v_jup = state_at(E2_enc, a2, b2, e2, B, T2)
r_rel = r_sc - r_jup
v_rel = v_sc - v_jup
r_norm = np.linalg.norm(r_rel)
v_norm = np.linalg.norm(v_rel)

print("\n=== Perijove computation ===")
print(f"  ‖r_rel‖ at SOI entry = {r_norm:,.0f} km  (r_SOI = {r_soi:,.0f})")
print(f"  ‖v_rel‖ at SOI entry = {v_norm:.4f} km/s")

# Kepler orbit around Jupiter
energy = 0.5 * v_norm**2 - mu_jup / r_norm          # > 0 → hyperbolic
a_jup = -mu_jup / (2 * energy)                      # < 0 for hyperbolic
h_vec = np.cross(r_rel, v_rel)
e_vec = np.cross(v_rel, h_vec) / mu_jup - r_rel / r_norm
ecc = np.linalg.norm(e_vec)
print(f"  a (around Jupiter) = {a_jup:,.0f} km   e = {ecc:.6f}  (hyperbolic: e>1)")

# True anomaly at SOI entry
cos_nu = np.dot(e_vec, r_rel) / (ecc * r_norm)
cos_nu = np.clip(cos_nu, -1.0, 1.0)
nu = np.arccos(cos_nu)
if np.dot(r_rel, v_rel) < 0:      # approaching pericenter → ν < 0
    nu = -nu

# Hyperbolic anomaly
cosh_H = (ecc + np.cos(nu)) / (1 + ecc*np.cos(nu))
H_enc = np.arccosh(cosh_H)
if nu < 0:
    H_enc = -H_enc

# Mean motion and time to pericenter
n_jup = np.sqrt(mu_jup / abs(a_jup)**3)
M_enc = ecc * np.sinh(H_enc) - H_enc                # mean anomaly at SOI entry
dt_to_peri = -M_enc / n_jup                         # >0 if M_enc < 0 (approaching)
t_peri = t_enc + dt_to_peri

perijove_date = J2000 + timedelta(seconds=t_peri)
r_peri = abs(a_jup) * (ecc - 1)                     # hyperbolic pericenter radius

print(f"  ν at entry = {np.degrees(nu):+.3f}°   H = {H_enc:+.4f}")
print(f"  time from SOI entry to perijove = {dt_to_peri/86400:.2f} days")
print(f"  pericenter radius r_p = {r_peri:,.0f} km  ({r_peri/71492:.2f} Jupiter radii)")
print(f"\n  Computed perijove: {perijove_date.isoformat()} UTC")
print(f"  Actual perijove:   1979-07-09 22:29:00 UTC")
delta_h = (perijove_date - datetime(1979, 7, 9, 22, 29, 0)).total_seconds() / 3600
print(f"  Error: {delta_h:+.2f} hours")
