"""Plot f(E1,E2)=0 and time constraints in (M1, M2) for BepiColombo Arc 3.

Arc 3 = 2nd Mercury flyby (2022-06-23) → 3rd Mercury flyby (2023-06-19).
The arc is a resonant transfer with two distinct geometric encounters per period
pair; each defines its own (α*, Δα) admissibility band.  This figure shows both
minima side-by-side.
"""

import numpy as np
import matplotlib.pyplot as plt
import yaml
from datetime import datetime
from lamberthub import izzo2015

AU_KM = 149597870.7
mu_sun = 132712440042.0

# --- Load Mercury from solar.yml ---
with open("../config/system/solar.yml") as f:
    solar = yaml.safe_load(f)
solar_t0 = datetime(2000, 1, 1, 12, 0, 0)


def planet_elements(name, at_epoch):
    p = next(pl for pl in solar["planets"] if pl["name"] == name)
    o = p["orbit"]
    a = o["a"]
    e = o["e"]
    T = 2 * np.pi * np.sqrt(a**3 / mu_sun)
    dt = (at_epoch - solar_t0).total_seconds()
    M0 = (o["M0"] + (2 * np.pi / T) * dt) % (2 * np.pi)
    return (a, e, np.radians(o["i"]), np.radians(o["Ω"]),
            np.radians(o["ω"]), M0, T, p["μ"])


def orb_to_ecl(Om, w, i):
    cO, sO = np.cos(Om), np.sin(Om)
    cw, sw = np.cos(w), np.sin(w)
    ci, si = np.cos(i), np.sin(i)
    c1 = np.array([cO*cw - sO*sw*ci, sO*cw + cO*sw*ci, sw*si])
    c2 = np.array([-cO*sw - sO*cw*ci, -sO*sw + cO*cw*ci, cw*si])
    return np.column_stack([c1, c2])


def kepler(M_, e_):
    E = M_
    for _ in range(60):
        dE = (E - e_*np.sin(E) - M_) / (1 - e_*np.cos(E))
        E -= dE
        if abs(dE) < 1e-14:
            break
    return E


def state_at(a, e, Om, w, i, M0, T, dt_seconds):
    n = 2*np.pi / T
    M = M0 + n * dt_seconds
    E = kepler(M, e)
    b = a * np.sqrt(1 - e**2)
    r_pf = np.array([a*(np.cos(E) - e), b*np.sin(E)])
    v_factor = np.sqrt(mu_sun * a) / (a*(1 - e*np.cos(E)))
    v_pf = v_factor * np.array([-np.sin(E), np.sqrt(1 - e**2)*np.cos(E)])
    R = orb_to_ecl(Om, w, i)
    return R @ r_pf, R @ v_pf


def orbit_from_rv(r, v, mu):
    rn = np.linalg.norm(r); vn = np.linalg.norm(v)
    h = np.cross(r, v); hn = np.linalg.norm(h)
    k_hat = np.array([0.0, 0.0, 1.0])
    n = np.cross(k_hat, h); nn = np.linalg.norm(n)
    e_vec = ((vn**2 - mu/rn)*r - np.dot(r, v)*v) / mu
    e = np.linalg.norm(e_vec)
    a = 1.0 / (2.0/rn - vn**2/mu)
    i = np.arccos(h[2]/hn)
    Om = np.arctan2(n[1], n[0]) % (2*np.pi) if nn > 1e-12 else 0.0
    w = np.arctan2(np.dot(np.cross(n, e_vec), h)/hn, np.dot(n, e_vec))
    w = w % (2*np.pi) if nn > 1e-12 and e > 1e-12 else 0.0
    nu = np.arctan2(np.dot(np.cross(e_vec, r), h)/hn, np.dot(e_vec, r))
    E = 2*np.arctan2(np.sqrt(1-e)*np.sin(nu/2), np.sqrt(1+e)*np.cos(nu/2))
    M = (E - e*np.sin(E)) % (2*np.pi)
    return a, e, i, Om, w, M


# --- Reconstruct Bepi's arc-3 orbit via Lambert (3 full revs) ---
dt1 = datetime(2022, 6, 23, 9, 44, 0)
dt2 = datetime(2023, 6, 19, 19, 34, 0)
a2, e2, i2, Om2, w2, M02, T2, mu_merc = planet_elements("Mercury", dt1)
r1, _ = state_at(a2, e2, Om2, w2, i2, M02, T2, 0.0)
tof = (dt2 - dt1).total_seconds()
r2, _ = state_at(a2, e2, Om2, w2, i2, M02, T2, tof)
v1, _ = izzo2015(mu_sun, r1, r2, tof, M=3, prograde=True, low_path=True)
a1, e1, i1, Om1, w1, M01 = orbit_from_rv(r1, v1, mu_sun)
b1 = a1 * np.sqrt(1 - e1**2)
b2 = a2 * np.sqrt(1 - e2**2)
T1 = 2 * np.pi * np.sqrt(a1**3 / mu_sun)
r_soi = a2 * (mu_merc / mu_sun) ** 0.4

print(f"T1 = {T1/86400:.3f} d,  T2 = {T2/86400:.3f} d,  T1/T2 = {T1/T2:.6f}")
print(f"r_SOI (Mercury) = {r_soi:,.0f} km")
print(f"Bepi: a={a1/1e6:.3f} Mkm, e={e1:.4f}, i={np.degrees(i1):.3f}°")


A = orb_to_ecl(Om1, w1, i1)
B = orb_to_ecl(Om2, w2, i2)
C = 2.0 * A.T @ B
M = np.array([[C[0, 0]*a1*a2, C[0, 1]*a1*b2],
              [C[1, 0]*b1*a2, C[1, 1]*b1*b2]])


def f_val(E1, E2):
    p1 = np.array([np.cos(E1) - e1, np.sin(E1)])
    p2 = np.array([np.cos(E2) - e2, np.sin(E2)])
    r1v = a1 * (1 - e1 * np.cos(E1))
    r2v = a2 * (1 - e2 * np.cos(E2))
    return r1v**2 + r2v**2 - p1 @ M @ p2 - r_soi**2


def f_grad_hess(E1, E2):
    p1 = np.array([np.cos(E1) - e1, np.sin(E1)])
    p2 = np.array([np.cos(E2) - e2, np.sin(E2)])
    w1v = np.array([-np.sin(E1), np.cos(E1)])
    w2v = np.array([-np.sin(E2), np.cos(E2)])
    u1 = np.array([np.cos(E1), np.sin(E1)])
    u2 = np.array([np.cos(E2), np.sin(E2)])
    g1 = 2*a1**2*e1*np.sin(E1)*(1 - e1*np.cos(E1)) - w1v @ M @ p2
    g2 = 2*a2**2*e2*np.sin(E2)*(1 - e2*np.cos(E2)) - p1 @ M @ w2v
    H11 = 2*a1**2*e1*(np.cos(E1) - e1 + 2*e1*np.sin(E1)**2) + u1 @ M @ p2
    H22 = 2*a2**2*e2*(np.cos(E2) - e2 + 2*e2*np.sin(E2)**2) + p1 @ M @ u2
    H12 = -w1v @ M @ w2v
    return np.array([g1, g2]), np.array([[H11, H12], [H12, H22]])


# --- Find all geometric minima via multi-seed Newton ---
def atan2_seed(E1_):
    r_SC = A @ np.array([a1*(np.cos(E1_) - e1), b1*np.sin(E1_)])
    u = B.T @ r_SC
    return np.arctan2(u[1] / b2, u[0] / a2 + e2)


def inner_min_E2(E1_):
    E2 = atan2_seed(E1_)
    for _ in range(30):
        g, H = f_grad_hess(E1_, E2)
        if abs(H[1, 1]) < 1e-12:
            break
        d = -g[1] / H[1, 1]
        E2 += d
        if abs(d) < 1e-12:
            break
    return f_val(E1_, E2), E2


N1 = 400
E1_grid = np.linspace(-np.pi, np.pi, N1, endpoint=False)
inner = [inner_min_E2(E1) for E1 in E1_grid]
min_f_per_E1 = np.array([v for v, _ in inner])
seed_idx = [i for i in range(N1)
            if min_f_per_E1[i] < 0
            and min_f_per_E1[i] <= min_f_per_E1[(i - 1) % N1]
            and min_f_per_E1[i] <= min_f_per_E1[(i + 1) % N1]]


def refine(E1_, E2_):
    for _ in range(50):
        g, H = f_grad_hess(E1_, E2_)
        try:
            d = np.linalg.solve(H, -g)
        except np.linalg.LinAlgError:
            return None
        E1_ += d[0]; E2_ += d[1]
        if np.linalg.norm(d) < 1e-14:
            break
    return (E1_, E2_) if f_val(E1_, E2_) < 0 else None


minima = []
for i in seed_idx:
    res = refine(E1_grid[i], inner[i][1])
    if res is None:
        continue
    if any(abs(((res[0]-m[0]+np.pi) % (2*np.pi)) - np.pi) < 1e-6
           and abs(((res[1]-m[1]+np.pi) % (2*np.pi)) - np.pi) < 1e-6
           for m in minima):
        continue
    minima.append(res)

print(f"Found {len(minima)} geometric minima.")
for (E1s, E2s) in minima:
    print(f"  E1*={np.degrees(E1s):+.2f}°, E2*={np.degrees(E2s):+.2f}°, "
          f"min dist = {np.sqrt(max(f_val(E1s, E2s) + r_soi**2, 0)):,.0f} km")


# --- Per-minimum Taylor ellipse + admissibility band + local f = 0 contour ---
slope = T1 / T2
M1_line_full = np.array([-np.pi, np.pi])

fig, axes = plt.subplots(1, len(minima), figsize=(9 * len(minima), 9))
if len(minima) == 1:
    axes = [axes]

for ax_idx, ((E1s, E2s), ax) in enumerate(zip(minima, axes)):
    f_min = f_val(E1s, E2s)
    _, H_min = f_grad_hess(E1s, E2s)
    dMdE1 = 1 - e1 * np.cos(E1s)
    dMdE2 = 1 - e2 * np.cos(E2s)
    H_M = H_min / np.array([[dMdE1*dMdE1, dMdE1*dMdE2],
                            [dMdE1*dMdE2, dMdE2*dMdE2]])
    eigvals, eigvecs = np.linalg.eigh(H_M)
    semi = np.sqrt(-2 * f_min / eigvals)
    M1s = E1s - e1 * np.sin(E1s)
    M2s = E2s - e2 * np.sin(E2s)

    t_ell = np.linspace(0, 2*np.pi, 300)
    ell = (semi[0] * np.cos(t_ell)[:, None] * eigvecs[:, 0]
           + semi[1] * np.sin(t_ell)[:, None] * eigvecs[:, 1])
    ell_M1 = M1s + ell[:, 0]
    ell_M2 = M2s + ell[:, 1]

    # Dense local grid for f = 0 contour.  Computed on a wider domain than the
    # display window so the full zero-contour is captured even when the view is
    # cropped tightly around the minimum.
    halfE1_disp = 1.5 * max(semi[0] * dMdE1, semi[1] * dMdE1)
    halfE2_disp = 1.5 * max(semi[0] * dMdE2, semi[1] * dMdE2)
    halfE1 = 4.0 * halfE1_disp
    halfE2 = 4.0 * halfE2_disp
    N_local = 2000
    E1_local = np.linspace(E1s - halfE1, E1s + halfE1, N_local)
    E2_local = np.linspace(E2s - halfE2, E2s + halfE2, N_local)
    E1gL, E2gL = np.meshgrid(E1_local, E2_local)
    p1x = np.cos(E1gL) - e1
    p1y = np.sin(E1gL)
    p2x = np.cos(E2gL) - e2
    p2y = np.sin(E2gL)
    r1L = a1 * (1 - e1 * np.cos(E1gL))
    r2L = a2 * (1 - e2 * np.cos(E2gL))
    crossL = p1x * (M[0, 0] * p2x + M[0, 1] * p2y) + p1y * (M[1, 0] * p2x + M[1, 1] * p2y)
    FL = r1L**2 + r2L**2 - crossL - r_soi**2
    M1gL = E1gL - e1 * np.sin(E1gL)
    M2gL = E2gL - e2 * np.sin(E2gL)

    # Plot bounds — fixed tight window
    x_lo, x_hi = -39.5, -35.5
    y_lo, y_hi = -74.0, -69.0

    ax.contour(np.degrees(M1gL), np.degrees(M2gL), FL, levels=[0],
               colors='black', linewidths=1.5)
    ax.text(0.02, 0.98, '$f(M_1, M_2) = 0$', transform=ax.transAxes,
            fontsize=12, va='top', color='black', fontweight='bold')

    ax.plot(np.degrees(ell_M1), np.degrees(ell_M2), color='red', linewidth=1.5,
            linestyle='--', label='Taylor ellipse')

    # Time-constraint α-lattice lines
    M1_line_deg = np.array([x_lo, x_hi])
    M1_line_rad = np.radians(M1_line_deg)
    k1_center = int(round((T1/(2*np.pi)) * (M1s - M01)  # approximate central k1
                           / (T1/(2*np.pi))))  # just use a wide range
    for k1 in range(-6, 20):
        for k2 in range(-6, 20):
            alpha = T1 * k1 - T2 * k2
            offset = -slope * M01 + M02 + 2 * np.pi * alpha / T2
            M2_line = slope * M1_line_rad + offset
            m2_deg = np.degrees(M2_line)
            if m2_deg.min() > y_hi or m2_deg.max() < y_lo:
                continue
            ax.plot(M1_line_deg, m2_deg, color='gray', linewidth=0.6)
            m2_at_left = np.degrees(slope * M1_line_rad[0] + offset)
            if y_lo <= m2_at_left <= y_hi:
                ax.text(x_lo + 0.02*(x_hi-x_lo), m2_at_left + 0.02*(y_hi-y_lo),
                        f'({k1},{k2})', fontsize=7, color='dimgray', va='bottom')

    # Admissibility band
    alpha_star = (T2 * (M2s - M02) - T1 * (M1s - M01)) / (2 * np.pi)
    A_q = H_M[0, 0] + 2 * slope * H_M[0, 1] + slope**2 * H_M[1, 1]
    det_HM = H_M[0, 0] * H_M[1, 1] - H_M[0, 1]**2
    d0_max = np.sqrt(-2 * A_q * f_min / det_HM)
    delta_alpha = d0_max * T2 / (2 * np.pi)
    off_star = -slope * M01 + M02 + 2*np.pi*alpha_star / T2
    off_plus = -slope * M01 + M02 + 2*np.pi*(alpha_star + delta_alpha) / T2
    off_minus = -slope * M01 + M02 + 2*np.pi*(alpha_star - delta_alpha) / T2

    ax.plot(M1_line_deg, np.degrees(slope * M1_line_rad + off_star),
            color='green', linewidth=1.0, linestyle=':',
            label='α = α*')
    ax.plot(M1_line_deg, np.degrees(slope * M1_line_rad + off_plus),
            color='green', linewidth=1.0, linestyle='--',
            label=f'α = α* ± Δα   (Δα = {delta_alpha/86400:.4f} d)')
    ax.plot(M1_line_deg, np.degrees(slope * M1_line_rad + off_minus),
            color='green', linewidth=1.0, linestyle='--')
    ax.fill_between(M1_line_deg,
                    np.degrees(slope * M1_line_rad + off_minus),
                    np.degrees(slope * M1_line_rad + off_plus),
                    color='green', alpha=0.08)

    ax.set_xlabel('$M_1$ (deg)', fontsize=13)
    ax.set_ylabel('$M_2$ (deg)', fontsize=13)
    ax.set_title(f'Minimum {ax_idx+1}: E1*={np.degrees(E1s):+.2f}°, '
                 f'E2*={np.degrees(E2s):+.2f}°\n'
                 f'f* = {f_min:.2e}, α* = {alpha_star:.2e} s',
                 fontsize=12)
    ax.set_xlim(x_lo, x_hi)
    ax.set_ylim(y_lo, y_hi)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower right', fontsize=9)

fig.suptitle(f'BepiColombo Arc 3: 2nd → 3rd Mercury flyby\n'
             f'T1 = {T1/86400:.2f} d, T2 = {T2/86400:.2f} d, T1/T2 = {T1/T2:.4f} '
             f'— {len(minima)} distinct geometric minima',
             fontsize=13)
plt.tight_layout()
plt.savefig('encounter_contours_bepi_arc3.png', dpi=150)
plt.savefig('encounter_contours_bepi_arc3.svg')
print("Saved encounter_contours_bepi_arc3.png/.svg")
