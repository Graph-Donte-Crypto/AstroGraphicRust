"""Plot f(E1,E2)=0 and time constraints in (M1, M2) for a Parker-like / Venus encounter.

Stress case: short spacecraft period, narrow Taylor ellipse relative to T2/2 lattice gap.
"""

import numpy as np
import matplotlib.pyplot as plt

AU_KM = 149597870.7

# --- Parker-like spacecraft (perihelion 0.046 AU, aphelion 0.80 AU) ---
rp = 0.046 * AU_KM
ra = 0.80 * AU_KM
a1 = 0.5 * (rp + ra)
e1 = (ra - rp) / (ra + rp)
# Coplanar with Venus to force a geometric encounter
i1 = np.radians(3.39468)
Om1 = np.radians(76.67984)
w1 = np.radians(234.92262)       # aphelion aligned with Venus perihelion direction
M01 = 0.0

# --- Venus ---
a2 = 108208000.0
e2 = 0.00677672
i2 = np.radians(3.39468)
Om2 = np.radians(76.67984)
w2 = np.radians(54.92262)
M02 = 0.87814

mu_sun = 132712440042.0
mu_venus = 324858.592
r_soi = a2 * (mu_venus / mu_sun) ** 0.4

b1 = a1 * np.sqrt(1 - e1**2)
b2 = a2 * np.sqrt(1 - e2**2)
T1 = 2 * np.pi * np.sqrt(a1**3 / mu_sun)
T2 = 2 * np.pi * np.sqrt(a2**3 / mu_sun)

print(f"T1 = {T1/86400:.2f} days,  T2 = {T2/86400:.2f} days,  T1/T2 = {T1/T2:.4f}")
print(f"r_SOI (Venus) = {r_soi:,.0f} km")


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
M = np.array([[C[0, 0] * a1 * a2, C[0, 1] * a1 * b2],
              [C[1, 0] * b1 * a2, C[1, 1] * b1 * b2]])

# (coarse grid only used to seed Newton's search for the minimum below)
N = 200
E1g_coarse, E2g_coarse = np.meshgrid(np.linspace(-np.pi, np.pi, N),
                                     np.linspace(-np.pi, np.pi, N))
p1x = np.cos(E1g_coarse) - e1
p1y = np.sin(E1g_coarse)
p2x = np.cos(E2g_coarse) - e2
p2y = np.sin(E2g_coarse)
r1 = a1 * (1 - e1 * np.cos(E1g_coarse))
r2 = a2 * (1 - e2 * np.cos(E2g_coarse))
cross = p1x * (M[0, 0] * p2x + M[0, 1] * p2y) + p1y * (M[1, 0] * p2x + M[1, 1] * p2y)
F_coarse = r1**2 + r2**2 - cross - r_soi**2


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


# --- Find geometric minimum via Newton (seeded from coarse grid) ---
idx = np.unravel_index(np.argmin(F_coarse), F_coarse.shape)
E1s, E2s = E1g_coarse[idx], E2g_coarse[idx]
for _ in range(40):
    g, H = f_grad_hess(E1s, E2s)
    delta = np.linalg.solve(H, -g)
    E1s += delta[0]
    E2s += delta[1]
    if np.linalg.norm(delta) < 1e-14:
        break
f_min = f_val(E1s, E2s)
_, H_min = f_grad_hess(E1s, E2s)
print(f"Geometric minimum at E1*={E1s:.4f}, E2*={E2s:.4f}, min dist = {np.sqrt(max(f_min + r_soi**2, 0)):,.0f} km")

# --- Taylor ellipse in (M1, M2) ---
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
print(f"Taylor ellipse semi-axes (M-domain): {np.degrees(semi[0]):.4f}°, {np.degrees(semi[1]):.4f}°")

# --- Dense local grid for the f = 0 contour around the minimum ---
# Cover a box comfortably larger than the Taylor-ellipse semi-axes.
halfE1 = 6.0 * max(semi[0] * dMdE1, semi[1] * dMdE1)
halfE2 = 6.0 * max(semi[0] * dMdE2, semi[1] * dMdE2)
N_local = 600
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

# --- Plot (tightly cropped around the geometric minimum) ---
x_lo, x_hi = 105.0, 115.0
y_lo, y_hi = -15.0, -5.0

fig, ax = plt.subplots(figsize=(9, 9))

# f = 0 contour (from the dense local grid around the minimum)
ax.contour(np.degrees(M1gL), np.degrees(M2gL), FL, levels=[0],
           colors='black', linewidths=1.5)
ax.text(0.02, 0.98, '$f(M_1, M_2) = 0$', transform=ax.transAxes,
        fontsize=12, va='top', color='black', fontweight='bold')

# Taylor ellipse
ax.plot(np.degrees(ell_M1), np.degrees(ell_M2), color='red', linewidth=1.5,
        linestyle='--', label='Taylor ellipse')

# --- Time constraint lines in (M1, M2) ---
slope = T1 / T2

M1_line = np.array([np.radians(x_lo), np.radians(x_hi)])
for k1 in range(-6, 7):
    for k2 in range(-6, 7):
        alpha = T1 * k1 - T2 * k2
        offset = -slope * M01 + M02 + 2 * np.pi * alpha / T2
        M2_line = slope * M1_line + offset
        m2_deg = np.degrees(M2_line)
        # Skip lines entirely outside the visible box
        if m2_deg.min() > y_hi or m2_deg.max() < y_lo:
            continue
        ax.plot(np.degrees(M1_line), m2_deg, color='gray', linewidth=0.6)
        # label at left edge of visible box
        m2_at_left_deg = np.degrees(slope * np.radians(x_lo) + offset)
        if y_lo <= m2_at_left_deg <= y_hi:
            ax.text(x_lo + 0.5, m2_at_left_deg + 0.3,
                    f'({k1},{k2})', fontsize=8, color='dimgray', va='bottom')

# --- Admissible (k1, k2) line from continued-fraction search ---
k1_adm, k2_adm = 91, 41
alpha_adm = T1 * k1_adm - T2 * k2_adm
offset_adm = -slope * M01 + M02 + 2 * np.pi * alpha_adm / T2
M2_adm_deg = np.degrees(slope * M1_line + offset_adm)
ax.plot(np.degrees(M1_line), M2_adm_deg, color='blue', linewidth=1.8,
        label=f'admissible (k₁, k₂) = ({k1_adm}, {k2_adm})')

# --- Taylor-ellipse admissibility band: α ∈ [α_* − W, α_* + W] ---
# Compute α_* (central line through the minimum) and W (half-width).
alpha_star = (T2 * (M2s - M02) - T1 * (M1s - M01)) / (2 * np.pi)
A_q = H_M[0, 0] + 2 * slope * H_M[0, 1] + slope**2 * H_M[1, 1]
det_HM = H_M[0, 0] * H_M[1, 1] - H_M[0, 1]**2
d0_max = np.sqrt(-2 * A_q * f_min / det_HM)
delta_alpha = d0_max * T2 / (2 * np.pi)

offset_star = -slope * M01 + M02 + 2 * np.pi * alpha_star / T2
offset_plus = -slope * M01 + M02 + 2 * np.pi * (alpha_star + delta_alpha) / T2
offset_minus = -slope * M01 + M02 + 2 * np.pi * (alpha_star - delta_alpha) / T2

ax.plot(np.degrees(M1_line), np.degrees(slope * M1_line + offset_star),
        color='green', linewidth=1.0, linestyle=':', label='α = α* (through minimum)')
ax.plot(np.degrees(M1_line), np.degrees(slope * M1_line + offset_plus),
        color='green', linewidth=1.0, linestyle='--',
        label=f'α = α* ± Δα   (Δα = {delta_alpha/86400:.3f} d)')
ax.plot(np.degrees(M1_line), np.degrees(slope * M1_line + offset_minus),
        color='green', linewidth=1.0, linestyle='--')
# Shade the admissibility band for extra clarity.
ax.fill_between(np.degrees(M1_line),
                np.degrees(slope * M1_line + offset_minus),
                np.degrees(slope * M1_line + offset_plus),
                color='green', alpha=0.08)

ax.set_xlabel('$M_1$ (deg)', fontsize=13)
ax.set_ylabel('$M_2$ (deg)', fontsize=13)
ax.set_title(f'Parker-like (a={a1/AU_KM:.2f} AU, e={e1:.3f}, T1={T1/86400:.0f} d) — Venus\n'
             f'Narrow Taylor ellipse vs. T2/2 α-lattice gap → one-shot pick fails',
             fontsize=13)
ax.set_xlim(x_lo, x_hi)
ax.set_ylim(y_lo, y_hi)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.legend(loc='lower right', fontsize=10)

plt.tight_layout()
plt.savefig('encounter_contours_parker_venus.png', dpi=150)
plt.savefig('encounter_contours_parker_venus.svg')
print("Saved encounter_contours_parker_venus.png/.svg")
