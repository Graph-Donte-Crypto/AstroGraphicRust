"""Plot f(E1,E2)=0 and h(E1,E2;λ)=0 for Voyager 2 / Jupiter encounter."""

import numpy as np
import matplotlib.pyplot as plt

# --- Orbital elements ---
# Voyager 2 (spacecraft)
a1 = 546095282.4529687   # km
e1 = 0.7331215656362654
i1 = np.radians(5.015)
Om1 = np.radians(-32.9834163461)
w1 = np.radians(12.2919523634)
M01 = 6.25059  # rad

# Jupiter
a2 = 778340816.69271
e2 = 0.04838624
i2 = np.radians(1.30439695)
Om2 = np.radians(100.47390909)
w2 = np.radians(274.25457074)
M02 = 0.343270671  # rad

mu_sun = 132712440042.0  # km^3/s^2

# Jupiter SOI radius: a * (m_planet / m_sun)^(2/5)
mu_jup = 1.266865319e8
r_soi = a2 * (mu_jup / mu_sun) ** 0.4

b1 = a1 * np.sqrt(1 - e1**2)
b2 = a2 * np.sqrt(1 - e2**2)

# Mean motions
n1 = np.sqrt(mu_sun / a1**3)  # rad/s
n2 = np.sqrt(mu_sun / a2**3)

# --- Rotation matrices (orbital plane → ecliptic) ---
def orb_to_ecl_matrix(Om, w, i):
    """3x2 matrix mapping (x_orb, y_orb) to (x,y,z)_ecl."""
    cO, sO = np.cos(Om), np.sin(Om)
    cw, sw = np.cos(w), np.sin(w)
    ci, si = np.cos(i), np.sin(i)
    # Column 1: direction of periapsis in ecliptic
    # Column 2: direction 90° ahead in orbital plane
    col1 = np.array([
        cO * cw - sO * sw * ci,
        sO * cw + cO * sw * ci,
        sw * si
    ])
    col2 = np.array([
        -cO * sw - sO * cw * ci,
        -sO * sw + cO * cw * ci,
        cw * si
    ])
    return np.column_stack([col1, col2])

A = orb_to_ecl_matrix(Om1, w1, i1)  # 3x2
B = orb_to_ecl_matrix(Om2, w2, i2)  # 3x2

# Coupling matrix C = 2 A^T B, then M = diag(a1,b1) C diag(a2,b2)
C = 2.0 * A.T @ B  # 2x2
M = np.array([
    [C[0, 0] * a1 * a2, C[0, 1] * a1 * b2],
    [C[1, 0] * b1 * a2, C[1, 1] * b1 * b2]
])

# --- f(E1, E2) = r1^2 + r2^2 - p1^T M p2 - r_soi^2 ---
def f_val(E1, E2):
    p1 = np.array([np.cos(E1) - e1, np.sin(E1)])
    p2 = np.array([np.cos(E2) - e2, np.sin(E2)])
    r1 = a1 * (1 - e1 * np.cos(E1))
    r2 = a2 * (1 - e2 * np.cos(E2))
    return r1**2 + r2**2 - p1 @ M @ p2 - r_soi**2

# --- h(E1, E2; lam) = n2*(E1 - e1*sin(E1) - M01) - n1*(E2 - e2*sin(E2) - M02) + 2*pi*lam ---
def h_val(E1, E2, lam):
    return (n2 * (E1 - e1 * np.sin(E1) - M01)
            - n1 * (E2 - e2 * np.sin(E2) - M02)
            + 2 * np.pi * lam)

# --- Grid ---
N = 500
E1_arr = np.linspace(-np.pi, np.pi, N)
E2_arr = np.linspace(-np.pi, np.pi, N)
E1g, E2g = np.meshgrid(E1_arr, E2_arr)

# Vectorized f
p1x = np.cos(E1g) - e1
p1y = np.sin(E1g)
p2x = np.cos(E2g) - e2
p2y = np.sin(E2g)
r1 = a1 * (1 - e1 * np.cos(E1g))
r2 = a2 * (1 - e2 * np.cos(E2g))
cross = (p1x * (M[0, 0] * p2x + M[0, 1] * p2y)
         + p1y * (M[1, 0] * p2x + M[1, 1] * p2y))
F = r1**2 + r2**2 - cross - r_soi**2

# --- Find admissible (k1, k2) pairs within time range [1950, 2000] ---
T1 = 2 * np.pi / n1
T2 = 2 * np.pi / n2
print(f"T1 = {T1/86400/365.25:.2f} yr, T2 = {T2/86400/365.25:.2f} yr")
print(f"n1 = {n1:.4e} rad/s, n2 = {n2:.4e} rad/s")
print(f"r_soi = {r_soi:.0f} km")

# Epochs: spacecraft at 1977-08-23, Jupiter at J2000.0
# Use J2000.0 as common reference. Propagate spacecraft M0 to J2000.
from datetime import datetime
epoch_sc = datetime(1977, 8, 23, 11, 29, 11)
epoch_j2000 = datetime(2000, 1, 1, 12, 0, 0)
dt_sc_to_j2000 = (epoch_j2000 - epoch_sc).total_seconds()
M01_j2000 = M01 + n1 * dt_sc_to_j2000  # spacecraft M0 at J2000

# Time in seconds from J2000 for a given (E1, k1):
# t = (E1 - e1*sin(E1) - M01_j2000 + 2*pi*k1) / n1
# Calendar year = 2000.0 + t / (365.25 * 86400)
SEC_PER_YEAR = 365.25 * 86400
year_lo, year_hi = 1970, 1990
t_lo = (year_lo - 2000.0) * SEC_PER_YEAR
t_hi = (year_hi - 2000.0) * SEC_PER_YEAR

# For E1 in [-pi, pi], the mean anomaly M1 = E1 - e1*sin(E1) ranges over [-pi, pi] roughly.
# t = (M1 - M01_j2000 + 2*pi*k1) / n1
# => k1 range: 2*pi*k1 = n1*t + M01_j2000 - M1
# k1 in [floor((n1*t_lo + M01_j2000 - pi)/(2pi)), ceil((n1*t_hi + M01_j2000 + pi)/(2pi))]
k1_lo = int(np.floor((n1 * t_lo + M01_j2000 - np.pi) / (2 * np.pi)))
k1_hi = int(np.ceil((n1 * t_hi + M01_j2000 + np.pi) / (2 * np.pi)))

# Precompute h0 on the grid (with M0s at J2000 epoch)
h0 = (n2 * (E1g - e1 * np.sin(E1g) - M01_j2000)
      - n1 * (E2g - e2 * np.sin(E2g) - M02))

# Vectorized enumeration of (k1, k2) pairs in [1950, 2000]
k1_arr = np.arange(k1_lo, k1_hi + 1)
# Approximate time for each k1 (at E1=0)
t_arr = (-M01_j2000 + 2 * np.pi * k1_arr) / n1
year_arr = 2000.0 + t_arr / SEC_PER_YEAR
# Filter k1 by approximate year
mask = (year_arr >= year_lo - 1) & (year_arr <= year_hi + 1)
k1_valid = k1_arr[mask]
year_valid = year_arr[mask]

# For each valid k1, estimate k2 and try k2_est-1..k2_est+1
unique_lambdas = []
seen = set()
for k1, yr in zip(k1_valid, year_valid):
    t_c = (-M01_j2000 + 2 * np.pi * k1) / n1
    k2_est = int(np.round((n2 * t_c + M02) / (2 * np.pi)))
    for k2 in range(k2_est - 2, k2_est + 3):
        if (k1, k2) not in seen:
            seen.add((k1, k2))
            lam = n2 * k1 - n1 * k2
            unique_lambdas.append((lam, int(k1), k2, float(yr)))
unique_lambdas.sort(key=lambda x: x[3])
print(f"Found {len(unique_lambdas)} time constraint curves in [{year_lo}, {year_hi}]")
for lam, k1, k2, yr in unique_lambdas:
    print(f"  k1={k1:>3}, k2={k2:>3}, λ={lam:>16.10f}, year≈{yr:.1f}")

# --- Plot ---
fig, ax = plt.subplots(figsize=(10, 8))

# f = 0 contour
cs_f = ax.contour(np.degrees(E1g), np.degrees(E2g), F, levels=[0],
                  colors='black', linewidths=1.5)
# Label
ax.text(0.02, 0.98, '$f(E_1, E_2) = 0$', transform=ax.transAxes,
        fontsize=12, va='top', color='black', fontweight='bold')

# h = 0 contours for each lambda, colored by year
cmap = plt.cm.Dark2
years = [yr for _, _, _, yr in unique_lambdas]
yr_min, yr_max = min(years), max(years)
norm = plt.Normalize(yr_min, yr_max)

for lam, k1, k2, yr in unique_lambdas:
    H = h0 + 2 * np.pi * lam
    color = cmap(norm(yr))
    cs = ax.contour(np.degrees(E1g), np.degrees(E2g), H, levels=[0],
                    colors=[color], linewidths=0.7, linestyles='--')
    # Label the curve with the approximate year
    segs = cs.allsegs[0] if cs.allsegs else []
    if segs and len(segs[0]) > 0:
        verts = segs[0]
        mid = verts[len(verts) // 2]
        ax.annotate(f'{yr:.0f}', xy=mid, fontsize=6, color=color, alpha=0.8)

sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])
cbar = plt.colorbar(sm, ax=ax, label='Approximate year', shrink=0.8)

ax.set_xlabel('$E_1$ (deg)', fontsize=13)
ax.set_ylabel('$E_2$ (deg)', fontsize=13)
ax.set_title('Distance constraint $f=0$ and time constraints $h=0$\n'
             f'Voyager 2 — Jupiter, {year_lo}–{year_hi}', fontsize=14)
ax.set_xlim(-180, 180)
ax.set_ylim(-180, 180)
ax.grid(True, alpha=0.3)

ax.text(0.02, 0.02, f'{len(unique_lambdas)} time constraint curves ($h = 0$, dashed)',
        transform=ax.transAxes, fontsize=10, color='gray')

plt.tight_layout()
plt.savefig('encounter_contours.png', dpi=150)
plt.savefig('encounter_contours.svg')
print("Saved encounter_contours.png and .svg")
plt.show()
