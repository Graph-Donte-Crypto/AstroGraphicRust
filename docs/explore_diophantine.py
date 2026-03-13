"""Find encounter opportunities via extended Euclidean algorithm.

The encounter requires:  k1*T1 - k2*T2 ∈ [Λ_min, Λ_max]

The extended Euclidean algorithm on T1, T2 produces a sequence of
remainders r_i = s_i*T1 + t_i*T2 with |r_i| decreasing. Each remainder
corresponds to a step (dk1, dk2) = (s_i, -t_i) that shifts the residual
k1*T1 - k2*T2 by r_i. Once |r_i| < window width, that step is fine
enough to enumerate all solutions.
"""

import numpy as np
from datetime import datetime
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# --- Orbital elements ---
a1 = 546095282.4529687
e1 = 0.7331215656362654
i1 = np.radians(5.015)
Om1 = np.radians(-32.9834163461)
w1 = np.radians(12.2919523634)
M01 = 6.25059

a2 = 778340816.69271
e2 = 0.04838624
i2 = np.radians(1.30439695)
Om2 = np.radians(100.47390909)
w2 = np.radians(274.25457074)
M02 = 0.343270671

mu_sun = 132712440042.0
b1 = a1 * np.sqrt(1 - e1**2)
b2 = a2 * np.sqrt(1 - e2**2)
n1 = np.sqrt(mu_sun / a1**3)
n2 = np.sqrt(mu_sun / a2**3)
T1 = 2 * np.pi / n1
T2 = 2 * np.pi / n2

epoch_sc = datetime(1977, 8, 23, 11, 29, 11)
epoch_j2000 = datetime(2000, 1, 1, 12, 0, 0)
dt_sc_to_j2000 = (epoch_j2000 - epoch_sc).total_seconds()
M01_j2000 = M01 + n1 * dt_sc_to_j2000

SEC_PER_YEAR = 365.25 * 86400
T1_yr = T1 / SEC_PER_YEAR
T2_yr = T2 / SEC_PER_YEAR

print(f"T1 = {T1_yr:.4f} yr")
print(f"T2 = {T2_yr:.4f} yr")

# --- Build coupling matrix ---
def orb_to_ecl_matrix(Om, w, inc):
    cO, sO = np.cos(Om), np.sin(Om)
    cw, sw = np.cos(w), np.sin(w)
    ci, si = np.cos(inc), np.sin(inc)
    col1 = np.array([cO*cw - sO*sw*ci, sO*cw + cO*sw*ci, sw*si])
    col2 = np.array([-cO*sw - sO*cw*ci, -sO*sw + cO*cw*ci, cw*si])
    return np.column_stack([col1, col2])

A = orb_to_ecl_matrix(Om1, w1, i1)
B = orb_to_ecl_matrix(Om2, w2, i2)
C_mat = 2.0 * A.T @ B
M_mat = np.array([
    [C_mat[0,0]*a1*a2, C_mat[0,1]*a1*b2],
    [C_mat[1,0]*b1*a2, C_mat[1,1]*b1*b2]
])
mu_jup = 1.266865319e8
r_soi = a2 * (mu_jup / mu_sun) ** 0.4

# === Step 1: Compute [Λ_min, Λ_max] from f=0 contour ===

N = 2000
E1_arr = np.linspace(-np.pi, np.pi, N)
E2_arr = np.linspace(-np.pi, np.pi, N)
E1g, E2g = np.meshgrid(E1_arr, E2_arr)
p1x = np.cos(E1g) - e1
p1y = np.sin(E1g)
p2x = np.cos(E2g) - e2
p2y = np.sin(E2g)
r1g = a1 * (1 - e1 * np.cos(E1g))
r2g = a2 * (1 - e2 * np.cos(E2g))
cross = (p1x * (M_mat[0,0]*p2x + M_mat[0,1]*p2y)
         + p1y * (M_mat[1,0]*p2x + M_mat[1,1]*p2y))
Fg = r1g**2 + r2g**2 - cross - r_soi**2

fig_tmp, ax_tmp = plt.subplots()
cs = ax_tmp.contour(E1g, E2g, Fg, levels=[0])
contour_pts = np.concatenate([s for s in cs.allsegs[0] if len(s) > 0])
plt.close(fig_tmp)

E1_c = contour_pts[:, 0]
E2_c = contour_pts[:, 1]
M1_c = E1_c - e1 * np.sin(E1_c)
M2_c = E2_c - e2 * np.sin(E2_c)
Lambda_c = (T2 * (M2_c - M02) - T1 * (M1_c - M01_j2000)) / (2 * np.pi)
Lambda_min = Lambda_c.min()
Lambda_max = Lambda_c.max()
window = Lambda_max - Lambda_min

print(f"\nΛ range: [{Lambda_min/SEC_PER_YEAR:.6f}, {Lambda_max/SEC_PER_YEAR:.6f}] yr  (width {window/SEC_PER_YEAR:.6f} yr)")

# === Step 2: Extended Euclidean algorithm on T1, T2 ===
#
# Produces remainders r_i = s_i*T1 + t_i*T2 with |r_i| decreasing.
# Each remainder corresponds to step (dk1, dk2) = (s_i, -t_i)
# that shifts k1*T1 - k2*T2 by r_i.

print(f"\nExtended Euclidean algorithm on T1, T2:")
print(f"  {'i':>3}  {'s':>6}  {'t':>6}  {'dk1':>6}  {'dk2':>6}  {'remainder':>14}  {'|r| < window':>12}")

r_prev, r_curr = T1, T2
s_prev, s_curr = 1, 0
t_prev, t_curr = 0, 1
steps = [(r_prev, s_prev, t_prev)]

i = 0
print(f"  {i:>3}  {s_prev:>6}  {t_prev:>6}  {s_prev:>6}  {-t_prev:>6}  {r_prev/SEC_PER_YEAR:>14.6f}  {'':>12}")

finest = None
while abs(r_curr) > 1e-6 * SEC_PER_YEAR:
    q = int(r_prev / r_curr)
    r_next = r_prev - q * r_curr
    s_next = s_prev - q * s_curr
    t_next = t_prev - q * t_curr

    r_prev, r_curr = r_curr, r_next
    s_prev, s_curr = s_curr, s_next
    t_prev, t_curr = t_curr, t_next

    i += 1
    fits = abs(r_curr) < window
    print(f"  {i:>3}  {s_curr:>6}  {t_curr:>6}  {s_curr:>6}  {-t_curr:>6}  {r_curr/SEC_PER_YEAR:>14.6f}  {'<--' if fits else '':>12}")

    if fits and finest is None:
        finest = (r_curr, s_curr, t_curr, i)
        # Previous step is the coarse step for finding base solution
        coarse = (r_prev, s_prev, t_prev)

# === Step 3: Find base solution, then enumerate ===

d_fine, s_fine, t_fine, idx = finest
dk1_fine, dk2_fine = s_fine, -t_fine
d_coarse, s_coarse, t_coarse = coarse
dk1_coarse, dk2_coarse = s_coarse, -t_coarse

print(f"\nFine step:   (dk1={dk1_fine:>4}, dk2={dk2_fine:>4}), residual = {d_fine/SEC_PER_YEAR:.6f} yr")
print(f"Coarse step: (dk1={dk1_coarse:>4}, dk2={dk2_coarse:>4}), residual = {d_coarse/SEC_PER_YEAR:.6f} yr")

# Base solution: find m such that m * d_coarse ≈ Λ_mid
Lambda_mid = (Lambda_min + Lambda_max) / 2
m0 = round(Lambda_mid / d_coarse)
k1_base = m0 * dk1_coarse
k2_base = m0 * dk2_coarse
base_res = k1_base * T1 - k2_base * T2

# Adjust with fine steps to land in [Λ_min, Λ_max]
if d_fine > 0:
    j_adj = int(np.ceil((Lambda_min - base_res) / d_fine))
else:
    j_adj = int(np.floor((Lambda_max - base_res) / d_fine))

k1_base += j_adj * dk1_fine
k2_base += j_adj * dk2_fine
base_res = k1_base * T1 - k2_base * T2

print(f"\nBase solution: k1={k1_base}, k2={k2_base}")
print(f"  k1*T1 - k2*T2 = {base_res/SEC_PER_YEAR:.6f} yr")
assert Lambda_min <= base_res <= Lambda_max, "Base solution not in range!"

# Enumerate: step by fine step in both directions
encounters = [(k1_base, k2_base, base_res)]
for direction in [1, -1]:
    j = direction
    while True:
        res = base_res + j * d_fine
        if not (Lambda_min <= res <= Lambda_max):
            break
        k1 = k1_base + j * dk1_fine
        k2 = k2_base + j * dk2_fine
        encounters.append((k1, k2, res))
        j += direction

encounters.sort(key=lambda x: x[0])

print(f"\nAll encounters (one group near base):")
print(f"  {'k1':>6}  {'k2':>6}  {'k1*T1 - k2*T2':>16}  {'approx year':>12}")
for k1, k2, res in encounters:
    t_approx = (-M01_j2000 + 2 * np.pi * k1) / n1
    yr = 2000.0 + t_approx / SEC_PER_YEAR
    print(f"  {k1:>6}  {k2:>6}  {res/SEC_PER_YEAR:>16.6f}  {yr:>12.1f}")

# More groups: shift by coarse step, re-adjust with fine step.
# For each coarse offset g, the residual shifts by g*d_coarse.
# Then adjust with fine steps to land back in [Λ_min, Λ_max].
print(f"\nAll groups (coarse step + fine re-adjustment):")
print(f"  {'k1':>6}  {'k2':>6}  {'k1*T1 - k2*T2':>16}  {'approx year':>12}")

all_encounters = []
for g in range(-10, 11):
    shifted_res = base_res + g * d_coarse
    # How many fine steps to get back into [Λ_min, Λ_max]?
    if d_fine > 0:
        j_lo = int(np.ceil((Lambda_min - shifted_res) / d_fine))
        j_hi = int(np.floor((Lambda_max - shifted_res) / d_fine))
    else:
        j_hi = int(np.floor((Lambda_min - shifted_res) / d_fine))
        j_lo = int(np.ceil((Lambda_max - shifted_res) / d_fine))
    for j in range(j_lo, j_hi + 1):
        res = shifted_res + j * d_fine
        if Lambda_min <= res <= Lambda_max:
            k1 = k1_base + g * dk1_coarse + j * dk1_fine
            k2 = k2_base + g * dk2_coarse + j * dk2_fine
            t_approx = (-M01_j2000 + 2 * np.pi * k1) / n1
            yr = 2000.0 + t_approx / SEC_PER_YEAR
            all_encounters.append((k1, k2, res, yr))

all_encounters.sort(key=lambda x: x[3])
for k1, k2, res, yr in all_encounters:
    print(f"  {k1:>6}  {k2:>6}  {res/SEC_PER_YEAR:>16.6f}  {yr:>12.1f}")
print(f"\nTotal: {len(all_encounters)} encounters")
