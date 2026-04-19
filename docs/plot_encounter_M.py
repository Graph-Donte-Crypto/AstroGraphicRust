"""Plot f(E1,E2)=0 and time constraints in (M1, M2) mean anomaly domain for Voyager 2 / Jupiter encounter."""

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

# Jupiter SOI radius
mu_jup = 1.266865319e8
r_soi = a2 * (mu_jup / mu_sun) ** 0.4

b1 = a1 * np.sqrt(1 - e1**2)
b2 = a2 * np.sqrt(1 - e2**2)

# Orbital periods
T1 = 2 * np.pi * np.sqrt(a1**3 / mu_sun)
T2 = 2 * np.pi * np.sqrt(a2**3 / mu_sun)

# --- Rotation matrices (orbital plane → ecliptic) ---
def orb_to_ecl_matrix(Om, w, i):
    cO, sO = np.cos(Om), np.sin(Om)
    cw, sw = np.cos(w), np.sin(w)
    ci, si = np.cos(i), np.sin(i)
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

A = orb_to_ecl_matrix(Om1, w1, i1)
B = orb_to_ecl_matrix(Om2, w2, i2)

# Coupling matrix C = 2 A^T B, then M = diag(a1,b1) C diag(a2,b2)
C = 2.0 * A.T @ B
M = np.array([
    [C[0, 0] * a1 * a2, C[0, 1] * a1 * b2],
    [C[1, 0] * b1 * a2, C[1, 1] * b1 * b2]
])

# --- Grid in E1, E2 ---
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

# Convert E to M: M_i = E_i - e_i * sin(E_i)
M1g = E1g - e1 * np.sin(E1g)
M2g = E2g - e2 * np.sin(E2g)

# --- Find minimum of f via Newton's method ---
def f_val(E1, E2):
    p1 = np.array([np.cos(E1) - e1, np.sin(E1)])
    p2 = np.array([np.cos(E2) - e2, np.sin(E2)])
    r1v = a1 * (1 - e1 * np.cos(E1))
    r2v = a2 * (1 - e2 * np.cos(E2))
    return r1v**2 + r2v**2 - p1 @ M @ p2 - r_soi**2

def f_grad_hess(E1, E2):
    p1 = np.array([np.cos(E1) - e1, np.sin(E1)])
    p2 = np.array([np.cos(E2) - e2, np.sin(E2)])
    w1 = np.array([-np.sin(E1), np.cos(E1)])
    w2 = np.array([-np.sin(E2), np.cos(E2)])
    u1 = np.array([np.cos(E1), np.sin(E1)])
    u2 = np.array([np.cos(E2), np.sin(E2)])
    g1 = 2*a1**2*e1*np.sin(E1)*(1 - e1*np.cos(E1)) - w1 @ M @ p2
    g2 = 2*a2**2*e2*np.sin(E2)*(1 - e2*np.cos(E2)) - p1 @ M @ w2
    H11 = 2*a1**2*e1*(np.cos(E1) - e1 + 2*e1*np.sin(E1)**2) + u1 @ M @ p2
    H22 = 2*a2**2*e2*(np.cos(E2) - e2 + 2*e2*np.sin(E2)**2) + p1 @ M @ u2
    H12 = -w1 @ M @ w2
    return np.array([g1, g2]), np.array([[H11, H12], [H12, H22]])

# Initial guess from grid minimum
E1_coarse = np.linspace(1.5, 2.8, 100)
E2_coarse = np.linspace(1.5, 2.5, 100)
E1c, E2c = np.meshgrid(E1_coarse, E2_coarse)
Fc = np.vectorize(f_val)(E1c, E2c)
idx = np.unravel_index(np.argmin(Fc), Fc.shape)
E1_star, E2_star = E1c[idx], E2c[idx]

for _ in range(20):
    fv = f_val(E1_star, E2_star)
    g, H = f_grad_hess(E1_star, E2_star)
    delta = np.linalg.solve(H, -g)
    E1_star += delta[0]
    E2_star += delta[1]
    if np.linalg.norm(g) < 1e-12:
        break

f_min = f_val(E1_star, E2_star)
_, H_min = f_grad_hess(E1_star, E2_star)

# Taylor ellipse: delta^T H delta = -2 f_min, in (E1, E2) space
# Convert to (M1, M2) using dM/dE = 1 - e*cos(E) at the minimum
dMdE1 = 1 - e1 * np.cos(E1_star)
dMdE2 = 1 - e2 * np.cos(E2_star)
# H_M = J^{-T} H_E J^{-1} where J = diag(dE/dM) = diag(1/dMdE1, 1/dMdE2)
# => H_M_{ij} = H_E_{ij} / (dMdE_i * dMdE_j)
H_M = H_min / np.array([[dMdE1*dMdE1, dMdE1*dMdE2],
                          [dMdE1*dMdE2, dMdE2*dMdE2]])

# Parametric ellipse from eigendecomposition
eigvals, eigvecs = np.linalg.eigh(H_M)
# semi-axes in (M1, M2): sqrt(-2*f_min / eigval)
semi = np.sqrt(-2 * f_min / eigvals)
M1_star = E1_star - e1 * np.sin(E1_star)
M2_star = E2_star - e2 * np.sin(E2_star)
t_ell = np.linspace(0, 2*np.pi, 200)
ell_local = semi[0] * np.cos(t_ell)[:, None] * eigvecs[:, 0] \
          + semi[1] * np.sin(t_ell)[:, None] * eigvecs[:, 1]
ell_M1 = M1_star + ell_local[:, 0]
ell_M2 = M2_star + ell_local[:, 1]

# --- Relative nodes between the two orbits ---
# Line of nodes direction ell = n1 x n2 (normals to each orbital plane)
n1 = np.cross(A[:, 0], A[:, 1])
n2 = np.cross(B[:, 0], B[:, 1])
ell = np.cross(n1, n2)
ell /= np.linalg.norm(ell)

def solve_node_E(a, b, e, v):
    """Eccentric anomalies where r(E) is parallel/antiparallel to 2D direction v."""
    vx, vy = v
    # a*vy*cos E - b*vx*sin E = a*e*vy
    R = np.hypot(a * vy, b * vx)
    delta = np.arctan2(-b * vx, a * vy)
    arg = np.clip(a * e * vy / R, -1.0, 1.0)
    E_a = delta + np.arccos(arg)
    E_b = delta - np.arccos(arg)
    return E_a, E_b

def classify_nodes(a, b, e, v, E_pair):
    """Return (E_plus, E_minus) sorted by sign of r(E)·v."""
    out = {}
    for E in E_pair:
        x = a * (np.cos(E) - e)
        y = b * np.sin(E)
        s = x * v[0] + y * v[1]
        out['plus' if s > 0 else 'minus'] = E
    return out['plus'], out['minus']

v1 = A.T @ ell
v2 = B.T @ ell

E1_plus, E1_minus = classify_nodes(a1, b1, e1, v1, solve_node_E(a1, b1, e1, v1))
E2_plus, E2_minus = classify_nodes(a2, b2, e2, v2, solve_node_E(a2, b2, e2, v2))

# Wrap E values to [-pi, pi]
def wrap_pi(x):
    return (x + np.pi) % (2 * np.pi) - np.pi

E1_plus, E1_minus = wrap_pi(E1_plus), wrap_pi(E1_minus)
E2_plus, E2_minus = wrap_pi(E2_plus), wrap_pi(E2_minus)

M1_plus = E1_plus - e1 * np.sin(E1_plus)
M1_minus = E1_minus - e1 * np.sin(E1_minus)
M2_plus = E2_plus - e2 * np.sin(E2_plus)
M2_minus = E2_minus - e2 * np.sin(E2_minus)

def pos3d(a_, b_, e_, E_, R_):
    return R_ @ np.array([a_ * (np.cos(E_) - e_), b_ * np.sin(E_)])

r1_plus = pos3d(a1, b1, e1, E1_plus, A)
r1_minus = pos3d(a1, b1, e1, E1_minus, A)
r2_plus = pos3d(a2, b2, e2, E2_plus, B)
r2_minus = pos3d(a2, b2, e2, E2_minus, B)

d_plus = np.linalg.norm(r1_plus - r2_plus)
d_minus = np.linalg.norm(r1_minus - r2_minus)

print(f"Ascending  node (+ell): M1={np.degrees(M1_plus):.3f} deg, "
      f"M2={np.degrees(M2_plus):.3f} deg, |r1-r2|={d_plus:.3e} km "
      f"({d_plus / r_soi:.2f} r_SOI)")
print(f"Descending node (-ell): M1={np.degrees(M1_minus):.3f} deg, "
      f"M2={np.degrees(M2_minus):.3f} deg, |r1-r2|={d_minus:.3e} km "
      f"({d_minus / r_soi:.2f} r_SOI)")

# --- Compare nodal vs radial-overlap initial guess ---
print()
print("=" * 78)
print("Initial-guess comparison for Voyager 2 / Jupiter")
print("=" * 78)
print(f"True minimum (converged Newton): "
      f"E1*={np.degrees(E1_star):.4f} deg, E2*={np.degrees(E2_star):.4f} deg, "
      f"sqrt(|f*|+r_soi^2) - r_soi = {np.sqrt(f_min + r_soi**2) - r_soi:.3e} km")

def E2_from_projection(E1):
    """Paper's projection guess: E_2 from projecting 3D r_1 onto orbit 2's plane."""
    r1_3d = A @ np.array([a1 * (np.cos(E1) - e1), b1 * np.sin(E1)])
    q = B.T @ r1_3d
    return np.arctan2(q[1] / b2, q[0] / a2 + e2)

def newton_from(E1_0, E2_0, tol=1e-12, max_iter=50):
    E1_, E2_ = E1_0, E2_0
    for k in range(1, max_iter + 1):
        g, H = f_grad_hess(E1_, E2_)
        delta = np.linalg.solve(H, -g)
        E1_ += delta[0]; E2_ += delta[1]
        if np.linalg.norm(delta) < tol:
            return E1_, E2_, k
    return E1_, E2_, max_iter

# --- Paper approach: radial overlap -> parabolic interp in E1, projection for E2 ---
lo = (a1 - a2 * (1 + e2) - r_soi) / (a1 * e1)
hi = (a1 - a2 * (1 - e2) + r_soi) / (a1 * e1)
lo_c, hi_c = np.clip(lo, -1, 1), np.clip(hi, -1, 1)
E_hi, E_lo = np.arccos(lo_c), np.arccos(hi_c)   # arccos flips order
print(f"\nRadial overlap: E1 in [{np.degrees(E_lo):.3f}, {np.degrees(E_hi):.3f}] deg "
      f"(and symmetric negative branch)")

def paper_guess(branch_sign):
    xs = branch_sign * np.array([E_lo, 0.5 * (E_lo + E_hi), E_hi])
    fs = np.array([f_val(x, E2_from_projection(x)) for x in xs])
    # parabolic minimiser around x1
    x0, x1, x2 = xs; f0, f1, f2 = fs
    num = (x1 - x0)**2 * (f1 - f2) - (x1 - x2)**2 * (f1 - f0)
    den = (x1 - x0) * (f1 - f2) - (x1 - x2) * (f1 - f0)
    E1_g = x1 - 0.5 * num / den
    return E1_g, E2_from_projection(E1_g)

E1_paper, E2_paper = paper_guess(+1)           # positive branch (the real encounter)
E1_nodal, E2_nodal = E1_plus, E2_plus          # same-side ascending pair

def report(name, E1_0, E2_0):
    f0 = f_val(E1_0, E2_0)
    d0 = np.sqrt(max(f0 + r_soi**2, 0.0))
    dE1 = np.degrees(wrap_pi(E1_0 - E1_star))
    dE2 = np.degrees(wrap_pi(E2_0 - E2_star))
    E1f, E2f, iters = newton_from(E1_0, E2_0)
    converged = (abs(wrap_pi(E1f - E1_star)) < 1e-6
                 and abs(wrap_pi(E2f - E2_star)) < 1e-6)
    print(f"\n[{name}]")
    print(f"  guess: E1={E1_0:.4f} rad ({np.degrees(E1_0):.4f} deg), "
          f"E2={E2_0:.4f} rad ({np.degrees(E2_0):.4f} deg)")
    print(f"  offset from true: dE1={dE1:+.3f} deg, dE2={dE2:+.3f} deg")
    print(f"  separation at guess: {d0:.3e} km ({d0 / r_soi:.3f} r_SOI)")
    print(f"  Newton iters to converge: {iters}  "
          f"{'(OK)' if converged else '(did NOT converge to global min)'}")

report("Paper (radial+parabolic+projection)", E1_paper, E2_paper)
report("Nodal (same-side ascending)",         E1_nodal, E2_nodal)

# --- Plot ---
fig, ax = plt.subplots(figsize=(10, 8))

# f = 0 contour in (M1, M2) coordinates
cs_f = ax.contour(np.degrees(M1g), np.degrees(M2g), F, levels=[0],
                  colors='black', linewidths=1.5)
ax.text(0.02, 0.98, '$f(M_1, M_2) = 0$', transform=ax.transAxes,
        fontsize=12, va='top', color='black', fontweight='bold')

# Taylor approximation ellipse
ax.plot(np.degrees(ell_M1), np.degrees(ell_M2), color='red', linewidth=1.2,
        linestyle='--', label='Taylor ellipse')

# --- Time constraint lines in (M1, M2) ---
# h = T1*(M1 - M01_epoch) - T2*(M2 - M02) + 2*pi*alpha = 0
# where alpha = T1*k1 - T2*k2
# => M2 = (T1/T2)*(M1 - M01_epoch) + M02 + 2*pi*alpha/T2
from datetime import datetime
dt_sc_to_j2000 = (datetime(2000, 1, 1, 12, 0, 0) - datetime(1977, 8, 23, 11, 29, 11)).total_seconds()
M01_epoch = M01 + (2 * np.pi / T1) * dt_sc_to_j2000
# Wrap to [0, 2π] so k₁, k₂ correspond to the physical revolution counts
# from the displayed M₁, M₂ rather than to bookkeeping artifacts of an
# unwrapped mean anomaly.
M01_epoch = M01_epoch % (2 * np.pi)
slope = T1 / T2

M1_line = np.array([0, np.radians(120)])
for k1 in range(-5, 6):
    for k2 in range(-5, 6):
        alpha = T1 * k1 - T2 * k2
        offset = -slope * M01_epoch + M02 + 2 * np.pi * alpha / T2
        M2_line = slope * M1_line + offset
        # skip lines entirely outside visible region
        if M2_line[0] > np.radians(130) or M2_line[-1] < 0:
            continue
        ax.plot(np.degrees(M1_line), np.degrees(M2_line),
                color='gray', linewidth=0.7)
        # label where line enters the visible region
        m2_at_left = offset
        m1_at_bottom = -offset / slope
        if m2_at_left >= 0:
            label_M1, label_M2 = 2, np.degrees(m2_at_left)
        else:
            label_M1, label_M2 = np.degrees(m1_at_bottom) + 1, 2
        if 0 <= label_M1 <= 118 and 0 <= label_M2 <= 128:
            ax.text(label_M1, label_M2,
                    f'({k1},{k2})', fontsize=8, color='dimgray',
                    va='bottom')

# --- Taylor-ellipse admissibility band: α ∈ [α_* − Δα, α_* + Δα] ---
alpha_star = (T2 * (M2_star - M02) - T1 * (M1_star - M01_epoch)) / (2 * np.pi)
A_q = H_M[0, 0] + 2 * slope * H_M[0, 1] + slope**2 * H_M[1, 1]
det_HM = H_M[0, 0] * H_M[1, 1] - H_M[0, 1]**2
d0_max = np.sqrt(-2 * A_q * f_min / det_HM)
delta_alpha = d0_max * T2 / (2 * np.pi)

offset_star = -slope * M01_epoch + M02 + 2 * np.pi * alpha_star / T2
offset_plus = -slope * M01_epoch + M02 + 2 * np.pi * (alpha_star + delta_alpha) / T2
offset_minus = -slope * M01_epoch + M02 + 2 * np.pi * (alpha_star - delta_alpha) / T2

ax.plot(np.degrees(M1_line), np.degrees(slope * M1_line + offset_star),
        color='green', linewidth=1.0, linestyle=':',
        label='α = α* (through minimum)')
ax.plot(np.degrees(M1_line), np.degrees(slope * M1_line + offset_plus),
        color='green', linewidth=1.0, linestyle='--',
        label=f'α = α* ± Δα   (Δα = {delta_alpha/86400:.2f} d)')
ax.plot(np.degrees(M1_line), np.degrees(slope * M1_line + offset_minus),
        color='green', linewidth=1.0, linestyle='--')
ax.fill_between(np.degrees(M1_line),
                np.degrees(slope * M1_line + offset_minus),
                np.degrees(slope * M1_line + offset_plus),
                color='green', alpha=0.12)

# --- Relative node markers ---
for label, M1v, M2v, d in [
    ('asc', M1_plus, M2_plus, d_plus),
    ('desc', M1_minus, M2_minus, d_minus),
]:
    M1d, M2d = np.degrees(M1v), np.degrees(M2v)
    ax.plot(M1d, M2d, marker='*', color='blue', markersize=14,
            markeredgecolor='black', markeredgewidth=0.6, zorder=5)
    ax.annotate(f'{label}\n|Δr|={d:.2e} km',
                xy=(M1d, M2d), xytext=(8, 8), textcoords='offset points',
                fontsize=9, color='blue')

ax.set_xlabel('$M_1$ (deg)', fontsize=13)
ax.set_ylabel('$M_2$ (deg)', fontsize=13)
ax.set_title('Distance constraint $f=0$ in mean anomaly domain\n'
             'Voyager 2 — Jupiter', fontsize=14)
ax.set_xlim(0, 120)
ax.set_ylim(0, 130)
ax.grid(True, alpha=0.3)
ax.legend(loc='lower right', fontsize=9)

plt.tight_layout()
plt.savefig('encounter_contours_M.png', dpi=150)
plt.savefig('encounter_contours_M.svg')
print("Saved encounter_contours_M.png and .svg")
plt.show()
