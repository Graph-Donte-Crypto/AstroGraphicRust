"""3D surface plot of f(E1, E2) showing the z=0 intersection and extremal points."""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- Orbital elements (same as plot_encounter.py) ---
# Voyager 2 (spacecraft)
a1 = 546095282.4529687   # km
e1 = 0.7331215656362654
i1 = np.radians(5.015)
Om1 = np.radians(-32.9834163461)
w1 = np.radians(12.2919523634)

# Jupiter
a2 = 778340816.69271
e2 = 0.04838624
i2 = np.radians(1.30439695)
Om2 = np.radians(100.47390909)
w2 = np.radians(274.25457074)

mu_sun = 132712440042.0  # km^3/s^2
mu_jup = 1.266865319e8
r_soi = a2 * (mu_jup / mu_sun) ** 0.4

b1 = a1 * np.sqrt(1 - e1**2)
b2 = a2 * np.sqrt(1 - e2**2)

# --- Rotation matrices ---
def orb_to_ecl_matrix(Om, w, i):
    cO, sO = np.cos(Om), np.sin(Om)
    cw, sw = np.cos(w), np.sin(w)
    ci, si = np.cos(i), np.sin(i)
    col1 = np.array([cO*cw - sO*sw*ci, sO*cw + cO*sw*ci, sw*si])
    col2 = np.array([-cO*sw - sO*cw*ci, -sO*sw + cO*cw*ci, cw*si])
    return np.column_stack([col1, col2])

A = orb_to_ecl_matrix(Om1, w1, i1)
B = orb_to_ecl_matrix(Om2, w2, i2)

C = 2.0 * A.T @ B
M = np.array([
    [C[0, 0] * a1 * a2, C[0, 1] * a1 * b2],
    [C[1, 0] * b1 * a2, C[1, 1] * b1 * b2]
])

# --- Compute f on grid ---
N = 300
E1_arr = np.linspace(-np.pi, np.pi, N)
E2_arr = np.linspace(-np.pi, np.pi, N)
E1g, E2g = np.meshgrid(E1_arr, E2_arr)

p1x = np.cos(E1g) - e1
p1y = np.sin(E1g)
p2x = np.cos(E2g) - e2
p2y = np.sin(E2g)
r1 = a1 * (1 - e1 * np.cos(E1g))
r2 = a2 * (1 - e2 * np.cos(E2g))
cross = (p1x * (M[0, 0] * p2x + M[0, 1] * p2y)
         + p1y * (M[1, 0] * p2x + M[1, 1] * p2y))
F = r1**2 + r2**2 - cross - r_soi**2

# Normalize for visualization (values are huge in km^2)
F_norm = F / r_soi**2

# --- Find extremal points numerically ---
# Gradient components
def grad_f(E1, E2):
    """Return (g1, g2) = gradient of f."""
    s1, c1 = np.sin(E1), np.cos(E1)
    s2, c2 = np.sin(E2), np.cos(E2)
    p1 = np.array([c1 - e1, s1])
    p2 = np.array([c2 - e2, s2])
    w1v = np.array([-s1, c1])
    w2v = np.array([-s2, c2])
    g1 = 2 * a1**2 * e1 * s1 * (1 - e1 * c1) - w1v @ M @ p2
    g2 = 2 * a2**2 * e2 * s2 * (1 - e2 * c2) - p1 @ M @ w2v
    return g1, g2

def f_val(E1, E2):
    p1 = np.array([np.cos(E1) - e1, np.sin(E1)])
    p2 = np.array([np.cos(E2) - e2, np.sin(E2)])
    r1v = a1 * (1 - e1 * np.cos(E1))
    r2v = a2 * (1 - e2 * np.cos(E2))
    return r1v**2 + r2v**2 - p1 @ M @ p2 - r_soi**2

def hessian_f(E1, E2):
    s1, c1 = np.sin(E1), np.cos(E1)
    s2, c2 = np.sin(E2), np.cos(E2)
    p1 = np.array([c1 - e1, s1])
    p2 = np.array([c2 - e2, s2])
    w1v = np.array([-s1, c1])
    w2v = np.array([-s2, c2])
    u1 = np.array([c1, s1])
    u2 = np.array([c2, s2])
    H11 = 2 * a1**2 * e1 * (c1 - e1 + 2 * e1 * s1**2) + u1 @ M @ p2
    H22 = 2 * a2**2 * e2 * (c2 - e2 + 2 * e2 * s2**2) + p1 @ M @ u2
    H12 = -(w1v @ M @ w2v)
    return H11, H22, H12

# Tolerances: f is in km^2, gradients in km^2/rad — normalize by r_soi^2
f_tol = r_soi**2 * 1e-10
g_tol = r_soi**2 * 1e-8

# Newton's method for {f=0, g1=0} (E2 extrema)
def find_e2_extremum(E1_init, E2_init, max_iter=50):
    E1, E2 = E1_init, E2_init
    for _ in range(max_iter):
        fv = f_val(E1, E2)
        g1, g2 = grad_f(E1, E2)
        H11, H22, H12 = hessian_f(E1, E2)
        det = g1 * H12 - g2 * H11
        if abs(det) < 1e-30:
            return None
        dE1 = (fv * H12 - g1 * g2) / det
        dE2 = (g1**2 - fv * H11) / det
        E1 -= dE1
        E2 -= dE2
        if abs(fv) < f_tol and abs(g1) < g_tol:
            return E1, E2
    return None

# Newton's method for {f=0, g2=0} (E1 extrema)
def find_e1_extremum(E1_init, E2_init, max_iter=50):
    E1, E2 = E1_init, E2_init
    for _ in range(max_iter):
        fv = f_val(E1, E2)
        g1, g2 = grad_f(E1, E2)
        H11, H22, H12 = hessian_f(E1, E2)
        det = g1 * H22 - g2 * H12
        if abs(det) < 1e-30:
            return None
        dE1 = (fv * H22 - g2**2) / det
        dE2 = (fv * H12 - g1 * g2) / det
        E1 -= dE1
        E2 -= dE2
        if abs(fv) < f_tol and abs(g2) < g_tol:
            return E1, E2
    return None

# Search for extremal points using scipy for robustness
from scipy.optimize import fsolve

e1_extrema = []  # points where ∂f/∂E2 = 0 on f=0
e2_extrema = []  # points where ∂f/∂E1 = 0 on f=0

# Find f=0 contour points for initial guesses using a hidden figure
_fig_tmp, _ax_tmp = plt.subplots()
cs_tmp = _ax_tmp.contour(E1g, E2g, F, levels=[0])
contour_pts = [seg for seg in cs_tmp.allsegs[0]]
plt.close(_fig_tmp)

def system_e1_bounds(x):
    """f=0 and ∂f/∂E2=0 (E1 extrema)."""
    fv = f_val(x[0], x[1])
    _, g2 = grad_f(x[0], x[1])
    return [fv, g2]

def system_e2_bounds(x):
    """f=0 and ∂f/∂E1=0 (E2 extrema)."""
    fv = f_val(x[0], x[1])
    g1, _ = grad_f(x[0], x[1])
    return [fv, g1]

for pts in contour_pts:
    n_try = min(50, len(pts))
    indices = np.linspace(0, len(pts) - 1, n_try, dtype=int)
    for idx in indices:
        E1_0, E2_0 = pts[idx]
        res, info, ier, _ = fsolve(system_e1_bounds, [E1_0, E2_0], full_output=True)
        if ier == 1 and abs(res[0]) < 2*np.pi and abs(res[1]) < 2*np.pi:
            e1_extrema.append((res[0], res[1]))
        res, info, ier, _ = fsolve(system_e2_bounds, [E1_0, E2_0], full_output=True)
        if ier == 1 and abs(res[0]) < 2*np.pi and abs(res[1]) < 2*np.pi:
            e2_extrema.append((res[0], res[1]))

# Deduplicate
def dedup(pts, tol=0.01):
    unique = []
    for p in pts:
        if all(abs(p[0] - q[0]) > tol or abs(p[1] - q[1]) > tol for q in unique):
            unique.append(p)
    return unique

e1_extrema = dedup(e1_extrema)
e2_extrema = dedup(e2_extrema)

print(f"E1 extrema (∂f/∂E2=0): {len(e1_extrema)} points")
for p in e1_extrema:
    print(f"  E1={np.degrees(p[0]):.1f}°, E2={np.degrees(p[1]):.1f}°")
print(f"E2 extrema (∂f/∂E1=0): {len(e2_extrema)} points")
for p in e2_extrema:
    print(f"  E1={np.degrees(p[0]):.1f}°, E2={np.degrees(p[1]):.1f}°")

# --- 3D Plot (zoomed into encounter region, gravity-well style) ---

# Determine zoom bounds from the z=1 contour (reuse the full grid)
_f0_center_e1 = np.mean([p[0] for p in e1_extrema + e2_extrema])
_f0_center_e2 = np.mean([p[1] for p in e1_extrema + e2_extrema])
_fig_z1, _ax_z1 = plt.subplots()
_cs_z1 = _ax_z1.contour(E1g, E2g, F_norm, levels=[1.0])
# Pick the contour loop closest to the encounter center
_best_seg = min(_cs_z1.allsegs[0],
                key=lambda s: np.min((s[:, 0] - _f0_center_e1)**2
                                    + (s[:, 1] - _f0_center_e2)**2))
_z1_pts = _best_seg
plt.close(_fig_z1)

_margin = np.radians(1.0)  # small margin so z=1 contour fits inside the grid
e1_lo = _z1_pts[:, 0].min() - _margin
e1_hi = _z1_pts[:, 0].max() + _margin
e2_lo = _z1_pts[:, 1].min() - _margin
e2_hi = _z1_pts[:, 1].max() + _margin
print(f"z=1 bounds: E1=[{np.degrees(e1_lo):.1f}°, {np.degrees(e1_hi):.1f}°], "
      f"E2=[{np.degrees(e2_lo):.1f}°, {np.degrees(e2_hi):.1f}°]")

# Fine grid over zoomed region
Nz = 100
E1z = np.linspace(e1_lo, e1_hi, Nz)
E2z = np.linspace(e2_lo, e2_hi, Nz)
E1zg, E2zg = np.meshgrid(E1z, E2z)

p1xz = np.cos(E1zg) - e1
p1yz = np.sin(E1zg)
p2xz = np.cos(E2zg) - e2
p2yz = np.sin(E2zg)
r1z = a1 * (1 - e1 * np.cos(E1zg))
r2z = a2 * (1 - e2 * np.cos(E2zg))
crossz = (p1xz * (M[0, 0] * p2xz + M[0, 1] * p2yz)
          + p1yz * (M[1, 0] * p2xz + M[1, 1] * p2yz))
Fz = r1z**2 + r2z**2 - crossz - r_soi**2
Fz_norm = Fz / r_soi**2

# Contour on zoomed grid
_fig_tmp2, _ax_tmp2 = plt.subplots()
cs_tmp2 = _ax_tmp2.contour(E1zg, E2zg, Fz, levels=[0])
contour_pts_zoom = [seg for seg in cs_tmp2.allsegs[0]]
plt.close(_fig_tmp2)

from matplotlib.colors import LinearSegmentedColormap

z_lo, z_hi = -1.0, 1.0
Fz_norm_z = Fz / r_soi**2

# Interpolate boundary to exact z_hi crossing, NaN the rest
Fz_plot = Fz_norm_z.copy()
# Along rows: interpolate E1 coordinate at z_hi crossing
for i in range(Fz_plot.shape[0]):
    row = Fz_plot[i, :]
    for j in range(len(row) - 1):
        if row[j] <= z_hi < row[j+1]:
            t = (z_hi - row[j]) / (row[j+1] - row[j])
            E1zg[i, j+1] = E1zg[i, j] + t * (E1zg[i, j+1] - E1zg[i, j])
            E2zg[i, j+1] = E2zg[i, j] + t * (E2zg[i, j+1] - E2zg[i, j])
            Fz_plot[i, j+1] = z_hi
        elif row[j+1] <= z_hi < row[j]:
            t = (z_hi - row[j+1]) / (row[j] - row[j+1])
            E1zg[i, j] = E1zg[i, j+1] + t * (E1zg[i, j] - E1zg[i, j+1])
            E2zg[i, j] = E2zg[i, j+1] + t * (E2zg[i, j] - E2zg[i, j+1])
            Fz_plot[i, j] = z_hi
# Along columns
for j in range(Fz_plot.shape[1]):
    col = Fz_plot[:, j]
    for i in range(len(col) - 1):
        if col[i] <= z_hi < col[i+1]:
            t = (z_hi - col[i]) / (col[i+1] - col[i])
            E1zg[i+1, j] = E1zg[i, j] + t * (E1zg[i+1, j] - E1zg[i, j])
            E2zg[i+1, j] = E2zg[i, j] + t * (E2zg[i+1, j] - E2zg[i, j])
            Fz_plot[i+1, j] = z_hi
        elif col[i+1] <= z_hi < col[i]:
            t = (z_hi - col[i+1]) / (col[i] - col[i+1])
            E1zg[i, j] = E1zg[i+1, j] + t * (E1zg[i, j] - E1zg[i+1, j])
            E2zg[i, j] = E2zg[i+1, j] + t * (E2zg[i, j] - E2zg[i+1, j])
            Fz_plot[i, j] = z_hi
Fz_plot[Fz_plot > z_hi] = np.nan

E1d = np.degrees(E1zg)
E2d = np.degrees(E2zg)

# Find minimum point
min_idx = np.nanargmin(Fz_norm_z)
min_i, min_j = np.unravel_index(min_idx, Fz_norm_z.shape)
E1_min_pt = E1z[min_j]
E2_min_pt = E2z[min_i]


# --- Colormap ---
from matplotlib.colors import LinearSegmentedColormap
from matplotlib import cm
# Lighter version of coolwarm: blend original colors toward white
_base = cm.coolwarm
_colors = _base(np.linspace(0, 1, 256))
_colors[:, :3] = 0.4 * _colors[:, :3] + 0.6  # lighten by blending toward white
_colors = _colors[::-1]  # invert
cmap = LinearSegmentedColormap.from_list('light_coolwarm_r', _colors)

# --- Figure ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Disable auto z-sorting so wireframe (drawn after surface) renders on top
ax.computed_zorder = False

# Surface
ax.plot_surface(E1d, E2d, Fz_plot,
                cmap=cmap, vmin=z_lo, vmax=z_hi,
                rstride=5, cstride=5,
                shade=True, alpha=1.0,
                edgecolor=(0.6, 0.6, 0.7, 0.3), linewidth=0.5, antialiased=True,
                zorder=0)

# --- Wireframe: z=1 contour only ---
wire_color = '#304080'
wire_lw = 1.2

# Use the original (undistorted) zoomed grid for contour extraction
_E1zg0, _E2zg0 = np.meshgrid(E1z, E2z)
_fig_c, _ax_c = plt.subplots()
cs_c = _ax_c.contour(_E1zg0, _E2zg0, Fz_norm_z, levels=[1.0])
plt.close(_fig_c)
for seg in cs_c.allsegs[0]:
    ax.plot(np.degrees(seg[:, 0]), np.degrees(seg[:, 1]),
            np.full(len(seg), 1.0),
            color=wire_color, linewidth=wire_lw, zorder=5)

# # --- Wireframe: contour lines at fixed z-levels ---
# z_levels = [1.0, 0.6, 0.2, -0.2, -0.6]
# for zl in z_levels:
#     _fig_c, _ax_c = plt.subplots()
#     cs_c = _ax_c.contour(E1zg, E2zg, Fz_norm_z, levels=[zl])
#     plt.close(_fig_c)
#     for seg in cs_c.allsegs[0]:
#         ax.plot(np.degrees(seg[:, 0]), np.degrees(seg[:, 1]),
#                 np.full(len(seg), zl),
#                 color=wire_color, linewidth=wire_lw, zorder=5)

# # --- Wireframe: radial meridian lines ---
# N_meridians = 12
# contour_ring = contour_pts_zoom[0]
# diffs_c = np.diff(contour_ring, axis=0)
# seg_lens_c = np.sqrt(diffs_c[:, 0]**2 + diffs_c[:, 1]**2)
# arc = np.concatenate([[0], np.cumsum(seg_lens_c)])
# arc_total = arc[-1]
# arc_targets = np.linspace(0, arc_total, N_meridians, endpoint=False)
# for s in arc_targets:
#     idx = np.searchsorted(arc, s, side='right') - 1
#     idx = min(idx, len(contour_ring) - 2)
#     frac = (s - arc[idx]) / seg_lens_c[idx] if seg_lens_c[idx] > 0 else 0
#     E1_target = contour_ring[idx, 0] + frac * diffs_c[idx, 0]
#     E2_target = contour_ring[idx, 1] + frac * diffs_c[idx, 1]
#     dE1 = E1_target - E1_min_pt
#     dE2 = E2_target - E2_min_pt
#     length = np.sqrt(dE1**2 + dE2**2)
#     if length < 1e-12:
#         continue
#     max_t = 10 * max(e1_hi - e1_lo, e2_hi - e2_lo) / length
#     t = np.linspace(0, max_t, 500)
#     E1_ray = E1_min_pt + t * dE1
#     E2_ray = E2_min_pt + t * dE2
#     if len(E1_ray) < 2:
#         continue
#     p1x_r = np.cos(E1_ray) - e1; p1y_r = np.sin(E1_ray)
#     p2x_r = np.cos(E2_ray) - e2; p2y_r = np.sin(E2_ray)
#     r1_r = a1 * (1 - e1 * np.cos(E1_ray))
#     r2_r = a2 * (1 - e2 * np.cos(E2_ray))
#     cross_r = (p1x_r * (M[0, 0] * p2x_r + M[0, 1] * p2y_r)
#                + p1y_r * (M[1, 0] * p2x_r + M[1, 1] * p2y_r))
#     fv = (r1_r**2 + r2_r**2 - cross_r - r_soi**2) / r_soi**2
#     exceed = np.where(fv > z_hi)[0]
#     if len(exceed) > 0:
#         cut = exceed[0]
#         if cut > 0:
#             t_frac = (z_hi - fv[cut-1]) / (fv[cut] - fv[cut-1])
#             E1_end = E1_ray[cut-1] + t_frac * (E1_ray[cut] - E1_ray[cut-1])
#             E2_end = E2_ray[cut-1] + t_frac * (E2_ray[cut] - E2_ray[cut-1])
#             E1_seg = np.append(E1_ray[:cut], E1_end)
#             E2_seg = np.append(E2_ray[:cut], E2_end)
#             fv_seg = np.append(fv[:cut], z_hi)
#         else:
#             continue
#     else:
#         E1_seg, E2_seg, fv_seg = E1_ray, E2_ray, fv
#     if len(fv_seg) >= 2:
#         ax.plot(np.degrees(E1_seg), np.degrees(E2_seg), fv_seg,
#                 color=wire_color, linewidth=wire_lw, zorder=5)

# --- f=0 contour ring (thicker) ---
for pts in contour_pts_zoom:
    ax.plot(np.degrees(pts[:, 0]), np.degrees(pts[:, 1]),
            np.zeros(len(pts)),
            color=wire_color, linewidth=wire_lw, zorder=10)

# --- Minimum (well bottom) ---
ax.plot([np.degrees(E1_min_pt)], [np.degrees(E2_min_pt)],
        [Fz_norm_z[min_i, min_j]],
        'o', color='black', markersize=4, zorder=20)

# --- Extremal points ---
for p in e1_extrema:
    ax.scatter(np.degrees(p[0]), np.degrees(p[1]), 0,
               color='red', s=20, zorder=15)

for p in e2_extrema:
    ax.scatter(np.degrees(p[0]), np.degrees(p[1]), 0,
               color='#00b386', s=20, zorder=15)

# --- Axes ---
ax.set_xlabel('$E_1$ (deg)', fontsize=11, labelpad=8)
ax.set_ylabel('$E_2$ (deg)', fontsize=11, labelpad=8)
ax.set_zlabel('')
padding = 1
ax.set_xlim(np.degrees(e1_lo) - padding, np.degrees(e1_hi) + padding)
ax.set_ylim(np.degrees(e2_lo) - padding, np.degrees(e2_hi) + padding)
ax.set_zlim(z_lo, z_hi)
# Manual z-axis label (set_zlabel gets clipped by bbox_inches='tight')
ax.text2D(1.02, 0.72, '$f / r_{\\mathrm{SOI}}^2$', fontsize=11,
          transform=ax.transAxes, ha='left', va='center')

# Integer ticks
e1_lo_d, e1_hi_d = np.degrees(e1_lo), np.degrees(e1_hi)
e2_lo_d, e2_hi_d = np.degrees(e2_lo), np.degrees(e2_hi)
ax.set_xticks(np.arange(np.ceil(e1_lo_d / 5) * 5, e1_hi_d, 5))
ax.set_yticks(np.arange(np.ceil(e2_lo_d / 5) * 5, e2_hi_d, 5))
ax.set_zticks(np.arange(z_lo, z_hi + 0.01, 0.5))

ax.tick_params(labelsize=9)
ax.view_init(elev=40, azim=-50, roll=0)
ax.set_proj_type('ortho')

fig.tight_layout()
fig.savefig('encounter_surface_3d.png', dpi=200, bbox_inches='tight')
print("Saved encounter_surface_3d.png")
plt.show()
