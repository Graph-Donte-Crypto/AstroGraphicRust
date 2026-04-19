"""Visualize why the paper's atan2 works and the focal-ray method doesn't.

Left panel: original coordinates (Sun = focus at origin). An off-ellipse
point q. Two candidate ellipse points:
  P_ray:   focal-ray method (ray from Sun through q hits the ellipse)
  P_paper: paper's atan2 method (eccentric anomaly E₂ → ellipse point)
True minimum-distance point P_true found numerically.

Right panel: after the affine transformation x → x/a + e, y → y/b.
The ellipse becomes a unit circle centered at the new origin (which is
the *ellipse center*, not the Sun — the Sun moves to (e, 0)). In this
frame, the paper's method reduces to "ray from new origin through
transformed q" — i.e. the natural closest-point-on-circle construction.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

# Use a visibly eccentric ellipse
a, e = 1.0, 0.5
b = a * np.sqrt(1 - e**2)

# Ellipse parametrization (focus at origin)
Es = np.linspace(0, 2*np.pi, 400)
ellipse_x = a * (np.cos(Es) - e)
ellipse_y = b * np.sin(Es)

# Sample point q — off-ellipse, off-axis
q = np.array([0.3, 0.8])

# --- Method 1: focal ray (user's method) ---
# ray from origin through q; s > 0 picks same-side intersection.
alpha = (q[0]/a)**2 + (q[1]/b)**2
beta  = q[0]*e/a
gamma = e*e - 1
disc  = np.sqrt(beta*beta - alpha*gamma)
s_plus  = (-beta + disc)/alpha
s_minus = (-beta - disc)/alpha
# Closer-to-q branch (the fix from earlier)
s = s_plus if abs(s_plus - 1) < abs(s_minus - 1) else s_minus
P_ray = s * q
E_ray = np.arctan2(P_ray[1]/b, P_ray[0]/a + e)

# --- Method 2: paper's atan2 ---
E_paper = np.arctan2(q[1]/b, q[0]/a + e)
P_paper = np.array([a*(np.cos(E_paper) - e), b*np.sin(E_paper)])

# --- True closest point on ellipse to q (numerical) ---
def dist2(E):
    p = np.array([a*(np.cos(E) - e), b*np.sin(E)])
    return (p - q) @ (p - q)
Es_fine = np.linspace(0, 2*np.pi, 5000)
E_true = Es_fine[np.argmin([dist2(E) for E in Es_fine])]
# Polish
for _ in range(30):
    eps = 1e-5
    d1 = (dist2(E_true + eps) - dist2(E_true - eps))/(2*eps)
    d2 = (dist2(E_true + eps) - 2*dist2(E_true) + dist2(E_true - eps))/eps**2
    E_true -= d1/d2
P_true = np.array([a*(np.cos(E_true) - e), b*np.sin(E_true)])

# --- Transformed coordinates (paper's hidden transformation) ---
# T(x, y) = (x/a + e, y/b)
def T(p):
    return np.array([p[0]/a + e, p[1]/b])
ellipse_T_x = ellipse_x/a + e
ellipse_T_y = ellipse_y/b
q_T        = T(q)
P_ray_T    = T(P_ray)
P_paper_T  = T(P_paper)
P_true_T   = T(P_true)
sun_T      = T(np.array([0.0, 0.0]))       # Sun moves to (e, 0)
center_T   = T(np.array([-a*e, 0.0]))      # ellipse center moves to origin

# --- Plot ---
fig, axes = plt.subplots(1, 2, figsize=(14, 7))

# ===== LEFT: original coordinates =====
ax = axes[0]
ax.plot(ellipse_x, ellipse_y, 'k-', lw=1.5, label='planet ellipse')
ax.plot(0, 0, 'y*', ms=22, markeredgecolor='orange', label='Sun (focus, origin)')
ax.plot(-a*e, 0, 'k+', ms=18, mew=2, label=f'ellipse center ({-a*e:.2f}, 0)')
ax.plot(*q, 'bo', ms=10, label=f'q = ({q[0]:.2f}, {q[1]:.2f})  [off-ellipse point]')

# Focal ray
t_ray = np.linspace(-1, 5, 2)
ax.plot(t_ray * q[0], t_ray * q[1], 'r--', lw=1.0, alpha=0.7, label='ray from Sun through q')
ax.plot(*P_ray, 'r^', ms=12, label=f'P_ray  (E={np.degrees(E_ray):.1f}°)')

# Paper's point (obtained via atan2 formula, NOT via a ray in original coords)
ax.plot(*P_paper, 'gs', ms=11, label=f'P_paper (E={np.degrees(E_paper):.1f}°)')
ax.plot([q[0], P_paper[0]], [q[1], P_paper[1]], 'g:', lw=1.0, alpha=0.5)

# True closest point
ax.plot(*P_true, 'mD', ms=9, label=f'P_true (E={np.degrees(E_true):.1f}°)  [numerical min]')
ax.plot([q[0], P_true[0]], [q[1], P_true[1]], 'm-', lw=1.3, alpha=0.5)

ax.set_aspect('equal')
ax.grid(alpha=0.3)
ax.axhline(0, color='gray', lw=0.3); ax.axvline(0, color='gray', lw=0.3)
ax.set_title('Original coordinates (Sun at origin)\n'
             f'ellipse: a={a}, e={e}, focus at (0,0), center at ({-a*e:.2f},0)')
ax.legend(loc='lower left', fontsize=9)
ax.set_xlim(-2, 1.6); ax.set_ylim(-1.2, 1.4)

# ===== RIGHT: transformed coordinates =====
ax = axes[1]
ax.plot(ellipse_T_x, ellipse_T_y, 'k-', lw=1.5, label='unit circle (= transformed ellipse)')
ax.plot(0, 0, 'k+', ms=18, mew=2, label='ellipse center (NEW origin)')
ax.plot(*sun_T, 'y*', ms=22, markeredgecolor='orange',
        label=f'Sun (moved to ({sun_T[0]:.2f}, 0))')
ax.plot(*q_T, 'bo', ms=10, label=f'T(q) = ({q_T[0]:.2f}, {q_T[1]:.2f})')

# Ray from NEW origin (ellipse center) through T(q) — this IS the paper's construction
t_ray = np.linspace(-0.2, 2.5, 2)
ax.plot(t_ray * q_T[0], t_ray * q_T[1], 'g--', lw=1.2,
        label='ray from ellipse center through T(q)')
ax.plot(*P_paper_T, 'gs', ms=11,
        label=f'intersection = unit-circle point\n  at angle E={np.degrees(E_paper):.1f}°')

# Also show where the focal-ray method's P_ray lands in this frame
ax.plot(*P_ray_T, 'r^', ms=12, alpha=0.7,
        label=f'P_ray transformed (E={np.degrees(E_ray):.1f}°)')
ax.plot(*P_true_T, 'mD', ms=9, alpha=0.7,
        label=f'P_true transformed (E={np.degrees(E_true):.1f}°)')

# Draw an arrow showing q_T to P_paper_T (perpendicular to circle tangent)
ax.annotate('', xy=P_paper_T, xytext=q_T,
            arrowprops=dict(arrowstyle='->', color='green', lw=1.0, alpha=0.5))

ax.set_aspect('equal')
ax.grid(alpha=0.3)
ax.axhline(0, color='gray', lw=0.3); ax.axvline(0, color='gray', lw=0.3)
ax.set_title('Transformed coords: x → x/a + e, y → y/b\n'
             'Ellipse becomes unit circle; Sun is no longer at origin')
ax.legend(loc='lower left', fontsize=9)
ax.set_xlim(-1.7, 1.7); ax.set_ylim(-1.3, 1.5)

plt.suptitle('Why the paper\'s atan2 works: the hidden affine transformation', fontsize=13)
plt.tight_layout()
plt.savefig('atan2_geometry.png', dpi=140)
plt.savefig('atan2_geometry.svg')
print('Saved atan2_geometry.png and .svg')

print(f"\nDistances from q to each candidate ellipse point:")
print(f"  |q − P_ray|   = {np.linalg.norm(q - P_ray):.4f}   (focal-ray method)")
print(f"  |q − P_paper| = {np.linalg.norm(q - P_paper):.4f}   (paper's atan2)")
print(f"  |q − P_true|  = {np.linalg.norm(q - P_true):.4f}   (true minimum)")
print(f"\nEccentric anomalies:")
print(f"  E_ray   = {np.degrees(E_ray):7.2f}°")
print(f"  E_paper = {np.degrees(E_paper):7.2f}°")
print(f"  E_true  = {np.degrees(E_true):7.2f}°")
