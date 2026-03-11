"""Why f₂=0 gives E₁ bounds and f₁=0 gives E₂ bounds."""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

# Use a tilted ellipse as a stand-in for the level curve f(E₁,E₂)=0
# f(x,y) = (x-3)²/4 + (y-3)²/2 + 0.3(x-3)(y-3) - 1
cx, cy = 3.0, 3.0

def f(x, y):
    dx, dy = x - cx, y - cy
    return dx**2 / 4 + dy**2 / 2 + 0.3 * dx * dy - 1

def f1(x, y):  # ∂f/∂E₁ (partial w.r.t. x)
    return (x - cx) / 2 + 0.3 * (y - cy)

def f2(x, y):  # ∂f/∂E₂ (partial w.r.t. y)
    return (y - cy) + 0.3 * (x - cx)

# Sample the level curve
t = np.linspace(0, 2 * np.pi, 1000)
# Parametric form (found by diagonalizing the quadratic)
E = np.array([[1/4, 0.15], [0.15, 1/2]])
eigvals, eigvecs = np.linalg.eigh(E)
a, b = 1.0 / np.sqrt(eigvals)
curve_x = cx + eigvecs[0, 0] * a * np.cos(t) + eigvecs[0, 1] * b * np.sin(t)
curve_y = cy + eigvecs[1, 0] * a * np.cos(t) + eigvecs[1, 1] * b * np.sin(t)

# Find points where f₂=0 (E₁ extremes — vertical tangents)
f2_vals = f2(curve_x, curve_y)
sign_changes_f2 = np.where(np.diff(np.sign(f2_vals)))[0]

# Find points where f₁=0 (E₂ extremes — horizontal tangents)
f1_vals = f1(curve_x, curve_y)
sign_changes_f1 = np.where(np.diff(np.sign(f1_vals)))[0]

fig, ax = plt.subplots(figsize=(7, 7))

# Shade the interior (f ≤ 0 region)
ax.fill(curve_x, curve_y, alpha=0.10, color='C0', label='f(E₁,E₂) ≤ 0')

# Level curve
ax.plot(curve_x, curve_y, 'C0', lw=2, label='f(E₁,E₂) = 0')

# Mark f₂=0 points (E₁ projection boundaries)
for idx in sign_changes_f2:
    px, py = curve_x[idx], curve_y[idx]
    ax.plot(px, py, 'o', color='C3', ms=10, zorder=5)
    # Vertical dashed line down to E₁ axis
    ax.plot([px, px], [0.5, py], '--', color='C3', lw=1.2)
    # Tangent line (vertical, since dE₁/dE₂=0)
    ax.plot([px, px], [py - 0.6, py + 0.6], '-', color='C3', lw=2, alpha=0.6)

# Mark f₁=0 points (E₂ projection boundaries)
for idx in sign_changes_f1:
    px, py = curve_x[idx], curve_y[idx]
    ax.plot(px, py, 's', color='C2', ms=10, zorder=5)
    # Horizontal dashed line to E₂ axis
    ax.plot([0.5, px], [py, py], '--', color='C2', lw=1.2)
    # Tangent line (horizontal, since dE₂/dE₁=0)
    ax.plot([px - 0.6, px + 0.6], [py, py], '-', color='C2', lw=2, alpha=0.6)

# Draw E₁ interval on x-axis
e1_pts = [curve_x[i] for i in sign_changes_f2]
if len(e1_pts) >= 2:
    e1_min, e1_max = min(e1_pts), max(e1_pts)
    ax.annotate('', xy=(e1_max, 0.7), xytext=(e1_min, 0.7),
                arrowprops=dict(arrowstyle='<->', color='C3', lw=2.5))
    ax.text((e1_min + e1_max) / 2, 0.45, 'E₁ interval',
            ha='center', fontsize=11, color='C3', fontweight='bold')

# Draw E₂ interval on y-axis
e2_pts = [curve_y[i] for i in sign_changes_f1]
if len(e2_pts) >= 2:
    e2_min, e2_max = min(e2_pts), max(e2_pts)
    ax.annotate('', xy=(0.7, e2_max), xytext=(0.7, e2_min),
                arrowprops=dict(arrowstyle='<->', color='C2', lw=2.5))
    ax.text(0.45, (e2_min + e2_max) / 2, 'E₂\ninterval',
            ha='center', va='center', fontsize=11, color='C2', fontweight='bold')

# Dummy entries for legend
ax.plot([], [], 'o', color='C3', ms=8, label='f₂=0  (vertical tangent → E₁ bounds)')
ax.plot([], [], 's', color='C2', ms=8, label='f₁=0  (horizontal tangent → E₂ bounds)')

ax.set_xlabel('E₁', fontsize=13)
ax.set_ylabel('E₂', fontsize=13)
ax.set_title('Projection of level curve onto E₁ and E₂ axes', fontsize=13)
ax.set_xlim(0.3, 5.7)
ax.set_ylim(0.3, 5.7)
ax.set_aspect('equal')
ax.legend(loc='upper right', fontsize=10)
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('docs/explain_projection.png', dpi=150)
plt.show()
