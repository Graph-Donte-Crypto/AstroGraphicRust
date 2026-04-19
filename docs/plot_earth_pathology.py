"""Visualise the f(E1, E2) surface for the problematic Voyager 2 / Earth case.

Earth's SOI is tiny compared to the spacecraft's approach distance, so the
positive-E1 branch contains no interior local minimum. The constrained optimum
on that branch sits exactly at the inner radial-crossing edge (E1 = lo), which
is why Halley overshoots and the solver has to fall back to gradient descent.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# --- Orbital elements (from config/config.yml + config/system/solar.yml) ---
# Voyager 2 heliocentric orbit just before Jupiter arrival.
a1 = 546095282.4529687  # km
e1 = 0.7331215656362654
i1 = np.radians(5.015)
Om1 = np.radians(-32.9834163461)
w1 = np.radians(12.2919523634)

# Earth
a2 = 149598023.0
e2 = 0.0167086
i2 = np.radians(0.00005)
Om2 = np.radians(-11.26064)
w2 = np.radians(114.20783)

mu_sun = 132712440042.0
mu_earth = 398600.435507
r_soi = a2 * (mu_earth / mu_sun) ** 0.4

b1 = a1 * np.sqrt(1 - e1**2)
b2 = a2 * np.sqrt(1 - e2**2)


def orb_to_ecl_matrix(Om, w, i):
    cO, sO = np.cos(Om), np.sin(Om)
    cw, sw = np.cos(w), np.sin(w)
    ci, si = np.cos(i), np.sin(i)
    col1 = np.array([cO * cw - sO * sw * ci, sO * cw + cO * sw * ci, sw * si])
    col2 = np.array([-cO * sw - sO * cw * ci, -sO * sw + cO * cw * ci, cw * si])
    return np.column_stack([col1, col2])


A = orb_to_ecl_matrix(Om1, w1, i1)
B = orb_to_ecl_matrix(Om2, w2, i2)
C = 2.0 * A.T @ B
M = np.array(
    [[C[0, 0] * a1 * a2, C[0, 1] * a1 * b2], [C[1, 0] * b1 * a2, C[1, 1] * b1 * b2]]
)


def dist_grid(E1g, E2g):
    """Vectorised 3D separation √(r1² + r2² − p1ᵀMp2) over a grid."""
    p1x = np.cos(E1g) - e1
    p1y = np.sin(E1g)
    p2x = np.cos(E2g) - e2
    p2y = np.sin(E2g)
    r1 = a1 * (1 - e1 * np.cos(E1g))
    r2 = a2 * (1 - e2 * np.cos(E2g))
    cross = p1x * (M[0, 0] * p2x + M[0, 1] * p2y) + p1y * (
        M[1, 0] * p2x + M[1, 1] * p2y
    )
    return np.sqrt(np.maximum(r1**2 + r2**2 - cross, 0.0))


# --- Branch intervals (from radial_initial_guesses) ---
lo_raw = (a1 + r_soi - a2 * (1 - e2)) / (a1 * e1)
hi_raw = (a1 - r_soi - a2 * (1 + e2)) / (a1 * e1)
lo = np.arccos(np.clip(lo_raw, -1.0, 1.0))
hi = np.arccos(np.clip(hi_raw, -1.0, 1.0))
print(f"Branch intervals: positive=[{lo:.4f}, {hi:.4f}], negative=[{-hi:.4f}, {-lo:.4f}]")
print(f"r_soi = {r_soi:.0f} km")

# --- Key iterates observed from the Rust solver (Earth seed 2 + converged minima) ---
seed1 = (-0.11875442825411689, -2.4474380961665076)
seed2 = (0.11875442825411689, -1.8425884312057759)
min1 = (-0.152316004955, -2.531790253136)   # converged from seed 1
min2 = (0.046499611391, -2.025275398)        # converged from seed 2 (bounded to lo)

# GD trajectory from the last Rust test run (seed 2).
seed2_path = np.array([
    (0.118754428254, -1.842588431206),
    (0.063784488648, -1.980932784461),
    (0.075421924011, -1.951321786852),   # Halley overshoot
    (0.050316188836, -2.018007730103),   # GD kicks in
    (0.048597123887, -2.017426025929),
    (0.049664198209, -2.017924654731),
    (0.049089082700, -2.017779252572),
    (0.049317945187, -2.017941324346),
    (0.046499611391, -2.027650173425),
    (0.046499611391, -2.022958108803),
    (0.047664185349, -2.023514088988),
    (0.046888933525, -2.023286590517),
    (0.047290479837, -2.023554844051),
    (0.046821605551, -2.023477591839),
    (0.047027470232, -2.023617162827),
    (0.046499611391, -2.025303065846),
    (0.046499611391, -2.025265301431),
    (0.046499611391, -2.025281755956),
    (0.046499611391, -2.025272152136),
    (0.046499611391, -2.025275398785),
])
# Seed 1 Halley path.
seed1_path = np.array([
    (-0.118754428254, -2.447438096167),
    (-0.148009079879, -2.520806608529),
    (-0.152310509560, -2.531776235975),
    (-0.152316004955, -2.531790253136),
])


# --- Plot ---
fig, axes = plt.subplots(1, 2, figsize=(15, 6.5))

# -- Left: full overview in degrees --
ax = axes[0]
N = 600
E1_arr = np.linspace(-np.pi, np.pi, N)
E2_arr = np.linspace(-np.pi, np.pi, N)
E1g, E2g = np.meshgrid(E1_arr, E2_arr)
D = dist_grid(E1g, E2g) / 1e6  # Mm
# Levels capped at √2 · d_min  (⇔ f ≤ 2 · f_min) — only the basin around the
# global minimum is drawn, everything further out is left blank.
d_min_full = float(D.min())
levels = np.linspace(d_min_full, np.sqrt(2) * d_min_full, 12)
ax.contour(
    np.degrees(E1g), np.degrees(E2g), D, levels=levels,
    colors="black", linewidths=0.6,
)
ax.contour(
    np.degrees(E1g), np.degrees(E2g), D, levels=[r_soi / 1e6],
    colors="red", linewidths=1.5,
)
ax.axvspan(np.degrees(lo), np.degrees(hi), color="cyan", alpha=0.2, label="+ branch")
ax.axvspan(np.degrees(-hi), np.degrees(-lo), color="magenta", alpha=0.2, label="− branch")
ax.plot(*np.degrees(seed1), "*", color="tab:blue", markersize=12, markeredgecolor="k", label="seed 1 (−)")
ax.plot(*np.degrees(seed2), "*", color="tab:orange", markersize=12, markeredgecolor="k", label="seed 2 (+)")
ax.plot(*np.degrees(min1), "o", color="tab:blue", markersize=8, markeredgecolor="k")
ax.plot(*np.degrees(min2), "o", color="tab:orange", markersize=8, markeredgecolor="k")
ax.set_xlabel("E₁ (deg)")
ax.set_ylabel("E₂ (deg)")
ax.set_title(
    f"Voyager 2 → Earth, d(E₁,E₂) over full domain\n"
    f"red contour = r_soi = {r_soi/1e3:.0f}×10³ km"
)
ax.legend(loc="upper right", fontsize=9)
ax.set_xlim(-180, 180)
ax.set_ylim(-180, 180)

# -- Right: zoom on the problematic band --
ax = axes[1]
N2 = 500
E1_z = np.linspace(-0.25, 0.25, N2)
E2_z = np.linspace(-3.0, -1.5, N2)
E1gz, E2gz = np.meshgrid(E1_z, E2_z)
Dz = dist_grid(E1gz, E2gz) / 1e6
# Levels capped at √2 · d_min  (⇔ f ≤ 2 · f_min) on the zoom window.
d_min_zoom = float(Dz.min())
levels_z = np.linspace(d_min_zoom, np.sqrt(2) * d_min_zoom, 15)
ax.contour(E1gz, E2gz, Dz, levels=levels_z, colors="black", linewidths=0.6)
ax.contour(E1gz, E2gz, Dz, levels=[r_soi / 1e6], colors="red", linewidths=1.5)

# Branch boxes (vertical bands × E2 extent of the plot).
ax.add_patch(
    Rectangle(
        (lo, -3.0), hi - lo, 1.5,
        linewidth=1.5, edgecolor="cyan", facecolor="cyan", alpha=0.15,
        label="+ branch allowed E₁",
    )
)
ax.add_patch(
    Rectangle(
        (-hi, -3.0), hi - lo, 1.5,
        linewidth=1.5, edgecolor="magenta", facecolor="magenta", alpha=0.15,
        label="− branch allowed E₁",
    )
)

# Trajectories.
ax.plot(
    seed1_path[:, 0], seed1_path[:, 1], "-",
    color="tab:blue", linewidth=1.5, alpha=0.9, label="seed 1 Halley path",
)
ax.plot(
    seed1_path[:, 0], seed1_path[:, 1], "o",
    color="tab:blue", markersize=3, markeredgecolor="k",
)
ax.plot(
    seed2_path[:, 0], seed2_path[:, 1], "-",
    color="tab:orange", linewidth=1.5, alpha=0.9, label="seed 2 Halley+GD path",
)
ax.plot(
    seed2_path[:, 0], seed2_path[:, 1], "o",
    color="tab:orange", markersize=3, markeredgecolor="k",
)

# Seeds and minima.
ax.plot(*seed1, "*", color="tab:blue", markersize=14, markeredgecolor="k", zorder=5)
ax.plot(*seed2, "*", color="tab:orange", markersize=14, markeredgecolor="k", zorder=5)
ax.plot(*min1, "o", color="tab:blue", markersize=10, markeredgecolor="k", zorder=5)
ax.plot(*min2, "o", color="tab:orange", markersize=10, markeredgecolor="k", zorder=5)
ax.annotate(
    "seed 1", seed1, xytext=(6, 6), textcoords="offset points",
    color="tab:blue", fontweight="bold",
)
ax.annotate(
    "seed 2", seed2, xytext=(6, 6), textcoords="offset points",
    color="tab:orange", fontweight="bold",
)
ax.annotate(
    f"min 1\nd={dist_grid(np.array([[min1[0]]]), np.array([[min1[1]]]))[0, 0]/1e6:.2f}×10⁶ km",
    min1, xytext=(8, -20), textcoords="offset points", color="tab:blue",
)
ax.annotate(
    f"min 2 (constrained)\nd={dist_grid(np.array([[min2[0]]]), np.array([[min2[1]]]))[0, 0]/1e6:.2f}×10⁶ km",
    min2, xytext=(8, 8), textcoords="offset points", color="tab:orange",
)

ax.set_xlabel("E₁ (rad)")
ax.set_ylabel("E₂ (rad)")
ax.set_title(
    "Zoom on radial-crossing band. Note there is NO\n"
    "interior local minimum on the + branch — min 2 sits\n"
    "at E₁ = lo (left wall of the cyan box)."
)
ax.legend(loc="lower left", fontsize=8)
ax.set_xlim(-0.25, 0.25)
ax.set_ylim(-3.0, -1.5)

plt.tight_layout()
out = "/tmp/earth_pathology.png"
plt.savefig(out, dpi=130, bbox_inches="tight")
print(f"saved {out}")
plt.show()
