"""Plot the quartic p(t) = BB·t⁴ + 2(AA+CC)·t³ + 2(AA−CC)·t − BB
and df/dE₂(E₂) for fixed E₁, to look for exploitable structure.

Setup: Voyager 2 / Jupiter (same as plot_encounter_M.py).
"""
import numpy as np
import matplotlib.pyplot as plt

# --- Voyager 2 / Jupiter orbital elements ---
a1 = 546095282.4529687
e1 = 0.7331215656362654
i1 = np.radians(5.015)
Om1 = np.radians(-32.9834163461)
w1 = np.radians(12.2919523634)

a2 = 778340816.69271
e2 = 0.04838624
i2 = np.radians(1.30439695)
Om2 = np.radians(100.47390909)
w2 = np.radians(274.25457074)

b1 = a1 * np.sqrt(1 - e1**2)
b2 = a2 * np.sqrt(1 - e2**2)

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
    [C[0,0]*a1*a2, C[0,1]*a1*b2],
    [C[1,0]*b1*a2, C[1,1]*b1*b2],
])

# Three representative E1 values spanning the orbit
E1_cases = [
    ("E1 = 2.2505 rad (encounter)", 2.2505),
    ("E1 = 0 rad (perihelion)",     0.0),
    ("E1 = π/2 rad (quarter)",      np.pi/2),
    ("E1 = -1.0 rad",               -1.0),
]

def quartic_coeffs(E1):
    p1 = np.array([np.cos(E1) - e1, np.sin(E1)])
    v = M.T @ p1   # (v1, v2)
    AA = 2.0 * a2**2 * e2 + v[0]
    BB = v[1]
    CC = 2.0 * a2**2 * e2**2
    return AA, BB, CC

def p_of_t(t, AA, BB, CC):
    return BB*t**4 + 2*(AA+CC)*t**3 + 2*(AA-CC)*t - BB

def df_dE2(E2, E1):
    p1 = np.array([np.cos(E1) - e1, np.sin(E1)])
    w2v = np.array([-np.sin(E2), np.cos(E2)])
    self_term = 2 * a2**2 * e2 * np.sin(E2) * (1 - e2*np.cos(E2))
    cross_term = p1 @ M @ w2v
    return self_term - cross_term

# --- Plot ---
fig, axes = plt.subplots(len(E1_cases), 2, figsize=(13, 4*len(E1_cases)))

for row, (label, E1) in enumerate(E1_cases):
    AA, BB, CC = quartic_coeffs(E1)

    # Quartic in t
    ax = axes[row, 0]
    t = np.linspace(-6, 6, 2000)
    p = p_of_t(t, AA, BB, CC)
    # Normalize to range for readability
    scale = np.max(np.abs(p))
    ax.plot(t, p / scale, 'b-', lw=1.2)
    ax.axhline(0, color='k', lw=0.5)
    ax.axvline(0, color='k', lw=0.5)
    # Mark the real roots found numerically
    roots_t = np.roots([BB, 2*(AA+CC), 0, 2*(AA-CC), -BB])
    real_roots = roots_t[np.abs(roots_t.imag) < 1e-9].real
    for rt in real_roots:
        if abs(rt) < 6:
            ax.plot(rt, 0, 'ro', markersize=8)
            ax.annotate(f't={rt:.2f}\nE2={np.degrees(2*np.arctan(rt)):.1f}°',
                        (rt, 0), xytext=(5, 8), textcoords='offset points',
                        fontsize=8)
    ax.set_xlabel('t = tan(E₂/2)')
    ax.set_ylabel('p(t) (normalized)')
    ax.set_title(f'{label}\nquartic in t:  AA={AA:.3e}  BB={BB:.3e}  CC={CC:.3e}')
    ax.grid(alpha=0.3)
    ax.set_xlim(-6, 6)
    ax.set_ylim(-1.2, 1.2)

    # df/dE2 over E2
    ax = axes[row, 1]
    E2 = np.linspace(-np.pi, np.pi, 2000)
    dfdE2 = np.vectorize(lambda e: df_dE2(e, E1))(E2)
    ax.plot(np.degrees(E2), dfdE2 / np.max(np.abs(dfdE2)), 'g-', lw=1.2)
    ax.axhline(0, color='k', lw=0.5)
    # Roots of df/dE2 — from the quartic roots
    for rt in real_roots:
        E2_root = 2 * np.arctan(rt)
        # wrap to [-pi, pi]
        E2_root = ((E2_root + np.pi) % (2*np.pi)) - np.pi
        ax.plot(np.degrees(E2_root), 0, 'ro', markersize=8)
    ax.set_xlabel('E₂ (deg)')
    ax.set_ylabel('df/dE₂ (normalized)')
    ax.set_title(f'{label} — df/dE₂ vs E₂')
    ax.grid(alpha=0.3)
    ax.set_xlim(-180, 180)
    ax.set_ylim(-1.2, 1.2)

plt.tight_layout()
plt.savefig('quartic_analysis.png', dpi=130)
plt.savefig('quartic_analysis.svg')
print("Saved quartic_analysis.png and .svg")

# --- Structural observations ---
print("\n=== Structural observations ===")
for label, E1 in E1_cases:
    AA, BB, CC = quartic_coeffs(E1)
    print(f"\n{label}:")
    print(f"  AA = {AA:>14.6e}    2a2²e2 = {2*a2**2*e2:.3e}")
    print(f"  BB = {BB:>14.6e}")
    print(f"  CC = {CC:>14.6e}    2a2²e2² = {2*a2**2*e2**2:.3e}")
    print(f"  |CC/AA| = {abs(CC/AA):.3e}     "
          f"(if tiny, quartic ≈ BB·t⁴ + 2AA·t³ + 2AA·t − BB)")
    print(f"  |BB/(2(AA+CC))| = {abs(BB/(2*(AA+CC))):.3e}     "
          f"(if tiny, quartic → cubic dominates)")
    # Are AA+CC and AA-CC close? (They are when CC is small)
    print(f"  (AA+CC)/(AA-CC) = {(AA+CC)/(AA-CC):.6f}    "
          "(if ≈1, the t³ and t coefficients are nearly equal)")
