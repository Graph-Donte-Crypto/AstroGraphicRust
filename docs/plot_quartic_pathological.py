"""Plot the pathological case where first_order_e2 fails (147° error).

Geometry: Hohmann transfer from 1 AU to 39.5 AU, i=17°, Ω2=30°, e2=0.25.
At E1 = -65.10°, the dropped harmonic (CC/2)·sin(2E2) is LARGER than the
kept √(AA²+BB²) first harmonic, so the approximation inverts.
"""
import numpy as np
import matplotlib.pyplot as plt

a2 = 39.5; e2 = 0.25
a1 = (1.0 + a2) / 2.0
e1 = (a2 - 1.0) / (a2 + 1.0)
b1 = a1 * np.sqrt(1 - e1**2); b2 = a2 * np.sqrt(1 - e2**2)

inc2 = np.radians(17.0); Om2 = np.radians(30.0)
ci, si = np.cos(inc2), np.sin(inc2)
cO, sO = np.cos(Om2), np.sin(Om2)
A = np.array([[1,0],[0,1],[0,0]], dtype=float)
B = np.array([[cO, -sO*ci], [sO, cO*ci], [0, si]])
C = 2 * A.T @ B
M = np.array([[C[0,0]*a1*a2, C[0,1]*a1*b2],
              [C[1,0]*b1*a2, C[1,1]*b1*b2]])

E1_bad = -1.1362  # = -65.10°
p1 = np.array([np.cos(E1_bad)-e1, np.sin(E1_bad)])
v = M.T @ p1
AA = 2*a2*a2*e2 + v[0]; BB = v[1]; CC = 2*a2*a2*e2*e2

def f_of_E2(E2):
    p2 = np.array([np.cos(E2)-e2, np.sin(E2)])
    r1 = a1*(1-e1*np.cos(E1_bad)); r2 = a2*(1-e2*np.cos(E2))
    return r1*r1 + r2*r2 - p1 @ M @ p2

def df_dE2(E2):
    return AA*np.sin(E2) - (CC/2)*np.sin(2*E2) - BB*np.cos(E2)

def df_dE2_kept(E2):
    return AA*np.sin(E2) - BB*np.cos(E2)

def df_dE2_dropped(E2):
    return -(CC/2)*np.sin(2*E2)

def p_of_t(t):
    return BB*t**4 + 2*(AA+CC)*t**3 + 2*(AA-CC)*t - BB

E2_grid = np.linspace(-np.pi, np.pi, 2000)

# True roots: where df/dE2 = 0 and H22 > 0
# Just find via fine scan and polish
def H22(E2):
    c2, s2 = np.cos(E2), np.sin(E2)
    u2 = np.array([c2, s2])
    return 2*a2*a2*e2*(c2 - e2 + 2*e2*s2*s2) + p1 @ M @ u2

# Find zero-crossings
df_vals = np.vectorize(df_dE2)(E2_grid)
roots = []
for i in range(len(E2_grid)-1):
    if df_vals[i] * df_vals[i+1] < 0:
        # Bisect
        lo, hi = E2_grid[i], E2_grid[i+1]
        for _ in range(60):
            mid = (lo+hi)/2
            if df_dE2(lo)*df_dE2(mid) < 0: hi = mid
            else: lo = mid
        roots.append((lo+hi)/2)

true_min = min([r for r in roots if H22(r) > 0], key=f_of_E2)
true_max = min([r for r in roots if H22(r) < 0], key=lambda r: -f_of_E2(r))

# Approximate roots: where kept harmonic = 0
E2_approx_primary = np.arctan2(BB, AA)
E2_approx_other = E2_approx_primary + np.pi if E2_approx_primary < 0 else E2_approx_primary - np.pi

fig, axes = plt.subplots(3, 1, figsize=(11, 11))

# --- Plot 1: df/dE2 decomposition ---
ax = axes[0]
ax.plot(np.degrees(E2_grid), np.vectorize(df_dE2_kept)(E2_grid),
        'g-', lw=1.5, label=f'kept: AA·sin E₂ − BB·cos E₂  (AA={AA:.1f}, BB={BB:.1f})')
ax.plot(np.degrees(E2_grid), np.vectorize(df_dE2_dropped)(E2_grid),
        'r-', lw=1.5, label=f'dropped: −(CC/2)·sin(2E₂)  (CC={CC:.1f})')
ax.plot(np.degrees(E2_grid), np.vectorize(df_dE2)(E2_grid),
        'k-', lw=2.0, label='total: df/dE₂')
ax.axhline(0, color='gray', lw=0.5)
ax.axvline(np.degrees(E2_approx_primary), color='r', ls='--', alpha=0.7,
           label=f'1ste2 guess (primary): {np.degrees(E2_approx_primary):.1f}°')
ax.axvline(np.degrees(true_min), color='b', ls='--', alpha=0.7,
           label=f'true min: {np.degrees(true_min):.1f}°')
ax.set_xlabel('E₂ (deg)')
ax.set_ylabel('df/dE₂')
ax.set_title(f'Pathological case:  E₁ = {np.degrees(E1_bad):.2f}°,  '
             f'Ω₂=30°, i=17°, Hohmann.\n'
             f'CC/√(AA²+BB²) = {CC/np.sqrt(AA**2+BB**2):.3f} — dropped term is LARGER than kept')
ax.legend(fontsize=9, loc='upper left')
ax.grid(alpha=0.3)
ax.set_xlim(-180, 180)

# --- Plot 2: f(E2) itself with true min and approx guess ---
ax = axes[1]
f_vals = np.vectorize(f_of_E2)(E2_grid)
ax.plot(np.degrees(E2_grid), f_vals, 'k-', lw=1.5)
ax.axvline(np.degrees(E2_approx_primary), color='r', ls='--',
           label=f'1ste2 primary guess: {np.degrees(E2_approx_primary):.1f}°  (f={f_of_E2(E2_approx_primary):.1f})')
ax.axvline(np.degrees(E2_approx_other), color='orange', ls=':',
           label=f'1ste2 opposite branch: {np.degrees(E2_approx_other):.1f}°  (f={f_of_E2(E2_approx_other):.1f})')
ax.axvline(np.degrees(true_min), color='b', ls='--',
           label=f'true minimum: {np.degrees(true_min):.1f}°  (f={f_of_E2(true_min):.1f})')
ax.axvline(np.degrees(true_max), color='g', ls=':', alpha=0.5,
           label=f'true maximum: {np.degrees(true_max):.1f}°  (f={f_of_E2(true_max):.1f})')
ax.set_xlabel('E₂ (deg)')
ax.set_ylabel('f(E₁*, E₂)')
ax.set_title('f restricted to fixed E₁ — approximation lands on the wrong critical point')
ax.legend(fontsize=9, loc='best')
ax.grid(alpha=0.3)
ax.set_xlim(-180, 180)

# --- Plot 3: quartic p(t) vs t ---
ax = axes[2]
t_grid = np.linspace(-6, 6, 2000)
p_vals = np.vectorize(p_of_t)(t_grid)
# True quartic roots
quartic_roots = np.roots([BB, 2*(AA+CC), 0, 2*(AA-CC), -BB])
real_roots_t = [r.real for r in quartic_roots if abs(r.imag) < 1e-9]
ax.plot(t_grid, p_vals, 'k-', lw=1.5, label='full quartic p(t)')
# Also plot the "CC=0" simplified quartic
p_simple = BB*t_grid**4 + 2*AA*t_grid**3 + 2*AA*t_grid - BB
ax.plot(t_grid, p_simple, 'r--', lw=1.2,
        label=f'simplified (CC=0): factors as (t²+1)(BB·t² + 2AA·t − BB)')
ax.axhline(0, color='gray', lw=0.5)
for rt in real_roots_t:
    if abs(rt) < 6:
        ax.plot(rt, 0, 'ko', ms=8)
        e2_rt = 2*np.arctan(rt)
        ax.annotate(f't={rt:.2f}\nE₂={np.degrees(e2_rt):.1f}°',
                    (rt, 0), xytext=(5, 10), textcoords='offset points', fontsize=9)
# Roots of simplified quadratic BB·t² + 2AA·t − BB
disc = AA**2 + BB**2
t_simple_1 = (-AA + np.sqrt(disc))/BB
t_simple_2 = (-AA - np.sqrt(disc))/BB
for ts in (t_simple_1, t_simple_2):
    if abs(ts) < 6:
        ax.plot(ts, 0, 'r^', ms=10, alpha=0.7)
        e2_ts = 2*np.arctan(ts)
        ax.annotate(f't={ts:.2f}\nE₂={np.degrees(e2_ts):.1f}°',
                    (ts, 0), xytext=(5, -30), textcoords='offset points',
                    fontsize=9, color='red')
ax.set_xlabel('t = tan(E₂/2)')
ax.set_ylabel('p(t)')
ax.set_title('Quartic and its CC=0 simplification — root positions differ by ~180° in E₂')
ax.legend(fontsize=9, loc='best')
ax.grid(alpha=0.3)
ax.set_xlim(-6, 6)
ylim = np.max(np.abs(p_vals)) * 1.1
ax.set_ylim(-ylim, ylim)

plt.tight_layout()
plt.savefig('quartic_pathological.png', dpi=130)
plt.savefig('quartic_pathological.svg')
print('Saved quartic_pathological.png and .svg')

print(f"\nAt E₁ = {np.degrees(E1_bad):.2f}°:")
print(f"  AA = {AA:.3f},  BB = {BB:.3f},  CC = {CC:.3f}")
print(f"  √(AA²+BB²) = {np.sqrt(AA**2+BB**2):.3f}")
print(f"  CC / √(AA²+BB²) = {CC / np.sqrt(AA**2+BB**2):.4f}  (should be <<1 for 1ste2 to work)")
print(f"  true min E₂ = {np.degrees(true_min):.2f}°")
print(f"  1ste2 primary guess = {np.degrees(E2_approx_primary):.2f}°  "
      f"(error {np.degrees(abs(E2_approx_primary - true_min)):.2f}°)")
print(f"  1ste2 opposite branch = {np.degrees(E2_approx_other):.2f}°")
