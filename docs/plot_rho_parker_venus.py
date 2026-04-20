"""Plot ρ(k1) := T1·k1 − T2·round((T1·k1 − α*)/T2) − α* for the Parker-Venus
encounter, k1 ∈ [0, 100]. k2 follows the ceiling rule @k2_of_k1 and is shown
as an annotation on each point.
"""

import numpy as np
import matplotlib.pyplot as plt

AU_KM = 149597870.700
mu_sun = 132712440042.0
mu_venus = 324858.592

rp = 0.046 * AU_KM
ra = 0.80 * AU_KM
a1 = 0.5 * (rp + ra)
a2 = 108208000.0
r_soi = a2 * (mu_venus / mu_sun) ** 0.4

T1 = 2 * np.pi * np.sqrt(a1**3 / mu_sun)
T2 = 2 * np.pi * np.sqrt(a2**3 / mu_sun)

# α* and Δα from the paper's branch (E1*≈+2.4716, E2*≈−0.1655). The Rust
# solver currently picks the mirror branch (E1*→−E1*, E2*→−E2*) and gets
# α*=+4.449e5 s, but the paper's (k1, k2)=(91, 41) result references this
# branch — using it here so the admissibility check matches the paper.
alpha_star = -5.871451e6
delta_alpha =  3.213920e4

day = 86400.0

print(f'T1 = {T1:.6e} s ({T1/day:.4f} days)')
print(f'T2 = {T2:.6e} s ({T2/day:.4f} days)')
print(f'τ = T1/T2 = {T1/T2:.10f}')
print(f'α* = {alpha_star:.3e} s ({alpha_star/day:.4f} days)')
print(f'Δα = {delta_alpha:.3e} s ({delta_alpha/day:.4f} days)')

# Parametrize by k2 instead — k1 follows from the ceiling rule on T1.
# ρ(k2) = T1·round((T2·k2 + α*)/T1) − T2·k2 − α*.
k2s_int = np.arange(0, 101)
k1s_int = np.round((T2 * k2s_int + alpha_star) / T1).astype(int)
rhos_int = T1 * k1s_int - T2 * k2s_int - alpha_star
ratios_int = np.abs(rhos_int) / delta_alpha
admissible = ratios_int <= 1.0

fig, ax = plt.subplots(figsize=(14, 6))
ax.plot(k2s_int, ratios_int, 'o-', color='steelblue', lw=0.8, ms=4,
        label='|ρ|/Δα')
if admissible.any():
    ax.scatter(k2s_int[admissible], ratios_int[admissible], s=80,
               color='crimson', zorder=5, label='admissible (|ρ|/Δα ≤ 1)')
ax.axhline(1.0, color='crimson', ls='--', lw=0.8, label='|ρ|/Δα = 1')

for k1, k2, r in zip(k1s_int, k2s_int, ratios_int):
    if 0 <= k1 <= 200:
        ax.annotate(f'{k1}', (k2, r), textcoords='offset points', xytext=(0, 6),
                    ha='center', fontsize=7, color='gray')

ax.set_xlabel('k2')
ax.set_ylabel('|ρ| / Δα')
ax.set_title(f'Parker-Venus  |ρ(k2)|/Δα with k1 = round((T2·k2 + α*)/T1)  '
             f'(τ={T1/T2:.6f}, α*/T2={alpha_star/T2:.6f}, Δα/T2={delta_alpha/T2:.6f})\n'
             f'gray labels = k1 at integer k2')
ax.set_xlim(-1, 101)
ax.legend(loc='upper right')
ax.grid(alpha=0.3)

plt.tight_layout()
out_png = 'rho_parker_venus.png'
plt.savefig(out_png, dpi=120)
print(f'saved {out_png}')
print(f'\nAll (k1, k2) pairs for k2 ∈ [0, 100]:')
print(f'  {"k2":>4} {"k1":>6}  {"ρ [s]":>12}  {"ρ/Δα":>10}  admissible')
for k1, k2 in zip(k1s_int, k2s_int):
    r = T1*k1 - T2*k2 - alpha_star
    tag = '  ✓' if abs(r) <= delta_alpha else ''
    print(f'  {k2:>4} {k1:>6}  {r:+12.3e}  {r/delta_alpha:+10.3f}{tag}')
plt.show()
