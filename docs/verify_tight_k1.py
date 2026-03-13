"""Find exact (k1, k2) encounter opportunities for Voyager 2 / Jupiter.

Strategy: start from the orbit with the narrower mean-anomaly encounter zone
(here Jupiter/E2), enumerate k2 values, then for each k2 check if any k1
puts the spacecraft in its encounter zone simultaneously.
"""

import numpy as np
from datetime import datetime

# --- Orbital elements ---
a1 = 546095282.4529687   # km
e1 = 0.7331215656362654
M01 = 6.25059  # rad

a2 = 778340816.69271
e2 = 0.04838624
M02 = 0.343270671  # rad

mu_sun = 132712440042.0
n1 = np.sqrt(mu_sun / a1**3)
n2 = np.sqrt(mu_sun / a2**3)

# Propagate M01 to J2000
epoch_sc = datetime(1977, 8, 23, 11, 29, 11)
epoch_j2000 = datetime(2000, 1, 1, 12, 0, 0)
dt_sc_to_j2000 = (epoch_j2000 - epoch_sc).total_seconds()
M01_j2000 = M01 + n1 * dt_sc_to_j2000

SEC_PER_YEAR = 365.25 * 86400

# --- Tight eccentric anomaly bounds from distance constraint (Rust solver) ---
E1_min, E1_max = 2.0976, 2.4251  # rad
E2_min, E2_max = 2.0121, 2.2145  # rad

# --- Mean anomaly bounds: M = E - e sin(E) ---
M1_min = E1_min - e1 * np.sin(E1_min)
M1_max = E1_max - e1 * np.sin(E1_max)
M2_min = E2_min - e2 * np.sin(E2_min)
M2_max = E2_max - e2 * np.sin(E2_max)

print(f"M1 zone: [{np.degrees(M1_min):.2f}°, {np.degrees(M1_max):.2f}°]  width {np.degrees(M1_max-M1_min):.2f}°")
print(f"M2 zone: [{np.degrees(M2_min):.2f}°, {np.degrees(M2_max):.2f}°]  width {np.degrees(M2_max-M2_min):.2f}°")

# --- Find (k1, k2) pairs ---
for year_lo, year_hi in [(1970, 1990), (1900, 2100), (1950, 2050)]:
    t_lo = (year_lo - 2000.0) * SEC_PER_YEAR
    t_hi = (year_hi - 2000.0) * SEC_PER_YEAR

    # Step 1: k2 range from planet's encounter zone
    k2_lo = int(np.ceil((n2 * t_lo + M02 - M2_max) / (2 * np.pi)))
    k2_hi = int(np.floor((n2 * t_hi + M02 - M2_min) / (2 * np.pi)))

    # Step 2: for each k2, find overlapping k1 values
    pairs = []
    for k2 in range(k2_lo, k2_hi + 1):
        # Time interval when planet is in encounter zone on orbit k2
        t2_lo = (M2_min - M02 + 2 * np.pi * k2) / n2
        t2_hi = (M2_max - M02 + 2 * np.pi * k2) / n2
        t2_lo = max(t2_lo, t_lo)
        t2_hi = min(t2_hi, t_hi)
        if t2_lo > t2_hi:
            continue

        # k1 values with spacecraft in encounter zone during [t2_lo, t2_hi]
        k1_lo = int(np.ceil((n1 * t2_lo + M01_j2000 - M1_max) / (2 * np.pi)))
        k1_hi = int(np.floor((n1 * t2_hi + M01_j2000 - M1_min) / (2 * np.pi)))
        for k1 in range(k1_lo, k1_hi + 1):
            lam = n2 * k1 - n1 * k2
            t_approx = (-M01_j2000 + 2 * np.pi * k1) / n1
            yr = 2000.0 + t_approx / SEC_PER_YEAR
            pairs.append((k1, k2, lam, yr))

    print(f"\n[{year_lo}, {year_hi}]: {len(pairs)} encounter opportunities")
    for k1, k2, lam, yr in pairs:
        print(f"  k1={k1}, k2={k2}, λ={lam:.10f}, year≈{yr:.1f}")
