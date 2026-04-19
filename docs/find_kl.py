"""Continued-fraction algorithm for finding (k_1, k_2) revolution pair
that places both bodies at their geometric-minimum anomalies simultaneously.

Adapted from Visser (2023), §3.4 + §6.1, to our paper's terminology.

Paper notation  →  Our notation
    (k, l)      →  (k_1, k_2)
    T_1, T_2    →  same (paper's is slower body 1, we re-order at the top)
    t_i^1       →  t_i^*  (times at geometric-min anomaly, not nodal-crossing)
    r_i         →  3D position at E_i^* (from Newton minimum, not linearized node)
    v_i         →  velocity at E_i^*
    d           →  d^*  (min 3D separation from Newton)
    δ           →  dimensionless half-width of the time-mismatch admissibility strip

The paper's derivation uses a *tangent-line* linearization; we use the
(stricter) second-order Taylor ellipse from our paper §7.2. For
SOI-sized encounters the two agree to within r_SOI/a.
"""

import math
import numpy as np
from datetime import datetime, timedelta

AU_KM = 149597870.7
MU_SUN = 132712440042.0


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def orb_to_ecl(Om, w, i):
    cO, sO = np.cos(Om), np.sin(Om)
    cw, sw = np.cos(w), np.sin(w)
    ci, si = np.cos(i), np.sin(i)
    col1 = np.array([cO*cw - sO*sw*ci, sO*cw + cO*sw*ci, sw*si])
    col2 = np.array([-cO*sw - sO*cw*ci, -sO*sw + cO*cw*ci, cw*si])
    return np.column_stack([col1, col2])


def coupling_matrix(a1, b1, a2, b2, A, B):
    C = 2.0 * A.T @ B
    return np.array([
        [C[0, 0]*a1*a2, C[0, 1]*a1*b2],
        [C[1, 0]*b1*a2, C[1, 1]*b1*b2],
    ])


def f_grad_hess(E1, E2, a1, e1, a2, e2, M, r_soi_sq):
    s1, c1 = np.sin(E1), np.cos(E1)
    s2, c2 = np.sin(E2), np.cos(E2)
    p1 = np.array([c1 - e1, s1])
    p2 = np.array([c2 - e2, s2])
    w1 = np.array([-s1, c1])
    w2 = np.array([-s2, c2])
    u1 = np.array([c1, s1])
    u2 = np.array([c2, s2])
    r1 = a1 * (1 - e1 * c1)
    r2 = a2 * (1 - e2 * c2)
    f = r1*r1 + r2*r2 - p1 @ M @ p2 - r_soi_sq
    g = np.array([
        2*a1*a1*e1*s1*(1 - e1*c1) - w1 @ M @ p2,
        2*a2*a2*e2*s2*(1 - e2*c2) - p1 @ M @ w2,
    ])
    H = np.array([
        [2*a1*a1*e1*(c1 - e1 + 2*e1*s1*s1) + u1 @ M @ p2,
         -w1 @ M @ w2],
        [-w1 @ M @ w2,
         2*a2*a2*e2*(c2 - e2 + 2*e2*s2*s2) + p1 @ M @ u2],
    ])
    return f, g, H


def newton_minimum(E1_0, E2_0, a1, e1, a2, e2, M, r_soi_sq, max_iter=50, tol=1e-14):
    E1, E2 = E1_0, E2_0
    for _ in range(max_iter):
        _, g, H = f_grad_hess(E1, E2, a1, e1, a2, e2, M, r_soi_sq)
        d = np.linalg.solve(H, -g)
        E1 += d[0]; E2 += d[1]
        if np.linalg.norm(d) < tol:
            break
    return E1, E2


def orbital_state_3d(a, e, E, epoch_offset, A, mu):
    """Return (r_3d, v_3d, M_wrapped, t_at_E) where t_at_E is the time
    at which this body reaches eccentric anomaly E counting from its epoch
    (when M = epoch_offset), taking the first forward passage if needed."""
    b = a * np.sqrt(1 - e*e)
    r_2d = np.array([a*(np.cos(E) - e), b*np.sin(E)])
    # Velocity: d(r_2d)/dt = d(r_2d)/dE · dE/dt, with dE/dt = n / (1 - e cos E)
    n = np.sqrt(mu / a**3)
    dr_dE = np.array([-a*np.sin(E), b*np.cos(E)])
    v_2d = dr_dE * n / (1 - e*np.cos(E))
    r_3d = A @ r_2d
    v_3d = A @ v_2d
    M = E - e*np.sin(E)
    # Time since M = epoch_offset (always take positive forward passage)
    dM = (M - epoch_offset) % (2*np.pi)
    t = dM / n
    return r_3d, v_3d, M, t


# ---------------------------------------------------------------------------
# Continued-fraction (k, l) search
# ---------------------------------------------------------------------------

def find_kl_continued_fraction(
    T_slow, T_fast,           # paper convention: T_slow > T_fast
    t_slow_star, t_fast_star, # times of geometric-min passage from epoch (forward)
    u_vec, w_vec,             # v_fast - v_slow, v_slow × v_fast (3D)
    d_star, r_soi,
    k_slow_min=0,             # minimum allowed k_slow (non-negative from epoch)
    t_sim_window=None,        # optional simulation horizon (seconds from epoch)
    max_iter=30,
    verbose=True,
    trace=False,              # detailed per-basis output
):
    """Find smallest (k_slow, k_fast) ∈ ℕ² with
        |t_slow_star + k_slow·T_slow − t_fast_star − k_fast·T_fast| < δ · |t_slow_star − t_fast_star|
    where δ comes from the tangent-line tolerance on 3D separation ≤ r_soi.

    Returns dict with keys: k_slow, k_fast, t_collision, delta, iterations.
    """
    assert T_slow > T_fast, "paper convention: body 1 must be slower"
    slack_sq = r_soi*r_soi - d_star*d_star
    if slack_sq < 0:
        if verbose:
            print(f"  NO ENCOUNTER: d* = {d_star:.3e} km > r_SOI = {r_soi:.3e} km")
        return None

    # Paper's algorithm requires the strip at +1 (i.e. t_slow* < t_fast*). If the
    # fast body reaches its encounter position first, shift it forward by integer
    # multiples of T_fast until it's past the slow body; record the shift so we
    # can restore the real k_fast at the end.
    k_fast_shift = 0
    while t_slow_star > t_fast_star:
        t_fast_star += T_fast
        k_fast_shift += 1

    dt = t_fast_star - t_slow_star                 # now strictly positive
    u_sq = float(u_vec @ u_vec)
    w_sq = float(w_vec @ w_vec)
    phys_tol = math.sqrt(u_sq * slack_sq / w_sq)   # seconds
    delta = phys_tol / dt if dt > 0 else float('inf')
    p = T_slow / dt
    q = T_fast / dt

    if verbose:
        shift_note = f"   (k_fast shifted by +{k_fast_shift} so strip sits at +1)" if k_fast_shift else ""
        print(f"  d* = {d_star:.4e} km (≤ r_SOI = {r_soi:.3e} km ✓)")
        print(f"  |t_slow* − t_fast*| = {dt:.4e} s = {dt/86400:.3f} days{shift_note}")
        print(f"  |u| = {math.sqrt(u_sq):.4e} km/s   |w| = {math.sqrt(w_sq):.4e} km²/s²")
        print(f"  physical tolerance on |α − α*|: {phys_tol:.4e} s = {phys_tol/86400:.3f} days")
        print(f"  δ (dimensionless) = {delta:.4e}")
        print(f"  p = T_slow/dt = {p:.4f}   q = T_fast/dt = {q:.4f}   p/q = {p/q:.6f}")

    # Degenerate guards
    if not (delta > 0) or not math.isfinite(delta):
        if verbose:
            print(f"  ABORT: delta is non-positive or non-finite ({delta})")
        return None
    if delta > 1e3:
        if verbose:
            print(f"  NOTE: δ = {delta:.2e} is very large — clipping x-strip to 32 samples")

    # Continued fraction: q_0=p, q_1=q, k_0=1, k_1=0. Even/odd pair iteration.
    q_seq = [p, q]
    k_seq = [1, 0]

    X_STRIP_CAP = 32   # hard cap on integer-point tests per basis

    for n in range(max_iter):
        q0, q1 = q_seq[2*n], q_seq[2*n+1]
        k0, k1 = k_seq[2*n], k_seq[2*n+1]

        x_lo_raw = math.ceil((1 - delta) / q0)
        if n > 0:
            x_lo_raw = max(1, x_lo_raw)
        x_lo_raw = max(0, x_lo_raw)
        x_hi_raw = max(x_lo_raw, math.ceil((1 + delta) / q0))
        x_hi = min(x_hi_raw, x_lo_raw + X_STRIP_CAP - 1)

        if trace:
            extra = "" if x_hi == x_hi_raw else f"  (capped from {x_hi_raw})"
            print(f"  [basis {n}]  q_2n={q0:.4f}, q_2n+1={q1:.4f}, "
                  f"k_2n={k0}, k_2n+1={k1},  x ∈ [{x_lo_raw}, {x_hi}]{extra}")

        for x in range(x_lo_raw, x_hi + 1):
            if q1 > 1e-30:
                y = max(0, math.ceil((q0*x - 1 - delta) / q1))
            else:
                y = 0
            val = x * q0 - y * q1
            k_slow = x * k0 - y * k1
            if trace:
                print(f"      x={x}, y={y}  →  val = {val:.4e}  "
                      f"(strip [{1-delta:.4e}, {1+delta:.4e}]),  k_slow = {k_slow}")
            if 1 - delta < val < 1 + delta and k_slow >= k_slow_min:
                t_coll = t_slow_star + k_slow * T_slow   # note: t_slow_star unchanged by shift
                if t_sim_window is not None and t_coll > t_sim_window:
                    continue
                k_fast_search = round((t_coll - t_fast_star) / T_fast)
                # Restore the original k_fast by removing the shift we added.
                k_fast = k_fast_search + k_fast_shift
                return {
                    "k_slow": k_slow, "k_fast": k_fast,
                    "t_collision": t_coll, "delta": delta,
                    "iterations": n + 1,
                }

        # Advance continued fraction by one (even, odd) pair.
        if q1 <= 1e-30:
            break
        a_even = int(q0 // q1)
        q2 = q0 - a_even * q1
        k2 = k0 - a_even * k1
        q_seq.append(q2); k_seq.append(k2)
        if abs(q2) < 1e-30:
            if verbose:
                print("  remainder 0 → p/q rational (mean-motion resonance)")
            break

        a_odd = int(q1 // q2)
        q3 = q1 - a_odd * q2
        k3 = k1 - a_odd * k2
        q_seq.append(q3); k_seq.append(k3)
        if abs(q3) < 1e-30:
            if verbose:
                print("  remainder 0 → resonance")
            break

    if verbose:
        print("  exhausted max_iter without finding (k, l)")
    return None


# ---------------------------------------------------------------------------
# Scenario runner
# ---------------------------------------------------------------------------

def run_scenario(name, epoch_dt, sc, pl, search_horizon_days=None):
    print("=" * 72)
    print(f"  {name}")
    print("=" * 72)
    print(f"Epoch: {epoch_dt.isoformat()}")

    a1, e1, i1, Om1, w1_ang, M01 = sc
    a2, e2, i2, Om2, w2_ang, M02, r_soi = pl
    b1 = a1*math.sqrt(1 - e1*e1)
    b2 = a2*math.sqrt(1 - e2*e2)
    T1 = 2*np.pi*math.sqrt(a1**3/MU_SUN)
    T2 = 2*np.pi*math.sqrt(a2**3/MU_SUN)
    A = orb_to_ecl(Om1, w1_ang, i1)
    B = orb_to_ecl(Om2, w2_ang, i2)
    M = coupling_matrix(a1, b1, a2, b2, A, B)
    r_soi_sq = r_soi*r_soi

    print(f"Spacecraft:  a={a1/AU_KM:.4f} AU, e={e1:.4f},  T={T1/86400:.2f} days")
    print(f"Planet:      a={a2/AU_KM:.4f} AU, e={e2:.4f},  T={T2/86400:.2f} days")
    print(f"r_SOI = {r_soi:,.0f} km   T_sc/T_pl = {T1/T2:.6f}")

    # --- Seed via coarse radial-overlap midpoint (§4.2.1) + atan2 projection (§4.2.2) ---
    cos_lo = (a1 - a2*(1 + e2) - r_soi) / (a1 * e1)
    cos_hi = (a1 - a2*(1 - e2) + r_soi) / (a1 * e1)
    E_lo = math.acos(max(-1.0, min(1.0, cos_hi)))  # smaller angle (bigger cos)
    E_hi = math.acos(max(-1.0, min(1.0, cos_lo)))  # larger angle (smaller cos)

    def atan2_seed(E1):
        r1_2d = np.array([a1*(np.cos(E1)-e1), b1*np.sin(E1)])
        qv = B.T @ (A @ r1_2d)
        return math.atan2(qv[1]/b2, qv[0]/a2 + e2)

    # Try both ±branches of the coarse interval, keep the smaller f.
    best = (None, None, np.inf)
    for sign in (+1.0, -1.0):
        E1_0 = sign * 0.5 * (E_lo + E_hi)
        E2_0 = atan2_seed(E1_0)
        f, _, _ = f_grad_hess(E1_0, E2_0, a1, e1, a2, e2, M, r_soi_sq)
        if f < best[2]:
            best = (E1_0, E2_0, f)
    E1s, E2s = newton_minimum(best[0], best[1], a1, e1, a2, e2, M, r_soi_sq, max_iter=20)
    f_min, _, _ = f_grad_hess(E1s, E2s, a1, e1, a2, e2, M, r_soi_sq)
    d_star = math.sqrt(max(f_min + r_soi_sq, 0.0))
    print(f"\nGeometric min:  E1* = {E1s:.4f} rad = {math.degrees(E1s):.2f}°   "
          f"E2* = {E2s:.4f} rad = {math.degrees(E2s):.2f}°")
    print(f"Minimum 3D separation d* = {d_star:,.0f} km   "
          f"({d_star/r_soi:.3f} r_SOI)")

    if d_star > r_soi:
        print(f"\n  (d* > r_SOI — no real SOI entry on this geometry; showing algorithm anyway)")

    # --- Compute t_i*, v_i*, u, w ---
    r1_3d_star, v1_3d, M1s, t1_star = orbital_state_3d(a1, e1, E1s, M01, A, MU_SUN)
    r2_3d_star, v2_3d, M2s, t2_star = orbital_state_3d(a2, e2, E2s, M02, B, MU_SUN)
    u = v2_3d - v1_3d
    w = np.cross(v1_3d, v2_3d)

    print(f"\nt_1* (from epoch): {t1_star:.4e} s = {t1_star/86400:.4f} days")
    print(f"t_2* (from epoch): {t2_star:.4e} s = {t2_star/86400:.4f} days")
    print(f"|v_1*| = {np.linalg.norm(v1_3d):.4f} km/s   |v_2*| = {np.linalg.norm(v2_3d):.4f} km/s")

    # --- Run continued-fraction (k, l) search ---
    # Paper requires T_slow > T_fast. Identify which is which.
    if T1 > T2:
        slow_label, fast_label = "spacecraft", "planet"
        T_slow, T_fast = T1, T2
        t_slow_star, t_fast_star = t1_star, t2_star
        u_used = -u       # paper's u = v_fast − v_slow
        w_used = -w       # paper's w = v_slow × v_fast; swapping sign
    else:
        slow_label, fast_label = "planet", "spacecraft"
        T_slow, T_fast = T2, T1
        t_slow_star, t_fast_star = t2_star, t1_star
        u_used = -u
        w_used = -w
    print(f"\nPaper convention: body_slow = {slow_label} (T={T_slow/86400:.2f} d), "
          f"body_fast = {fast_label} (T={T_fast/86400:.2f} d)")

    t_sim = search_horizon_days * 86400 if search_horizon_days else None
    print(f"Search horizon: {search_horizon_days} days from epoch\n")
    result = find_kl_continued_fraction(
        T_slow, T_fast,
        t_slow_star, t_fast_star,
        u_used, w_used,
        d_star, r_soi,
        k_slow_min=0,
        t_sim_window=t_sim,
        verbose=True,
        trace=True,
    )
    print()
    if result is None:
        print("→ No (k, l) found.")
    else:
        k_slow = result["k_slow"]
        k_fast = result["k_fast"]
        t_coll = result["t_collision"]
        coll_dt = epoch_dt + timedelta(seconds=t_coll)
        # Translate back to our (k_1 = spacecraft, k_2 = planet) convention.
        if slow_label == "spacecraft":
            k1_sc, k2_pl = k_slow, k_fast
        else:
            k1_sc, k2_pl = k_fast, k_slow
        print(f"→ SOLUTION:  (k_1, k_2) = ({k1_sc}, {k2_pl})   "
              f"[spacecraft revs, planet revs from epoch]")
        print(f"   Encounter time: {t_coll:.4e} s = {t_coll/86400:.2f} days = "
              f"{t_coll/86400/365.25:.4f} years from epoch")
        print(f"   Absolute date:  {coll_dt.isoformat(sep=' ', timespec='seconds')}")
        print(f"   Iterations used: {result['iterations']}")
    print()


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------

if __name__ == "__main__":

    # ---- Voyager 2 / Jupiter, launch epoch ----
    v2 = (
        546095282.4529687,                    # a (km)
        0.7331215656362654,                   # e
        math.radians(5.015),                  # i
        math.radians(-32.9834163461),         # Ω
        math.radians(12.2919523634),          # ω
        6.25059,                              # M0 (at launch 1977-08-23)
    )
    # Jupiter at launch (propagate J2000 M = 0.343 back 22.4 years)
    T2_jup = 2*np.pi*math.sqrt((778340816.69271)**3/MU_SUN)
    n2_jup = 2*np.pi / T2_jup
    dt_j2000_to_launch = -(datetime(2000, 1, 1, 12) - datetime(1977, 8, 23, 11, 29, 11)).total_seconds()
    M02_jup_launch = (0.343270671 + n2_jup * dt_j2000_to_launch) % (2*math.pi)
    jupiter = (
        778340816.69271, 0.04838624,
        math.radians(1.30439695), math.radians(100.47390909), math.radians(274.25457074),
        M02_jup_launch,
        48205582.0,  # r_SOI
    )
    run_scenario(
        "Voyager 2 / Jupiter (epoch = launch 1977-08-23)",
        datetime(1977, 8, 23, 11, 29, 11),
        v2, jupiter,
        search_horizon_days=10 * 365,
    )

    # ---- Voyager 2 / Jupiter, J2000 epoch (to show the NEGATIVE-k limitation) ----
    # Same orbital elements, but propagate M01 forward from launch to J2000:
    v2_j2000 = list(v2)
    T1_v2 = 2*np.pi*math.sqrt(v2[0]**3 / MU_SUN)
    n1_v2 = 2*np.pi / T1_v2
    dt_launch_to_j2000 = (datetime(2000, 1, 1, 12) - datetime(1977, 8, 23, 11, 29, 11)).total_seconds()
    v2_j2000[5] = (v2[5] + n1_v2 * dt_launch_to_j2000) % (2*math.pi)
    jupiter_j2000 = list(jupiter)
    jupiter_j2000[5] = 0.343270671
    run_scenario(
        "Voyager 2 / Jupiter (epoch = J2000 — demonstrates negative-k failure)",
        datetime(2000, 1, 1, 12, 0, 0),
        tuple(v2_j2000), tuple(jupiter_j2000),
        search_horizon_days=365,  # only 1 year forward; the real encounter is in the past
    )

    # ---- Parker-like / Venus, launch epoch ----
    rp = 0.046 * AU_KM
    ra = 0.80 * AU_KM
    a_parker = 0.5 * (rp + ra)
    e_parker = (ra - rp) / (ra + rp)
    parker = (
        a_parker, e_parker,
        math.radians(3.39468), math.radians(76.67984),
        math.radians(234.92262),
        0.0,
    )
    venus = (
        108208000.0, 0.00677672,
        math.radians(3.39468), math.radians(76.67984), math.radians(54.92262),
        0.87814,
        616000.0,  # r_SOI Venus
    )
    run_scenario(
        "Parker-like / Venus (epoch = Parker launch 2018-08-12)",
        datetime(2018, 8, 12, 0, 0, 0),
        parker, venus,
        search_horizon_days=50 * 365,  # extend well past the ~25 yr Parker-Venus CF solution
    )
