"""Compute T1, T2, alpha_min, alpha_max for two Keplerian orbits.

T_i             orbital period = 2π √(|a_i|³ / μ)
alpha_star      value of α that puts the time line through the geometric minimum
Δα              admissibility half-width (line tangent to the Taylor ellipse)
alpha_min/max   alpha_star ∓ Δα — the admissibility window on the α axis

Self-contained: inlines the subset of the SOI encounter pipeline needed to
locate geometric minima of f(E1, E2) and build the Taylor-ellipse window.
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from datetime import datetime
from typing import Iterator, Optional

import numpy as np


MU_SUN = 132_712_440_042.0
SOLAR_J2000 = datetime(2000, 1, 1, 12, 0, 0)

KEPLER_TOL = 1e-14
KEPLER_MAX_ITERS = 50
NEWTON_1D_TOL = 1e-12
NEWTON_1D_MAX_ITERS = 30
NEWTON_2D_TOL = 1e-14
NEWTON_2D_MAX_ITERS = 50
E1_GRID_SIZE = 400
DEDUP_TOL = 1e-6


@dataclass
class Orbit:
    a: float        # semi-major axis (km)
    e: float        # eccentricity
    i: float        # inclination (rad)
    Om: float       # RAAN (rad)
    w: float        # argument of periapsis (rad)
    M0: float       # mean anomaly at epoch (rad)

    @property
    def b(self) -> float:
        return self.a * math.sqrt(1 - self.e**2)

    @property
    def T(self) -> float:
        return 2 * math.pi * math.sqrt(self.a**3 / MU_SUN)

    @property
    def rotmat(self) -> np.ndarray:
        """3x2 perifocal-to-inertial rotation (columns = P, Q unit vectors)."""
        cO, sO = math.cos(self.Om), math.sin(self.Om)
        cw, sw = math.cos(self.w), math.sin(self.w)
        ci, si = math.cos(self.i), math.sin(self.i)
        c1 = np.array([cO*cw - sO*sw*ci,  sO*cw + cO*sw*ci, sw*si])
        c2 = np.array([-cO*sw - sO*cw*ci, -sO*sw + cO*cw*ci, cw*si])
        return np.column_stack([c1, c2])


@dataclass
class AlphaWindow:
    E1_star: float
    E2_star: float
    f_star: float
    alpha_star: float
    delta_alpha: float
    alpha_min: float
    alpha_max: float


# ----------------------------------------------------------------------
# Scenario 1: Tesla Roadster (Starman) vs. Earth
# ----------------------------------------------------------------------

# Tesla Roadster: JPL Horizons state at EPOCH = JD 2458239.5 (2018-05-01.0 TDB).
#   EC=0.2557169270061025  A=1.324944859803637 AU  MA=55.56868650049249°
#   OM=317.1102702420760°  W=177.4781316189274°    IN=1.077407526004059°
ORBIT_ROADSTER = Orbit(
    a=1.324944859803637 * 149_597_870.7,
    e=0.2557169270061025,
    i=math.radians(1.077407526004059),
    Om=math.radians(317.1102702420760),
    w=math.radians(177.4781316189274),
    M0=math.radians(55.56868650049249),
)

# Earth: J2000 mean elements, M0 propagated J2000 → JD 2458239.5.
ORBIT_EARTH = Orbit(
    a=1.00000261 * 149_597_870.7,
    e=0.01671123,
    i=math.radians(-0.00001531),
    Om=0.0,
    w=math.radians(102.93768193),
    M0=(math.radians(100.46457166 - 102.93768193)
        + 2*math.pi * (2458239.5 - 2451545.0) * 86400.0
          / (2*math.pi * math.sqrt((1.00000261 * 149_597_870.7)**3 / MU_SUN))
       ) % (2*math.pi),
)
R_SOI_EARTH = ORBIT_EARTH.a * (398600.4418 / MU_SUN) ** 0.4


# ----------------------------------------------------------------------
# Scenario 2: Voyager 2 vs. Jupiter
# ----------------------------------------------------------------------

# Voyager 2 at its post-launch epoch 1977-08-23 14:29 UTC.
ORBIT_V2 = Orbit(
    a=546_095_282.4529687,
    e=0.7331215656362654,
    i=math.radians(5.015),
    Om=math.radians(-32.9834163461),
    w=math.radians(12.2919523634),
    M0=6.25059,
)

# Jupiter, M0 propagated from J2000 back to Voyager 2 epoch 1977-08-23 14:29 UTC.
# dt_s = (J2000 - epoch).total_seconds() = +706_810_260.0 s
ORBIT_JUPITER = Orbit(
    a=778_340_816.69271,
    e=0.04838624,
    i=math.radians(1.30439695),
    Om=math.radians(100.47390909),
    w=math.radians(274.25457074),
    M0=(0.343270671
        - (2*math.pi / (2*math.pi * math.sqrt(778_340_816.69271**3 / MU_SUN))) * 706_810_260.0
       ) % (2*math.pi),
)
R_SOI_JUPITER = ORBIT_JUPITER.a * (1.266_865_319e8 / MU_SUN) ** 0.4


class _EncounterGeometry:
    """f(E1, E2) = ‖r_sc(E1) − r_pl(E2)‖² − r_soi²  and its (grad, Hess)."""

    def __init__(self, sc: Orbit, pl: Orbit, r_soi: float):
        self.sc, self.pl, self.r_soi = sc, pl, r_soi
        self.A, self.B = sc.rotmat, pl.rotmat
        self.a1, self.e1, self.b1 = sc.a, sc.e, sc.b
        self.a2, self.e2, self.b2 = pl.a, pl.e, pl.b
        C = 2.0 * self.A.T @ self.B
        self.M = np.array([
            [C[0, 0]*self.a1*self.a2, C[0, 1]*self.a1*self.b2],
            [C[1, 0]*self.b1*self.a2, C[1, 1]*self.b1*self.b2],
        ])

    def _pts(self, E1: float, E2: float):
        p1 = np.array([math.cos(E1) - self.e1, math.sin(E1)])
        p2 = np.array([math.cos(E2) - self.e2, math.sin(E2)])
        return p1, p2

    def f(self, E1: float, E2: float) -> float:
        p1, p2 = self._pts(E1, E2)
        r1 = self.a1 * (1 - self.e1*math.cos(E1))
        r2 = self.a2 * (1 - self.e2*math.cos(E2))
        return r1**2 + r2**2 - p1 @ self.M @ p2 - self.r_soi**2

    def grad_hess(self, E1: float, E2: float) -> tuple[np.ndarray, np.ndarray]:
        p1, p2 = self._pts(E1, E2)
        wv1 = np.array([-math.sin(E1), math.cos(E1)])
        wv2 = np.array([-math.sin(E2), math.cos(E2)])
        u1 = np.array([math.cos(E1), math.sin(E1)])
        u2 = np.array([math.cos(E2), math.sin(E2)])
        a1, e1, a2, e2, M = self.a1, self.e1, self.a2, self.e2, self.M
        g1 = 2*a1**2*e1*math.sin(E1)*(1 - e1*math.cos(E1)) - wv1 @ M @ p2
        g2 = 2*a2**2*e2*math.sin(E2)*(1 - e2*math.cos(E2)) - p1 @ M @ wv2
        H11 = 2*a1**2*e1*(math.cos(E1) - e1 + 2*e1*math.sin(E1)**2) + u1 @ M @ p2
        H22 = 2*a2**2*e2*(math.cos(E2) - e2 + 2*e2*math.sin(E2)**2) + p1 @ M @ u2
        H12 = -wv1 @ M @ wv2
        return np.array([g1, g2]), np.array([[H11, H12], [H12, H22]])

    def atan2_seed(self, E1: float) -> float:
        r_sc = self.A @ np.array([
            self.a1*(math.cos(E1) - self.e1),
            self.b1*math.sin(E1),
        ])
        u = self.B.T @ r_sc
        return math.atan2(u[1] / self.b2, u[0] / self.a2 + self.e2)


def _newton_1d_on_E2(geom: _EncounterGeometry, E1: float) -> tuple[float, float]:
    E2 = geom.atan2_seed(E1)
    for _ in range(NEWTON_1D_MAX_ITERS):
        g, H = geom.grad_hess(E1, E2)
        if abs(H[1, 1]) < 1e-12:
            break
        d = -g[1] / H[1, 1]
        E2 += d
        if abs(d) < NEWTON_1D_TOL:
            break
    return geom.f(E1, E2), E2


def _newton_2d_refine(
    geom: _EncounterGeometry, E1: float, E2: float,
) -> Optional[tuple[float, float]]:
    for _ in range(NEWTON_2D_MAX_ITERS):
        g, H = geom.grad_hess(E1, E2)
        try:
            d = np.linalg.solve(H, -g)
        except np.linalg.LinAlgError:
            return None
        E1 += d[0]
        E2 += d[1]
        if np.linalg.norm(d) < NEWTON_2D_TOL:
            break
    return (E1, E2) if geom.f(E1, E2) < 0 else None


def _is_duplicate(E1: float, E2: float, existing: list[tuple[float, float]]) -> bool:
    def centered(d: float) -> float:
        return abs(((d + math.pi) % (2*math.pi)) - math.pi)
    return any(
        centered(E1 - a) < DEDUP_TOL and centered(E2 - b) < DEDUP_TOL
        for a, b in existing
    )


def _find_geometric_minima(geom: _EncounterGeometry) -> list[tuple[float, float]]:
    E1_grid = np.linspace(-math.pi, math.pi, E1_GRID_SIZE, endpoint=False)
    inner = [_newton_1d_on_E2(geom, E1) for E1 in E1_grid]
    min_f = np.array([v for v, _ in inner])
    seeds = [
        i for i in range(E1_GRID_SIZE)
        if min_f[i] < 0
        and min_f[i] <= min_f[(i - 1) % E1_GRID_SIZE]
        and min_f[i] <= min_f[(i + 1) % E1_GRID_SIZE]
    ]
    minima: list[tuple[float, float]] = []
    for i in seeds:
        res = _newton_2d_refine(geom, E1_grid[i], inner[i][1])
        if res is None or _is_duplicate(*res, minima):
            continue
        minima.append(res)
    return minima


def _alpha_window_for(
    geom: _EncounterGeometry, E1s: float, E2s: float, sc: Orbit, pl: Orbit,
) -> AlphaWindow:
    f_star = geom.f(E1s, E2s)
    _, H_star = geom.grad_hess(E1s, E2s)

    M1s_wrapped = E1s - sc.e*math.sin(E1s)
    M2s_wrapped = E2s - pl.e*math.sin(E2s)
    M1s = sc.M0 + ((M1s_wrapped - sc.M0) % (2*math.pi))
    M2s = pl.M0 + ((M2s_wrapped - pl.M0) % (2*math.pi))

    dM1dE1 = 1 - sc.e*math.cos(E1s)
    dM2dE2 = 1 - pl.e*math.cos(E2s)
    H_M = H_star / np.array([
        [dM1dE1**2,         dM1dE1*dM2dE2],
        [dM1dE1*dM2dE2,     dM2dE2**2],
    ])
    slope = sc.T / pl.T
    A_q = H_M[0, 0] + 2*slope*H_M[0, 1] + slope**2*H_M[1, 1]
    det_HM = H_M[0, 0]*H_M[1, 1] - H_M[0, 1]**2
    d0_max = math.sqrt(-2 * A_q * f_star / det_HM)
    delta_alpha = d0_max * pl.T / (2*math.pi)
    # α is defined modulo T2 (changing k2 by 1 shifts α by T2); fold into [0, T2).
    alpha_star = ((pl.T*(M2s - pl.M0) - sc.T*(M1s - sc.M0)) / (2*math.pi)) % pl.T

    return AlphaWindow(
        E1_star=E1s,
        E2_star=E2s,
        f_star=f_star,
        alpha_star=alpha_star,
        delta_alpha=delta_alpha,
        alpha_min=alpha_star - delta_alpha,
        alpha_max=alpha_star + delta_alpha,
    )


def _cf_convergents(x: float) -> Iterator[tuple[int, int, int]]:
    """Yield (a_n, p_n, q_n) of the continued-fraction expansion of x."""
    p_prev, p = 0, 1
    q_prev, q = 1, 0
    while True:
        a = int(math.floor(x))
        p, p_prev = a*p + p_prev, p
        q, q_prev = a*q + q_prev, q
        yield a, p, q
        frac = x - a
        if frac < 1e-15:
            return
        x = 1 / frac


def ostrowski_smallest_k1(
    w: AlphaWindow, T1: float, T2: float, k1_min: int = 1,
) -> Optional[tuple[int, int]]:
    """Smallest k1 ≥ k1_min with |ρ(k1)| ≤ Δα, returned as (k1, k2).

    ρ(k1) := T1·k1 − T2·round((T1·k1 − α*)/T2) − α*      (∈ (−T2/2, T2/2])

    Uses Ostrowski expansion: step k1 by CF-convergent denominators q_n to
    nudge ρ onto the admissibility band in O(log(T2/Δα)) levels.
    """
    alpha_star, delta_alpha = w.alpha_star, w.delta_alpha
    tau = T1 / T2

    def rho_at(k: int) -> float:
        k2 = round((T1 * k - alpha_star) / T2)
        return T1 * k - T2 * k2 - alpha_star

    k1 = k1_min
    rho = rho_at(k1)

    cf_iter = _cf_convergents(tau)
    next(cf_iter)  # skip 0-th convergent
    pending = next(cf_iter, None)
    while pending is not None and abs(rho) > delta_alpha:
        _, p_n, q_n = pending
        nxt = next(cf_iter, None)
        a_next = nxt[0] if nxt is not None else 1
        eta_T2 = (q_n * tau - p_n) * T2
        if abs(eta_T2) < 1e-9:
            break
        best_c = max(0, min(a_next, int(round(-rho / eta_T2))))
        if best_c > 0:
            k1 += best_c * q_n
            rho += best_c * eta_T2
        pending = nxt

    if abs(rho) > delta_alpha:
        return None
    k2 = round((T1 * k1 - alpha_star) / T2)
    return k1, k2


def compute_alpha_windows(
    sc: Orbit, pl: Orbit, r_soi: float,
) -> tuple[float, float, list[AlphaWindow]]:
    """Return (T1, T2, [AlphaWindow per geometric minimum of f])."""
    geom = _EncounterGeometry(sc, pl, r_soi)
    windows = [
        _alpha_window_for(geom, E1s, E2s, sc, pl)
        for E1s, E2s in _find_geometric_minima(geom)
    ]
    return sc.T, pl.T, windows


def _report(label: str, sc: Orbit, pl: Orbit, r_soi: float) -> None:
    print("=" * 72)
    print(f"  {label}")
    print("=" * 72)
    # Enforce T1 > T2 ⇒ τ > 1 by swapping the orbits if needed.
    # This relabels (k1, k2): the slower body is now "1", the faster is "2".
    if sc.T < pl.T:
        print(f"(swapping: T_sc={sc.T/86400:.3f} d < T_pl={pl.T/86400:.3f} d → "
              f"slower body becomes body 1)")
        sc, pl = pl, sc
    T1, T2, windows = compute_alpha_windows(sc, pl, r_soi)
    print(f"T1 = {T1:.6e} s  ({T1/86400:.3f} d)")
    print(f"T2 = {T2:.6e} s  ({T2/86400:.3f} d)")
    print(f"τ  = T1/T2 = {T1/T2:.9f}")
    if not windows:
        print("No geometric minima with f* < 0 — no encounter admissibility window.")
        print()
        return
    for idx, w in enumerate(windows):
        print(f"\nMinimum {idx+1}:")
        print(f"  E1*        = {math.degrees(w.E1_star):+.3f}°")
        print(f"  E2*        = {math.degrees(w.E2_star):+.3f}°")
        print(f"  f*         = {w.f_star:.3e}")
        print(f"  α*         = {w.alpha_star:.6e} s")
        print(f"  Δα         = {w.delta_alpha:.6e} s")
        print(f"  α_min      = {w.alpha_min:.6e} s")
        print(f"  α_max      = {w.alpha_max:.6e} s")
        beta_min = w.alpha_min / T2
        beta_max = w.alpha_max / T2
        print(f"  β_min      = {beta_min:.6e}")
        print(f"  β_max      = {beta_max:.6e}")
        delta_beta = beta_max - beta_min
        print(f"  Δβ         = {delta_beta:.6e}")
        phi = (1 + math.sqrt(5)) / 2
        print(f"  log_φ(1/Δβ) = {math.log(1/delta_beta) / math.log(phi):.3f}  "
              f"(CF-search worst-case depth L)")
        pair = ostrowski_smallest_k1(w, T1, T2, k1_min=1)
        if pair is None:
            print("  (k1, k2)   = no admissible lattice point found")
        else:
            k1, k2 = pair
            alpha = T1 * k1 - T2 * k2
            print(f"  (k1, k2)   = ({k1}, {k2})   "
                  f"α = {alpha:.6e} s,  ρ = {alpha - w.alpha_star:+.3e} s")
    print()


if __name__ == "__main__":
    _report("Tesla Roadster (2018-05-01) / Earth", ORBIT_ROADSTER, ORBIT_EARTH, R_SOI_EARTH)
    _report("Voyager 2 (1977-08-23) / Jupiter", ORBIT_V2, ORBIT_JUPITER, R_SOI_JUPITER)
