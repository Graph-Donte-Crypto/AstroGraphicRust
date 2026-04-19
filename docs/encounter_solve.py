"""Find the smallest admissible (k1, k2) revolution pair via continued fractions.

Pipeline:
  1. Find all geometric minima (E1*, E2*) of the separation function f via
     Newton's method seeded from a grid.
  2. For each minimum, compute the Taylor-ellipse admissibility half-width Δα.
  3. Expand T1/T2 as a continued fraction; find the smallest admissible k1 ≥ k1_min
     such that |(T1·k1 - α*) centered-mod-T2| ≤ Δα.
  4. Refine to machine-precision SOI entry via 1D Newton along the time line.
"""

from __future__ import annotations
import math from dataclasses import dataclass, field
from datetime import datetime, timedelta
from typing import Callable, Iterator, Optional

import numpy as np


AU_KM = 149_597_870.7
MU_SUN = 132_712_440_042.0

KEPLER_TOL = 1e-14
KEPLER_MAX_ITERS = 50
NEWTON_1D_TOL = 1e-12
NEWTON_1D_MAX_ITERS = 30
NEWTON_2D_TOL = 1e-14
NEWTON_2D_MAX_ITERS = 50
TIME_NEWTON_TOL = 1e-13
TIME_NEWTON_MAX_ITERS = 50
TIME_NEWTON_H = 1e-7
E1_GRID_SIZE = 400
DEDUP_TOL = 1e-6


# ----------------------------------------------------------------------
# Data classes
# ----------------------------------------------------------------------

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
class Minimum:
    """A geometric minimum of f(E1, E2) with f* < 0."""
    E1s: float
    E2s: float
    f_star: float
    H_M: np.ndarray         # 2x2 Hessian w.r.t. (M1, M2)
    A_q: float              # quadratic coefficient along slope direction
    M1s: float              # mean anomaly at minimum, lifted above M01
    M2s: float
    alpha_star: float
    delta_alpha: float


@dataclass
class Candidate:
    minimum: Minimum
    k1: int
    trace: list[tuple[int, int, int, float]] = field(default_factory=list)


@dataclass
class EncounterResult:
    k1: int
    k2: int
    t_enc: float             # seconds from epoch
    t_enc_dt: datetime
    E1: float
    E2: float
    dist: float
    alpha: float
    minimum: Minimum
    trace: list[tuple[int, int, int, float]]


# ----------------------------------------------------------------------
# Numerics
# ----------------------------------------------------------------------

def kepler(M: float, e: float) -> float:
    """Solve Kepler's equation E - e sin E = M for E."""
    E = M
    for _ in range(KEPLER_MAX_ITERS):
        dE = (E - e*math.sin(E) - M) / (1 - e*math.cos(E))
        E -= dE
        if abs(dE) < KEPLER_TOL:
            break
    return E


def cf_convergents(x: float) -> Iterator[tuple[int, int, int]]:
    """Lazily yield (a_n, p_n, q_n) of the continued-fraction expansion of x."""
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


# ----------------------------------------------------------------------
# Geometry of the encounter function f(E1, E2)
# ----------------------------------------------------------------------

class EncounterGeometry:
    """Encapsulates f(E1, E2) = ‖r_sc(E1) − r_pl(E2)‖² − r_soi².

    Derivatives are taken w.r.t. (E1, E2) in the respective perifocal frames.
    """

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
        a1, e1, a2, e2 = self.a1, self.e1, self.a2, self.e2
        M = self.M
        g1 = 2*a1**2*e1*math.sin(E1)*(1 - e1*math.cos(E1)) - wv1 @ M @ p2
        g2 = 2*a2**2*e2*math.sin(E2)*(1 - e2*math.cos(E2)) - p1 @ M @ wv2
        H11 = 2*a1**2*e1*(math.cos(E1) - e1 + 2*e1*math.sin(E1)**2) + u1 @ M @ p2
        H22 = 2*a2**2*e2*(math.cos(E2) - e2 + 2*e2*math.sin(E2)**2) + p1 @ M @ u2
        H12 = -wv1 @ M @ wv2
        return np.array([g1, g2]), np.array([[H11, H12], [H12, H22]])

    def atan2_seed(self, E1_seed: float) -> float:
        """Project SC position onto planet's perifocal plane → E2 via atan2.

        The ellipse (a2(cos E - e2), b2 sin E) becomes a unit circle under
        (x, y) → ((x + a2·e2)/a2, y/b2), so the closest E is atan2 in stretched
        coords — no true-anomaly conversion needed.
        """
        r_sc = self.A @ np.array([
            self.a1*(math.cos(E1_seed) - self.e1),
            self.b1*math.sin(E1_seed),
        ])
        u = self.B.T @ r_sc
        return math.atan2(u[1] / self.b2, u[0] / self.a2 + self.e2)


# ----------------------------------------------------------------------
# Stage 1: find all geometric minima (E1*, E2*) with f* < 0
# ----------------------------------------------------------------------

def _newton_1d_on_E2(geom: EncounterGeometry, E1: float) -> tuple[float, float]:
    """Minimise f(E1, ·) by 1D Newton seeded from the atan2 projection."""
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
    geom: EncounterGeometry, E1: float, E2: float,
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


def find_geometric_minima(geom: EncounterGeometry) -> list[tuple[float, float]]:
    """Return all (E1*, E2*) critical points with f* < 0."""
    E1_grid = np.linspace(-math.pi, math.pi, E1_GRID_SIZE, endpoint=False)
    inner = [_newton_1d_on_E2(geom, E1) for E1 in E1_grid]
    min_f = np.array([v for v, _ in inner])

    # Local minima (in E1) of the envelope, with f < 0.
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


# ----------------------------------------------------------------------
# Stage 2: Taylor-ellipse admissibility + Ostrowski/CF search for k1
# ----------------------------------------------------------------------

def build_minimum_info(
    geom: EncounterGeometry, E1s: float, E2s: float,
    sc: Orbit, pl: Orbit,
) -> Minimum:
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
    alpha_star = (pl.T*(M2s - pl.M0) - sc.T*(M1s - sc.M0)) / (2*math.pi)

    return Minimum(E1s, E2s, f_star, H_M, A_q, M1s, M2s, alpha_star, delta_alpha)


def ostrowski_search(
    info: Minimum, T1: float, T2: float, k1_min: int,
) -> Optional[Candidate]:
    """Find smallest admissible k1 ≥ k1_min via CF expansion of T1/T2."""
    alpha_star, delta_alpha = info.alpha_star, info.delta_alpha
    tau = T1 / T2

    def rho_at(k: int) -> float:
        alpha = T1 * k - T2 * round((T1 * k - alpha_star) / T2)
        return alpha - alpha_star

    k1 = k1_min
    rho = rho_at(k1)
    trace: list[tuple[int, int, int, float]] = []

    cf_iter = cf_convergents(tau)
    next(cf_iter)  # skip 0-th convergent
    n = 1
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
            trace.append((n, q_n, best_c, rho))
        pending = nxt
        n += 1

    if abs(rho) > delta_alpha:
        return None
    return Candidate(minimum=info, k1=k1, trace=trace)


# ----------------------------------------------------------------------
# Stage 3: refine encounter via line-ellipse seed + 1D time Newton
# ----------------------------------------------------------------------

def refine_encounter(
    geom: EncounterGeometry, cand: Candidate, sc: Orbit, pl: Orbit,
) -> tuple[int, float, float, float, float, float]:
    """Return (k2, alpha, t_enc, E1, E2, dist)."""
    T1, T2 = sc.T, pl.T
    slope = T1 / T2
    info = cand.minimum
    k2 = int(round((T1*cand.k1 - info.alpha_star) / T2))
    alpha = T1*cand.k1 - T2*k2

    c_line = -slope*sc.M0 + pl.M0 + 2*math.pi*alpha/T2
    d0 = slope*info.M1s + c_line - info.M2s
    A_q = info.A_q
    B_q = 2*d0*(info.H_M[0, 1] + slope*info.H_M[1, 1])
    C_q = d0**2 * info.H_M[1, 1] + 2*info.f_star
    disc = B_q**2 - 4*A_q*C_q
    if disc <= 0:
        raise AssertionError("line-ellipse intersection should exist")
    u_low = (-B_q - math.sqrt(disc)) / (2*A_q)
    E1 = kepler(info.M1s + u_low, sc.e)

    h = TIME_NEWTON_H
    for _ in range(TIME_NEWTON_MAX_ITERS):
        M1 = E1 - sc.e*math.sin(E1)
        M2 = slope*M1 + c_line
        E2 = kepler(M2, pl.e)
        fv = geom.f(E1, E2)
        M1p = (E1+h) - sc.e*math.sin(E1+h)
        E2p = kepler(slope*M1p + c_line, pl.e)
        M1m = (E1-h) - sc.e*math.sin(E1-h)
        E2m = kepler(slope*M1m + c_line, pl.e)
        dfdE1 = (geom.f(E1+h, E2p) - geom.f(E1-h, E2m)) / (2*h)
        dE1 = -fv / dfdE1
        E1 += dE1
        if abs(dE1) < TIME_NEWTON_TOL:
            break

    E2 = kepler(slope*(E1 - sc.e*math.sin(E1)) + c_line, pl.e)
    M1_enc = E1 - sc.e*math.sin(E1)
    t_enc = T1/(2*math.pi) * (M1_enc + 2*math.pi*cand.k1 - sc.M0)

    r_sc = geom.A @ np.array([sc.a*(math.cos(E1) - sc.e), sc.b*math.sin(E1)])
    r_pl = geom.B @ np.array([pl.a*(math.cos(E2) - pl.e), pl.b*math.sin(E2)])
    dist = float(np.linalg.norm(r_sc - r_pl))

    return k2, alpha, t_enc, E1, E2, dist


# ----------------------------------------------------------------------
# Orchestrator
# ----------------------------------------------------------------------

def solve(
    sc: Orbit, pl: Orbit, r_soi: float, epoch: datetime,
    t_start: float = 0.0,
) -> Optional[EncounterResult]:
    """Pure computation — no I/O. Returns None if no encounter."""
    geom = EncounterGeometry(sc, pl, r_soi)
    minima_pts = find_geometric_minima(geom)
    if not minima_pts:
        return None

    T1, T2 = sc.T, pl.T
    candidates: list[Candidate] = []
    for E1s, E2s in minima_pts:
        info = build_minimum_info(geom, E1s, E2s, sc, pl)
        k1_min = int(math.ceil(t_start/T1 - (info.M1s - sc.M0)/(2*math.pi)))
        cand = ostrowski_search(info, T1, T2, k1_min)
        if cand is not None:
            candidates.append(cand)

    if not candidates:
        return None

    candidates.sort(key=lambda c: c.k1)
    cand = candidates[0]
    k2, alpha, t_enc, E1, E2, dist = refine_encounter(geom, cand, sc, pl)
    return EncounterResult(
        k1=cand.k1, k2=k2, t_enc=t_enc,
        t_enc_dt=epoch + timedelta(seconds=t_enc),
        E1=E1, E2=E2, dist=dist, alpha=alpha,
        minimum=cand.minimum, trace=cand.trace,
    )


# ----------------------------------------------------------------------
# Reporting (all I/O lives here)
# ----------------------------------------------------------------------

def _print_header(label: str, epoch: datetime, T1: float, T2: float) -> None:
    bar = "=" * 74
    print(bar)
    print(f"  {label}")
    print(bar)
    print(f"Epoch: {epoch.isoformat(sep=' ', timespec='seconds')} (t = 0)")
    print(f"T1 = {T1/86400:.3f} d   T2 = {T2/86400:.3f} d   T1/T2 = {T1/T2:.6f}")


def _print_minimum(idx: int, total: int, info: Minimum, k1_min: int) -> None:
    print(
        f"\n--- Minimum {idx+1}/{total}: "
        f"E1*={math.degrees(info.E1s):+.2f}°, E2*={math.degrees(info.E2s):+.2f}°, "
        f"f*={info.f_star:.3e}, Δα={info.delta_alpha:.3e} s, "
        f"α*={info.alpha_star:.3e} s, k1_min={k1_min}"
    )


def _print_trace(trace: list[tuple[int, int, int, float]]) -> None:
    if not trace:
        return
    print("\nOstrowski expansion of Δk (winning minimum):")
    for n, q_n, c_n, r in trace:
        print(f"    c_{n} = {c_n},  q_{n} = {q_n}   →   k1 += {c_n*q_n},  ρ = {r:+.3e} s")


def _print_result(
    res: EncounterResult, T1: float, r_soi: float,
    ref_dt: Optional[datetime],
) -> None:
    info = res.minimum
    print("\n=== Result ===")
    print(f"(k1, k2)  = ({res.k1}, {res.k2})")
    print(f"α - α*    = {res.alpha - info.alpha_star:+.3e} s   "
          f"(Δα = {info.delta_alpha:.3e} s)")
    print(f"t_enc     = {res.t_enc_dt.isoformat(sep=' ', timespec='seconds')}")
    print(f"            = {res.t_enc/86400:.3f} d from epoch "
          f"= {res.t_enc/T1:.3f} × T1 = {res.t_enc/86400/365.25:.4f} yr")
    print(f"‖Δr‖      = {res.dist:,.3f} km   r_SOI = {r_soi:,.3f} km   "
          f"diff = {res.dist-r_soi:+.3f} km")
    if ref_dt is not None:
        delta = (res.t_enc_dt - ref_dt).total_seconds()
        print(f"ref       = {ref_dt.isoformat(sep=' ', timespec='seconds')}   "
              f"t_enc - ref = {delta/86400:+.3f} d")
    print()


def solve_and_report(
    label: str, sc: Orbit, pl: Orbit, r_soi: float, epoch: datetime,
    t_start: float = 0.0, ref_dt: Optional[datetime] = None,
) -> Optional[EncounterResult]:
    """Verbose wrapper that reproduces the original script's output."""
    _print_header(label, epoch, sc.T, pl.T)

    # Run the pipeline inline so we can narrate intermediate steps.
    geom = EncounterGeometry(sc, pl, r_soi)
    minima_pts = find_geometric_minima(geom)
    if not minima_pts:
        print("  No geometric minima with f < 0 — orbits don't cross SOI.")
        if ref_dt is not None:
            print(f"  ref = {ref_dt.isoformat(sep=' ', timespec='seconds')}\n")
        return None

    print(f"Found {len(minima_pts)} geometric minima with f* < 0.")
    T1, T2 = sc.T, pl.T
    candidates: list[Candidate] = []
    for idx, (E1s, E2s) in enumerate(minima_pts):
        info = build_minimum_info(geom, E1s, E2s, sc, pl)
        k1_min = int(math.ceil(t_start/T1 - (info.M1s - sc.M0)/(2*math.pi)))
        _print_minimum(idx, len(minima_pts), info, k1_min)
        cand = ostrowski_search(info, T1, T2, k1_min)
        if cand is None:
            print(f"    Ostrowski failed: |ρ| > Δα={info.delta_alpha:.3e}")
            continue
        print(f"    k1={cand.k1}, admissible")
        candidates.append(cand)

    if not candidates:
        print("\n  No minimum yielded an admissible k1.")
        if ref_dt is not None:
            print(f"  ref = {ref_dt.isoformat(sep=' ', timespec='seconds')}\n")
        return None

    candidates.sort(key=lambda c: c.k1)
    cand = candidates[0]
    k2, alpha, t_enc, E1, E2, dist = refine_encounter(geom, cand, sc, pl)
    res = EncounterResult(
        k1=cand.k1, k2=k2, t_enc=t_enc,
        t_enc_dt=epoch + timedelta(seconds=t_enc),
        E1=E1, E2=E2, dist=dist, alpha=alpha,
        minimum=cand.minimum, trace=cand.trace,
    )
    _print_trace(res.trace)
    _print_result(res, T1, r_soi, ref_dt)
    return res


# ----------------------------------------------------------------------
# Solar-system helpers (for BepiColombo scenario)
# ----------------------------------------------------------------------

SOLAR_J2000 = datetime(2000, 1, 1, 12, 0, 0)


def planet_orbit(solar_cfg: dict, name: str, at_epoch: datetime) -> tuple[Orbit, float]:
    p = next(pl for pl in solar_cfg["planets"] if pl["name"] == name)
    o = p["orbit"]
    T = 2*math.pi * math.sqrt(o["a"]**3 / MU_SUN)
    dt = (at_epoch - SOLAR_J2000).total_seconds()
    M0 = (o["M0"] + (2*math.pi / T) * dt) % (2*math.pi)
    orbit = Orbit(
        a=o["a"], e=o["e"],
        i=math.radians(o["i"]), Om=math.radians(o["Ω"]),
        w=math.radians(o["ω"]), M0=M0,
    )
    return orbit, p["μ"]


def state_at(orbit: Orbit, dt_seconds: float) -> tuple[np.ndarray, np.ndarray]:
    """Heliocentric (r, v) at time dt_seconds past orbit.M0."""
    n = 2*math.pi / orbit.T
    M = orbit.M0 + n * dt_seconds
    E = kepler(M, orbit.e)
    cE, sE = math.cos(E), math.sin(E)
    r_pf = np.array([orbit.a*(cE - orbit.e), orbit.b*sE])
    v_factor = math.sqrt(MU_SUN * orbit.a) / (orbit.a*(1 - orbit.e*cE))
    v_pf = v_factor * np.array([-sE, math.sqrt(1 - orbit.e**2)*cE])
    R = orbit.rotmat
    return R @ r_pf, R @ v_pf


def orbit_from_rv(r: np.ndarray, v: np.ndarray, mu: float) -> Orbit:
    """Classical Keplerian elements from heliocentric state (r, v)."""
    r_norm = float(np.linalg.norm(r))
    v_norm = float(np.linalg.norm(v))
    h = np.cross(r, v)
    h_norm = float(np.linalg.norm(h))
    k_hat = np.array([0.0, 0.0, 1.0])
    n = np.cross(k_hat, h)
    n_norm = float(np.linalg.norm(n))
    e_vec = ((v_norm**2 - mu/r_norm)*r - np.dot(r, v)*v) / mu
    e = float(np.linalg.norm(e_vec))
    a = 1.0 / (2.0/r_norm - v_norm**2/mu)
    i = math.acos(h[2]/h_norm)
    Om = math.atan2(n[1], n[0]) % (2*math.pi) if n_norm > 1e-12 else 0.0
    if n_norm > 1e-12 and e > 1e-12:
        w = math.atan2(
            float(np.dot(np.cross(n, e_vec), h))/h_norm,
            float(np.dot(n, e_vec)),
        ) % (2*math.pi)
    else:
        w = 0.0
    nu = math.atan2(
        float(np.dot(np.cross(e_vec, r), h))/h_norm,
        float(np.dot(e_vec, r)),
    )
    E = 2*math.atan2(math.sqrt(1-e)*math.sin(nu/2), math.sqrt(1+e)*math.cos(nu/2))
    M = E - e*math.sin(E)
    return Orbit(a=a, e=e, i=i, Om=Om, w=w, M0=M % (2*math.pi))


# ----------------------------------------------------------------------
# Scenarios
# ----------------------------------------------------------------------

def _scenario_parker_venus() -> None:
    """Synthetic Parker-like SC × Venus — narrow Taylor ellipse."""
    epoch = datetime(2018, 8, 12, 7, 31, 0)
    parker = Orbit(
        a=0.5 * (0.046 * AU_KM + 0.80 * AU_KM),
        e=(0.80 - 0.046) / (0.80 + 0.046),
        i=math.radians(3.39468),
        Om=math.radians(76.67984),
        w=math.radians(234.92262),
        M0=0.0,
    )
    venus = Orbit(
        a=108_208_000.0, e=0.006_776_72,
        i=math.radians(3.39468), Om=math.radians(76.67984),
        w=math.radians(54.92262), M0=0.87814,
    )
    r_soi = venus.a * (324_858.592 / MU_SUN) ** 0.4
    solve_and_report(
        "Parker-like / Venus (narrow Taylor ellipse)",
        parker, venus, r_soi, epoch=epoch,
    )


def _scenario_voyager2_jupiter() -> None:
    """Voyager 2 × Jupiter — wide Taylor ellipse, k1 = 0."""
    epoch = datetime(1977, 8, 23, 14, 29, 0)
    v2 = Orbit(
        a=546_095_282.4529687, e=0.7331215656362654,
        i=math.radians(5.015),
        Om=math.radians(-32.9834163461),
        w=math.radians(12.2919523634),
        M0=6.25059,
    )
    a_jup = 778_340_816.69271
    T_jup = 2*math.pi * math.sqrt(a_jup**3 / MU_SUN)
    dt = (SOLAR_J2000 - epoch).total_seconds()
    M0_jup = (0.343270671 - (2*math.pi / T_jup) * dt) % (2*math.pi)
    jupiter = Orbit(
        a=a_jup, e=0.04838624,
        i=math.radians(1.30439695), Om=math.radians(100.47390909),
        w=math.radians(274.25457074), M0=M0_jup,
    )
    r_soi = jupiter.a * (1.266_865_319e8 / MU_SUN) ** 0.4
    solve_and_report(
        "Voyager 2 / Jupiter (wide Taylor ellipse, k1 = 0)",
        v2, jupiter, r_soi, epoch=epoch,
    )


_BEPI_FLYBYS = [
    ("Venus flyby 2",     datetime(2021,  8, 10, 13, 51,  0), "Venus"),
    ("1st Mercury flyby", datetime(2021, 10,  1, 23, 34, 41), "Mercury"),
    ("2nd Mercury flyby", datetime(2022,  6, 23,  9, 44,  0), "Mercury"),
    ("3rd Mercury flyby", datetime(2023,  6, 19, 19, 34,  0), "Mercury"),
    ("4th Mercury flyby", datetime(2024,  9,  4, 21, 48,  0), "Mercury"),
]
# Approximate Bepi-orbit revolution count per arc; Lambert M = round(N).
_BEPI_ARC_REVS = [0.33, 2, 3, 4]


def _scenario_bepi_mercury() -> None:
    import yaml
    from lamberthub import izzo2015

    with open("../config/system/solar.yml") as f:
        solar = yaml.safe_load(f)

    for idx in range(len(_BEPI_FLYBYS) - 1):
        name1, dt1, body1 = _BEPI_FLYBYS[idx]
        name2, dt2, body2 = _BEPI_FLYBYS[idx + 1]
        mercury, mu_merc = planet_orbit(solar, "Mercury", dt1)
        p1, _ = planet_orbit(solar, body1, dt1)
        p2, _ = planet_orbit(solar, body2, dt1)
        r1, _ = state_at(p1, 0.0)
        tof = (dt2 - dt1).total_seconds()
        r2, _ = state_at(p2, tof)

        M_rev = int(round(_BEPI_ARC_REVS[idx]))
        v1, _ = izzo2015(MU_SUN, r1, r2, tof, M=M_rev, prograde=True, low_path=True)
        bepi = orbit_from_rv(r1, v1, MU_SUN)
        r_soi = mercury.a * (mu_merc / MU_SUN) ** 0.4

        try:
            solve_and_report(
                f"BepiColombo arc {idx+1}: {name1} → {name2}",
                bepi, mercury, r_soi, epoch=dt1,
                t_start=tof - 86400.0, ref_dt=dt2,
            )
        except (np.linalg.LinAlgError, AssertionError, ValueError) as ex:
            print(f"  [arc {idx+1} solver failed: {type(ex).__name__}: {ex}]\n")


if __name__ == "__main__":
    _scenario_parker_venus()
    _scenario_voyager2_jupiter()
    _scenario_bepi_mercury()
