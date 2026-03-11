#set document(title: "SOI Encounter Derivation — Analytical")
#set page(margin: 1cm, numbering: "1")
#set text(size: 11pt)
#set heading(numbering: "1.")
#set math.equation(numbering: "(1)")

= Analytical Derivation of Sphere-of-Influence Encounter Points

== Problem Statement

Given two Keplerian elliptical orbits (spacecraft orbit 1, planet orbit 2) in heliocentric ecliptic coordinates, find the eccentric anomalies $E_1$, $E_2$ such that the distance between the spacecraft and the planet equals the planet's sphere-of-influence radius $r_"SOI"$:

$ norm(bold(A) bold(r)_1 (E_1) - bold(B) bold(r)_2 (E_2)) = r_"SOI" $

where $bold(A)$ and $bold(B)$ are known constant $3 times 2$ rotation matrices mapping from orbital-plane 2D coordinates to 3D heliocentric ecliptic coordinates.

== Notation

- $a_i$, $b_i$, $e_i$: semi-major axis, semi-minor axis, and eccentricity of orbit $i$ (with $b_i = a_i sqrt(1 - e_i^2)$)
- $E_i$: eccentric anomaly of orbit $i$
- $bold(A) in RR^(3 times 2)$, $bold(B) in RR^(3 times 2)$: orbital-to-ecliptic rotation matrices with components $A_(j k)$, $B_(j k)$
- $r_"SOI"$: radius of the planet's sphere of influence

== Position in Orbital Plane

For a Keplerian ellipse, the 2D position vector in the orbital plane as a function of eccentric anomaly is:

$ bold(r)_i (E_i) = vec(a_i (cos E_i - e_i), b_i sin E_i) $ <orbital_pos>

== Distance Constraint

Square both sides of the constraint:

$ (bold(A) bold(r)_1 - bold(B) bold(r)_2)^top (bold(A) bold(r)_1 - bold(B) bold(r)_2) = r_"SOI"^2 $ <constraint_sq>

Expand the left-hand side:

$ bold(r)_1^top bold(A)^top bold(A) bold(r)_1 - 2 bold(r)_1^top bold(A)^top bold(B) bold(r)_2 + bold(r)_2^top bold(B)^top bold(B) bold(r)_2 = r_"SOI"^2 $ <expanded>

=== Orthonormality of $bold(A)$ and $bold(B)$

Since $bold(A)$ and $bold(B)$ are $3 times 2$ matrices whose columns are orthonormal (they embed an orthonormal frame from the orbital plane into 3D space):

$ bold(A)^top bold(A) = bold(I)_(2 times 2), quad bold(B)^top bold(B) = bold(I)_(2 times 2) $ <ortho>

Therefore the self-terms simplify:

$ bold(r)_1^top bold(A)^top bold(A) bold(r)_1 = bold(r)_1^top bold(r)_1 = |bold(r)_1|^2 =: r_1^2 $
$ bold(r)_2^top bold(B)^top bold(B) bold(r)_2 = bold(r)_2^top bold(r)_2 = |bold(r)_2|^2 =: r_2^2 $

=== The coupling matrix $bold(C)$

Define the $2 times 2$ matrix:

$ bold(C) := 2 bold(A)^top bold(B) $ <C_def>

This encodes the entire mutual orientation of the two orbital planes. @expanded becomes:

$ r_1^2 + r_2^2 - bold(r)_1^top bold(C) bold(r)_2 = r_"SOI"^2 $ <constraint_C>

Note: $bold(C)$ is *not* necessarily symmetric — it is a general $2 times 2$ real matrix with $|C_(j k)| <= 2$.

== Expanding the Constraint

=== Self-terms

Using $r_i = a_i (1 - e_i cos E_i)$:

$ r_1^2 + r_2^2 = a_1^2 (1 - e_1 cos E_1)^2 + a_2^2 (1 - e_2 cos E_2)^2 $ <self_terms>

=== Cross-term

The position vector @orbital_pos factors as $bold(r)_i = op("diag")(a_i, b_i) bold(p)_i$, where:

$ bold(p)_i = vec(cos E_i - e_i, sin E_i) $ <p_def>

Substituting into the cross-term:

$ bold(r)_1^top bold(C) bold(r)_2 = bold(p)_1^top op("diag")(a_1, b_1) bold(C) op("diag")(a_2, b_2) bold(p)_2 = bold(p)_1^top bold(M) bold(p)_2 $

where the *scaled coupling matrix* is:

$ bold(M) := op("diag")(a_1, b_1) bold(C) op("diag")(a_2, b_2) = mat(M_(11), M_(12); M_(21), M_(22)) := mat(C_(11) a_1 a_2, C_(12) a_1 b_2; C_(21) a_2 b_1, C_(22) b_1 b_2) $ <M>

Expanding the bilinear form $bold(p)_1^top bold(M) bold(p)_2$:

$ bold(p)_1^top bold(M) bold(p)_2 = & M_(11) cos E_1 cos E_2 - M_(11) e_2 cos E_1 - M_(11) e_1 cos E_2 + M_(11) e_1 e_2 \
  + & M_(12) cos E_1 sin E_2 - M_(12) e_1 sin E_2 \
  + & M_(21) sin E_1 cos E_2 - M_(21) e_2 sin E_1 \
  + & M_(22) sin E_1 sin E_2 $ <cross_expanded>

=== Full constraint

Substituting @self_terms and @cross_expanded into @constraint_C and collecting by trigonometric structure:

$ & underbrace(a_1^2 + a_2^2 - M_(11) e_1 e_2 - r_"SOI"^2, "constant") \
  + & underbrace((-2 a_1^2 e_1 + M_(11) e_2), alpha_1) cos E_1 + underbrace((-2 a_2^2 e_2 + M_(11) e_1), alpha_2) cos E_2 \
  + & underbrace(M_(21) e_2, beta_1) sin E_1 + underbrace(M_(12) e_1, beta_2) sin E_2 \
  + & a_1^2 e_1^2 cos^2 E_1 + a_2^2 e_2^2 cos^2 E_2 \
  - & M_(11) cos E_1 cos E_2 - M_(12) cos E_1 sin E_2 \
  - & M_(21) sin E_1 cos E_2 - M_(22) sin E_1 sin E_2 \
  = & 0 $ <full_constraint>

== Sum-to-Product Reduction

Define sum and difference anomalies $S := E_1 + E_2$, $D := E_1 - E_2$.

=== Squared cosines

Using $cos^2 E_i = 1/2 (1 + cos 2 E_i)$:

$ a_1^2 e_1^2 cos^2 E_1 + a_2^2 e_2^2 cos^2 E_2 = (a_1^2 e_1^2 + a_2^2 e_2^2) / 2 + (a_1^2 e_1^2) / 2 cos 2 E_1 + (a_2^2 e_2^2) / 2 cos 2 E_2 $

=== Mixed products

Applying the product-to-sum identities $cos A cos B = 1/2 [cos(A - B) + cos(A + B)]$, $sin A sin B = 1/2 [cos(A - B) - cos(A + B)]$, $sin A cos B = 1/2 [sin(A + B) + sin(A - B)]$, $cos A sin B = 1/2 [sin(A + B) - sin(A - B)]$:

$ -M_(11) cos E_1 cos E_2 &= -M_(11) / 2 (cos D + cos S) \
  -M_(22) sin E_1 sin E_2 &= -M_(22) / 2 (cos D - cos S) \
  -M_(21) sin E_1 cos E_2 &= -M_(21) / 2 (sin S + sin D) \
  -M_(12) cos E_1 sin E_2 &= -M_(12) / 2 (sin S - sin D) $

Collecting coefficients of $cos D$, $cos S$, $sin D$, $sin S$:

$ -(M_(11) + M_(22)) / 2 cos D - (M_(11) - M_(22)) / 2 cos S + (M_(12) - M_(21)) / 2 sin D - (M_(12) + M_(21)) / 2 sin S $

=== Collected constraint

Absorbing the constant $1/2 (a_1^2 e_1^2 + a_2^2 e_2^2)$ into the constant term and defining:

$ K &:= a_1^2 + a_2^2 + (a_1^2 e_1^2 + a_2^2 e_2^2) / 2 - M_(11) e_1 e_2 - r_"SOI"^2 $ <K>

the full constraint becomes:

$ & alpha_1 cos E_1 + alpha_2 cos E_2 + beta_1 sin E_1 + beta_2 sin E_2 \
  + & (a_1^2 e_1^2) / 2 cos 2 E_1 + (a_2^2 e_2^2) / 2 cos 2 E_2 \
  - & (M_(11) + M_(22)) / 2 cos D - (M_(11) - M_(22)) / 2 cos S \
  + & (M_(12) - M_(21)) / 2 sin D - (M_(12) + M_(21)) / 2 sin S \
  + & K \
  = & 0 $ <collected>

where $alpha_1 = -2 a_1^2 e_1 + M_(11) e_2$, $alpha_2 = -2 a_2^2 e_2 + M_(11) e_1$, $beta_1 = M_(21) e_2$, $beta_2 = M_(12) e_1$.

== Harmonic Addition

The terms $alpha_1 cos E_1 + beta_1 sin E_1$ in @collected have the same frequency and can be combined via the harmonic addition theorem:

$ alpha_1 cos E_1 + beta_1 sin E_1 = R_1 cos(E_1 - phi_1) $

where:

$ R_1 := sqrt(alpha_1^2 + beta_1^2), quad phi_1 := op("atan2")(beta_1, alpha_1) $ <R1>

Similarly for $E_2$, $D$, and $S$:

$ alpha_2 cos E_2 + beta_2 sin E_2 &= R_2 cos(E_2 - phi_2) \
  -(M_(11) + M_(22)) / 2 cos D + (M_(12) - M_(21)) / 2 sin D &= R_D cos(D - phi_D) \
  -(M_(11) - M_(22)) / 2 cos S - (M_(12) + M_(21)) / 2 sin S &= R_S cos(S - phi_S) $

where:

$ R_2 &:= sqrt(alpha_2^2 + beta_2^2), &quad phi_2 &:= op("atan2")(beta_2, alpha_2) $ <R2>

$ R_D &:= 1/2 sqrt((M_(11) + M_(22))^2 + (M_(12) - M_(21))^2), &quad phi_D &:= op("atan2")(M_(12) - M_(21), -(M_(11) + M_(22))) $ <RD>

$ R_S &:= 1/2 sqrt((M_(11) - M_(22))^2 + (M_(12) + M_(21))^2), &quad phi_S &:= op("atan2")(-(M_(12) + M_(21)), -(M_(11) - M_(22))) $ <RS>

Substituting back into @collected:

$ R_1 cos(E_1 - phi_1) + R_2 cos(E_2 - phi_2) + (a_1^2 e_1^2) / 2 cos 2 E_1 + (a_2^2 e_2^2) / 2 cos 2 E_2 + R_D cos(D - phi_D) + R_S cos(S - phi_S) + K = 0 $ <harmonic>

== Existence Bounds

=== Naive bound

Since each cosine lies in $[-1, 1]$, the function $f(E_1, E_2)$ in @harmonic is bounded in $[K - A, K + A]$ where:

$ A := R_1 + R_2 + (a_1^2 e_1^2) / 2 + (a_2^2 e_2^2) / 2 + R_D + R_S $ <A_naive>

A necessary condition for solutions is $|K| <= A$. Equivalently, if $|K| > A$ then *no encounter is possible*. This treats all six cosines as independent, but they share only two degrees of freedom.

=== Separable decomposition

Split $f$ into terms by their variable dependence:

$ f(E_1, E_2) = g_1 (E_1) + g_2 (E_2) + h(E_1, E_2) + K $ <separable>

where:

$ g_i (E_i) &:= R_i cos(E_i - phi_i) + (a_i^2 e_i^2) / 2 cos 2 E_i $ <gi>

$ h(E_1, E_2) &:= R_D cos(D - phi_D) + R_S cos(S - phi_S) $ <h>

==== Bounding $h$: amplitude depends on $E_1$

Since both terms of $h$ have frequency 1 in $E_2$ (one via $D = E_1 - E_2$, one via $S = E_1 + E_2$), we can collect them. Expanding into $cos E_2$ and $sin E_2$ components:

$ h = P(E_1) cos E_2 + Q(E_1) sin E_2 $

where:

$ P(E_1) &= R_D cos(E_1 - phi_D) + R_S cos(E_1 - phi_S) \
  Q(E_1) &= R_D sin(E_1 - phi_D) - R_S sin(E_1 - phi_S) $

By harmonic addition in $E_2$, for any fixed $E_1$:

$ |h| <= sqrt(P^2 + Q^2) =: rho(E_1) $

Computing $P^2 + Q^2$ and using $cos A cos B - sin A sin B = cos(A + B)$:

$ rho(E_1)^2 = R_D^2 + R_S^2 + 2 R_D R_S cos(2 E_1 - phi_D - phi_S) $ <rho>

This varies between $(R_D - R_S)^2$ and $(R_D + R_S)^2$, so:

$ |R_D - R_S| <= rho(E_1) <= R_D + R_S $

The naive bound uses $rho = R_D + R_S$ everywhere. The tighter bound recognizes that for most values of $E_1$, the mixed term has a smaller amplitude.

==== Bounding $g_i$: coupled harmonics

Each $g_i$ is a sum of first and second harmonics of the same variable. Writing $theta_i = E_i - phi_i$:

$ g_i = R_i cos theta_i + (a_i^2 e_i^2) / 2 cos(2 theta_i + 2 phi_i) $

The extreme values $g_(i, min)$ and $g_(i, max)$ can be found by solving the 1D stationarity condition $g'_i = 0$, which is a cubic in $cos theta_i$ — solvable in closed form or cheaply by 1D numerical sweep. The naive bound $|g_i| <= R_i + a_i^2 e_i^2 \/ 2$ is not tight because the two harmonics cannot simultaneously achieve their extremes.

=== Tighter necessary condition

Combining the exact ranges of $g_i$ with the $E_1$-dependent bound on $h$:

$ g_(1, min) + g_(2, min) - (R_D + R_S) + K <= f <= g_(1, max) + g_(2, max) + (R_D + R_S) + K $

No encounter is possible if the interval does not contain zero:

$ g_(1, max) + g_(2, max) + (R_D + R_S) + K < 0 quad "or" quad g_(1, min) + g_(2, min) - (R_D + R_S) + K > 0 $

For an even tighter check, one can sweep $E_1$ and use the $E_1$-dependent amplitude $rho(E_1)$ from @rho together with the exact $g_1(E_1)$:

$ min_(E_2) f = g_1 (E_1) + g_(2, min) - rho(E_1) + K $
$ max_(E_2) f = g_1 (E_1) + g_(2, max) + rho(E_1) + K $

If $g_1(E_1) + g_(2, max) + rho(E_1) + K < 0$ for all $E_1$, no encounter exists. This reduces the 2D problem to a 1D sweep.
