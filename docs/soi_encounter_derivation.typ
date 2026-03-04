#set document(title: "SOI Encounter Derivation")
#set page(margin: 2cm, numbering: "1")
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

$ bold(C) := bold(A)^top bold(B) $ <C_def>

This encodes the entire mutual orientation of the two orbital planes. @expanded becomes:

$ r_1^2 + r_2^2 - 2 bold(r)_1^top bold(C) bold(r)_2 = r_"SOI"^2 $ <constraint_C>

with components:

$ bold(C) = mat(C_11, C_12; C_21, C_22) = mat(bold(a)_1 dot bold(b)_1, bold(a)_1 dot bold(b)_2; bold(a)_2 dot bold(b)_1, bold(a)_2 dot bold(b)_2) $

where $bold(a)_k$, $bold(b)_k$ denote column $k$ of $bold(A)$, $bold(B)$ respectively.

Note: $bold(C)$ is *not* necessarily symmetric — it is a general $2 times 2$ real matrix with $|C_(j k)| <= 1$.

== Expanding the Scalar Terms

=== Heliocentric distances $r_i^2$

$ r_i^2 = a_i^2 (1 - e_i cos E_i)^2 $

=== Cross-term $bold(r)_1^top bold(C) bold(r)_2$

Write out the matrix product:

$ bold(r)_1^top bold(C) bold(r)_2 = vec(x_1, y_1)^top mat(C_11, C_12; C_21, C_22) vec(x_2, y_2) $

where $x_i := a_i (cos E_i - e_i)$ and $y_i := b_i sin E_i$.

$ bold(r)_1^top bold(C) bold(r)_2 = C_11 x_1 x_2 + C_12 x_1 y_2 + C_21 y_1 x_2 + C_22 y_1 y_2 $ <cross_term>

Substitute the explicit forms:

$ bold(r)_1^top bold(C) bold(r)_2 = &C_11 a_1 (cos E_1 - e_1) dot a_2 (cos E_2 - e_2) \
  + &C_12 a_1 (cos E_1 - e_1) dot b_2 sin E_2 \
  + &C_21 b_1 sin E_1 dot a_2 (cos E_2 - e_2) \
  + &C_22 b_1 sin E_1 dot b_2 sin E_2 $ <cross_substituted>

Expand each product:

$ (cos E_1 - e_1)(cos E_2 - e_2) &= cos E_1 cos E_2 - e_2 cos E_1 - e_1 cos E_2 + e_1 e_2 $
$ (cos E_1 - e_1) sin E_2 &= cos E_1 sin E_2 - e_1 sin E_2 $
$ sin E_1 (cos E_2 - e_2) &= sin E_1 cos E_2 - e_2 sin E_1 $

Collecting by type of dependence on $E_1$, $E_2$:

$ bold(r)_1^top bold(C) bold(r)_2 = underbrace(S(E_1, E_2), "bilinear") - underbrace(e_1 Q(E_2), E_2 "only") - underbrace(e_2 P(E_1), E_1 "only") + underbrace(a_1 a_2 e_1 e_2 C_11, "constant") $ <cross_grouped>

where:

$ S(E_1, E_2) &:= C_11 a_1 a_2 cos E_1 cos E_2 + C_12 a_1 b_2 cos E_1 sin E_2 + C_21 b_1 a_2 sin E_1 cos E_2 + C_22 b_1 b_2 sin E_1 sin E_2 $ <S_def>

$ P(E_1) &:= C_11 a_1 a_2 cos E_1 + C_21 b_1 a_2 sin E_1 $ <P_def>

$ Q(E_2) &:= C_11 a_1 a_2 cos E_2 + C_12 a_1 b_2 sin E_2 $ <Q_def>

=== Full constraint

Substituting into @constraint_C:

$ a_1^2 (1 - e_1 cos E_1)^2 + a_2^2 (1 - e_2 cos E_2)^2 - 2 [S - e_1 Q - e_2 P + a_1 a_2 e_1 e_2 C_11] = r_"SOI"^2 $ <full_constraint>

Expand the squared terms and rearrange:

$ &underbrace((a_1^2 + a_2^2 - 2 a_1 a_2 e_1 e_2 C_11 - r_"SOI"^2), =: K) \
+ &underbrace((-2 a_1^2 e_1 cos E_1 + a_1^2 e_1^2 cos^2 E_1 + 2 e_2 P(E_1)), =: f_1(E_1)) \
+ &underbrace((-2 a_2^2 e_2 cos E_2 + a_2^2 e_2^2 cos^2 E_2 + 2 e_1 Q(E_2)), =: f_2(E_2)) \
- &2 S(E_1, E_2) = 0 $ <full_explicit>

=== Constant $K$

$ K := a_1^2 + a_2^2 - 2 a_1 a_2 e_1 e_2 C_11 - r_"SOI"^2 $ <K_def>

=== Terms depending only on $E_1$

Substituting @P_def:

$ f_1(E_1) &= -2 a_1^2 e_1 cos E_1 + a_1^2 e_1^2 cos^2 E_1 + 2 e_2 (C_11 a_1 a_2 cos E_1 + C_21 b_1 a_2 sin E_1) \
  &= -2 a_1 e_1 (a_1 - a_2 e_2 C_11) cos E_1 + 2 a_2 b_1 e_2 C_21 sin E_1 + a_1^2 e_1^2 cos^2 E_1 $ <f1>

=== Terms depending only on $E_2$

Substituting @Q_def:

$ f_2(E_2) = -2 a_2 e_2 (a_2 - a_1 e_1 C_11) cos E_2 + 2 a_1 b_2 e_1 C_12 sin E_2 + a_2^2 e_2^2 cos^2 E_2 $ <f2>

=== Compact form

$ K + f_1(E_1) + f_2(E_2) - 2 S(E_1, E_2) = 0 $ <compact>

== Half-Angle (Weierstrass) Substitution

To convert the transcendental equation into a polynomial, substitute:

$ t_i := tan(E_i / 2) $ <half_angle>

$ cos E_i = (1 - t_i^2) / (1 + t_i^2), quad sin E_i = (2 t_i) / (1 + t_i^2) $ <weierstrass>

This is valid for $E_i != pi + 2 pi k$ (i.e. $t_i != infinity$), which corresponds to apoapsis. That case can be checked separately.

=== Rationalize $f_1$

Let $T_i := 1 + t_i^2$. From @f1, substitute @weierstrass:

$ f_1 = -lambda_1 (1 - t_1^2) / T_1 + sigma_1 t_1 / T_1 + q_1 (1 - t_1^2)^2 / T_1^2 $

where we defined shorthand constants:
$ lambda_1 := 2 a_1 e_1 (a_1 - a_2 e_2 C_11), quad sigma_1 := 4 a_2 b_1 e_2 C_21, quad q_1 := a_1^2 e_1^2 $ <f1_constants>

Multiply by $T_1^2$:

$ f_1 T_1^2 &= -lambda_1 (1 - t_1^2)(1 + t_1^2) + sigma_1 t_1 (1 + t_1^2) + q_1 (1 - t_1^2)^2 \
  &= -lambda_1 (1 - t_1^4) + sigma_1 (t_1 + t_1^3) + q_1 (1 - 2 t_1^2 + t_1^4) $

Collecting by powers of $t_1$:

$ f_1 T_1^2 = (q_1 - lambda_1) + sigma_1 t_1 - 2 q_1 t_1^2 + sigma_1 t_1^3 + (q_1 + lambda_1) t_1^4 $ <f1_poly>

=== Rationalize $f_2$

By analogy, define:
$ lambda_2 := 2 a_2 e_2 (a_2 - a_1 e_1 C_11), quad sigma_2 := 4 a_1 b_2 e_1 C_12, quad q_2 := a_2^2 e_2^2 $ <f2_constants>

$ f_2 T_2^2 = (q_2 - lambda_2) + sigma_2 t_2 - 2 q_2 t_2^2 + sigma_2 t_2^3 + (q_2 + lambda_2) t_2^4 $ <f2_poly>

=== Rationalize $S$

From @S_def, substituting @weierstrass for both variables:

$ S = &C_11 a_1 a_2 (1 - t_1^2)(1 - t_2^2) / (T_1 T_2) + C_12 a_1 b_2 (1 - t_1^2)(2 t_2) / (T_1 T_2) \
    + &C_21 b_1 a_2 (2 t_1)(1 - t_2^2) / (T_1 T_2) + C_22 b_1 b_2 (2 t_1)(2 t_2) / (T_1 T_2) $

$ S dot T_1 T_2 = &C_11 a_1 a_2 (1 - t_1^2 - t_2^2 + t_1^2 t_2^2) \
  + &2 C_12 a_1 b_2 (t_2 - t_1^2 t_2) \
  + &2 C_21 b_1 a_2 (t_1 - t_1 t_2^2) \
  + &4 C_22 b_1 b_2 t_1 t_2 $ <S_expanded>

=== Full polynomial constraint (distance)

Multiply @compact by $T_1^2 T_2^2$:

$ K T_1^2 T_2^2 + (f_1 T_1^2) T_2^2 + T_1^2 (f_2 T_2^2) - 2 (S dot T_1 T_2) T_1 T_2 = 0 $ <poly_dist>

Each factor in parentheses has been computed in @f1_poly, @f2_poly, @S_expanded. This is a bivariate polynomial of bidegree $(4, 4)$ in $(t_1, t_2)$:

$ F(t_1, t_2) = 0 $ <F_eq>

== Structure of the Quartic in $t_1$

For a fixed $t_2$, @poly_dist is a quartic in $t_1$:

$ F(t_1; t_2) = c_4 t_1^4 + c_3 t_1^3 + c_2 t_1^2 + c_1 t_1 + c_0 = 0 $ <quartic_form>

where each $c_j$ is a polynomial of degree $<= 4$ in $t_2$.

=== Explicit coefficients

To collect the coefficients, first extract the $t_1$-structure of each contribution to @poly_dist.

*$S$ contribution.* From @S_expanded, collect by powers of $t_1$:

$ S dot T_1 T_2 = s_0 + s_1 t_1 - s_0 t_1^2 $

where:
$ s_0 &:= C_11 a_1 a_2 (1 - t_2^2) + 2 C_12 a_1 b_2 t_2 \
  s_1 &:= 2 C_21 b_1 a_2 (1 - t_2^2) + 4 C_22 b_1 b_2 t_2 $ <s_defs>

Multiply by $(1 + t_1^2)$ to form $-2 (S dot T_1 T_2) T_1 T_2$:

$ (s_0 + s_1 t_1 - s_0 t_1^2)(1 + t_1^2) = s_0 + s_1 t_1 + s_1 t_1^3 - s_0 t_1^4 $

*Remaining terms.* $K T_1^2 T_2^2$ and $T_1^2 (f_2 T_2^2)$ are even in $t_1$ via the factor $T_1^2 = 1 + 2 t_1^2 + t_1^4$, contributing at powers $t_1^0, t_1^2, t_1^4$ with ratio $1 : 2 : 1$. Define:

$ Gamma := f_2 T_2^2 = (q_2 - lambda_2) + sigma_2 t_2 - 2 q_2 t_2^2 + sigma_2 t_2^3 + (q_2 + lambda_2) t_2^4 $ <Gamma_def>

Summing all four contributions by powers of $t_1$:

$ c_0 &= (K T_2^2 + q_1 - lambda_1) T_2^2 + Gamma - 2 s_0 T_2 $ <c0>

$ c_1 &= sigma_1 T_2^2 - 2 s_1 T_2 $ <c1>

$ c_2 &= (2 K T_2^2 - 2 q_1) T_2^2 + 2 Gamma $ <c2>

$ c_4 &= (K T_2^2 + q_1 + lambda_1) T_2^2 + Gamma + 2 s_0 T_2 $ <c4>

One can verify that $c_0 + c_4 - c_2 = 4 q_1 T_2^2$, confirming @division.

=== The $c_1 = c_3$ identity

The quartic *always* satisfies $c_1 = c_3$, regardless of orbital parameters. This follows from the structure of each contribution to @poly_dist:

- $K T_1^2 T_2^2$ and $T_1^2 (f_2 T_2^2)$: contain only *even* powers of $t_1$ (since $T_1^2 = (1 + t_1^2)^2$ and $f_2$ is independent of $t_1$). They contribute nothing to $c_1$ or $c_3$.

- $f_1 T_1^2$: from @f1_poly, the coefficients of $t_1$ and $t_1^3$ are both $sigma_1$, so $c_1 = c_3$ for this part.

- $S dot T_1 T_2$: is at most degree 2 in $t_1$ (from @S_expanded). When multiplied by $T_1 T_2 = (1 + t_1^2)(1 + t_2^2)$, the factor $(1 + t_1^2)$ guarantees equal $t_1^1$ and $t_1^3$ coefficients: if $S dot T_1 T_2 = s_0 + s_1 t_1 + s_2 t_1^2$, then $(s_0 + s_1 t_1 + s_2 t_1^2)(1 + t_1^2) = s_0 + s_1 t_1 + (s_0 + s_2) t_1^2 + s_1 t_1^3 + s_2 t_1^4$.

Therefore the quartic has the form:

$ c_4 t_1^4 + c_1 t_1^3 + c_2 t_1^2 + c_1 t_1 + c_0 = 0 $ <quartic_c1c3>

=== Decomposition

This can be split into a biquadratic part (even powers only) and a residual:

$ F(t_1; t_2) = underbrace(c_4 t_1^4 + c_2 t_1^2 + c_0, =: B(t_1^2)) + c_1 t_1 (t_1^2 + 1) $ <decomposition>

The biquadratic $B(w) = c_4 w^2 + c_2 w + c_0$ is a quadratic in $w = t_1^2$.

Polynomial division by $(t_1^2 + 1)$ gives:

$ F = (t_1^2 + 1)(c_4 t_1^2 + c_1 t_1 + c_2 - c_4) + underbrace((c_0 + c_4 - c_2), = 4 q_1 T_2^2) $ <division>

where $q_1 = a_1^2 e_1^2$ from @f1_constants.

=== Special cases

The remainder in @division vanishes when $q_1 = 0$, i.e. when $e_1 = 0$ (circular spacecraft orbit). In that case $(t_1^2 + 1)$ is a factor; its roots $t_1 = plus.minus i$ are complex, so the quartic reduces to the *quadratic* $c_4 t_1^2 + c_1 t_1 + c_2 - c_4 = 0$, giving at most 2 real geometric solutions per $E_2$.

When the orbital planes are coplanar with identical orientation ($bold(C) = bold(I)$), we have $C_(21) = 0$ and $C_(12) = 0$, which makes $sigma_1 = 0$ (from @f1_constants). Since $c_1 = sigma_1 T_2^2 + dots$ terms proportional to $C_(21)$ and $C_(22)$ cross-products, coplanarity drives $c_1 -> 0$. With $c_1 = 0$, @decomposition reduces to the pure biquadratic $B(t_1^2) = 0$, solvable via a single quadratic formula in $t_1^2$.

#table(
  columns: (1fr, 1fr, 1fr),
  align: center,
  table.header([*Condition*], [*Simplification*], [*Effective degree*]),
  [General], [$c_1 = c_3$ quartic], [4],
  [Circular s/c ($e_1 = 0$)], [$(t_1^2+1)$ factors out $arrow$ quadratic], [2],
  [Coplanar ($bold(C) = bold(I)$)], [$c_1 = 0$ $arrow$ biquadratic], [2 (in $t_1^2$)],
  [Both], [Trivial quadratic], [2],
)

In the general (non-degenerate) case, $c_1 = c_3$ does not reduce the quartic to lower degree, and Ferrari's method (or equivalent) is required.

=== Ferrari's resolvent cubic

To solve the quartic @quartic_c1c3 via Ferrari's method, first convert to depressed form by substituting $t_1 = u - c_1 \/ (4 c_4)$:

$ u^4 + alpha u^2 + beta u + gamma = 0 $ <depressed>

where:
$ alpha &:= c_2 / c_4 - 3 c_1^2 / (8 c_4^2) \
  beta &:= c_1 / c_4 (1 - c_2 / (2 c_4) + c_1^2 / (8 c_4^2)) \
  gamma &:= c_0 / c_4 - c_1^2 / (4 c_4^2) + c_1^2 c_2 / (16 c_4^3) - 3 c_1^4 / (256 c_4^4) $ <depressed_coeffs>

The $c_1 = c_3$ identity manifests as the factored form of $beta$: the linear coefficient of the depressed quartic is proportional to $c_1$. When $c_1 = 0$ (coplanar case), $beta = 0$ and the depressed quartic is a biquadratic — consistent with the special-case analysis above.

Ferrari's method rewrites @depressed as:

$ (u^2 + y / 2)^2 = (y - alpha) u^2 - beta u + (y^2 / 4 - gamma) $ <ferrari_square>

The right-hand side is a perfect square in $u$ when its discriminant vanishes, giving the resolvent cubic:

$ y^3 - alpha y^2 - 4 gamma y + (4 alpha gamma - beta^2) = 0 $ <resolvent>

The constant term $4 alpha gamma - beta^2$ vanishes when $beta = 0$ (i.e. $c_1 = 0$), making $y = 0$ a root of the resolvent — from which Ferrari immediately yields the biquadratic factorization, consistent with the coplanar special case.

Once a real root $y_0$ of @resolvent is found (analytically via Cardano's formula or numerically), define:

$ m := sqrt(y_0 - alpha), quad n := beta / (2 m) $ <ferrari_mn>

(the case $m = 0$ corresponds to $beta = 0$, i.e. the biquadratic reduction). The depressed quartic factors into two quadratics:

$ (u^2 + m u + y_0 / 2 - n)(u^2 - m u + y_0 / 2 + n) = 0 $ <ferrari_factor>

Each factor is a standard quadratic, giving up to 4 roots for $u$, from which $t_1 = u - c_1 \/ (4 c_4)$.

For numerical implementation, the resolvent cubic should be solved by a robust cubic solver (handling the casus irreducibilis when all three roots are real). The subsequent quadratic solves are straightforward.

== Time-Equality Constraint

Both bodies must be at their respective positions *at the same time*. This is the second equation of the system.

=== Kepler's equation

For orbit $i$, the time $t$ relates to eccentric anomaly via Kepler's equation:

$ n_i (t - tau_i) = E_i - e_i sin E_i $ <kepler>

where $n_i = sqrt(mu \/ a_i^3)$ is the mean motion, $tau_i$ is the epoch of perihelion passage, and $mu$ is the gravitational parameter of the central body.

=== Eliminating time

From @kepler, express the common time $t$ from each orbit and equate:

$ tau_1 + (E_1 - e_1 sin E_1) / n_1 = tau_2 + (E_2 - e_2 sin E_2) / n_2 $

Rearrange:

$ (E_1 - e_1 sin E_1) / n_1 - (E_2 - e_2 sin E_2) / n_2 = tau_2 - tau_1 $ <time_eq_raw>

Since $E_i$ is an angle (defined modulo $2 pi$), the general form accounting for multiple orbital revolutions ($k_1$, $k_2$ complete orbits) is:

$ (E_1 - e_1 sin E_1 + 2 pi k_1) / n_1 - (E_2 - e_2 sin E_2 + 2 pi k_2) / n_2 = tau_2 - tau_1 $ <time_eq_periodic>

Define the epoch offset constant:
$ Delta_tau := tau_2 - tau_1 $ <Delta_tau>

Multiply through by $n_1 n_2$:

$ n_2 (E_1 - e_1 sin E_1 + 2 pi k_1) - n_1 (E_2 - e_2 sin E_2 + 2 pi k_2) = n_1 n_2 Delta_tau $ <time_eq_scaled>

=== Separation into analytical and transcendental parts

Rearrange @time_eq_scaled:

$ n_2 E_1 - n_1 E_2 - n_2 e_1 sin E_1 + n_1 e_2 sin E_2 = n_1 n_2 Delta_tau - 2 pi (n_2 k_1 - n_1 k_2) $ <time_eq_rearranged>

The right-hand side is a constant for each choice of integer pair $(k_1, k_2)$. Define:

$ Phi(k_1, k_2) := n_1 n_2 Delta_tau - 2 pi (n_2 k_1 - n_1 k_2) $ <Phi_def>

So the time-equality constraint is:

$ G(E_1, E_2) := n_2 E_1 - n_1 E_2 - n_2 e_1 sin E_1 + n_1 e_2 sin E_2 - Phi = 0 $ <G_eq>

Note: $G$ is *not* rationalizable via the Weierstrass substitution because it contains both $E_i$ and $sin E_i$ — the $E_i$ terms are transcendental in $t_i = tan(E_i\/2)$. This is the fundamental reason Kepler's equation has no closed-form inverse.

== The Full System

The SOI encounter problem is the $2 times 2$ nonlinear system:

$ cases(
  F(E_1, E_2) = 0 quad &"(distance constraint, polynomial in" t_1 = tan(E_1\/2)"," t_2 = tan(E_2\/2)")",
  G(E_1, E_2) = 0 quad &"(time equality, transcendental)"
) $ <system>

for unknowns $(E_1, E_2) in [0, 2pi) times [0, 2pi)$, parametrized by the integer pair $(k_1, k_2)$.

== Restricting the Search Domain

Before solving the full system, we can drastically narrow the range of $E_1$ (and symmetrically $E_2$) that needs to be searched.

=== Coarse filter: heliocentric radius bounds

For a fixed spacecraft position at $E_1$, the heliocentric distance is $r_1 = a_1 (1 - e_1 cos E_1)$. The planet's heliocentric distance ranges over $r_2 in [a_2(1 - e_2), a_2(1 + e_2)]$ as $E_2$ varies. By the triangle inequality, the minimum possible distance between the two bodies (over all $E_2$ and all mutual orientations) satisfies:

$ d_"min" >= |r_1 - r_2| $

A necessary condition for the spacecraft to be within $r_"SOI"$ of the planet is therefore $|r_1 - r_2| <= r_"SOI"$ for *some* $r_2$ in the planet's range. This requires:

$ a_2(1 - e_2) - r_"SOI" <= r_1 <= a_2(1 + e_2) + r_"SOI" $ <radius_filter>

Substituting $r_1 = a_1(1 - e_1 cos E_1)$ and solving for $cos E_1$:

$ (a_1 - a_2(1 + e_2) - r_"SOI") / (a_1 e_1) <= cos E_1 <= (a_1 - a_2(1 - e_2) + r_"SOI") / (a_1 e_1) $ <E1_bounds>

(with the inequality direction flipped when $e_1 < 0$, but $e_1 >= 0$ by definition). Clamp both sides to $[-1, 1]$ before taking $arccos$. If the interval is empty, no encounter is possible on this orbit pair. If it covers $[-1, 1]$ entirely, no pruning occurs (only possible when the orbits nearly overlap).

For typical interplanetary trajectories, $r_"SOI" << a_2$, and the spacecraft orbit crosses the planet's radial range over a narrow arc. This filter eliminates the vast majority of $E_1$ values. For example, Voyager 2's heliocentric orbit crosses Jupiter's radial range (approximately $4.95$–$5.46$ AU) over a small fraction of its eccentric anomaly.

=== Fine filter: quartic real-root existence

Within the interval surviving @E1_bounds, a tighter test asks: does the quartic $F(t_1, t_2) = 0$ (viewed as a quartic in $t_2$ for fixed $t_1 = tan(E_1\/2)$) have at least one real root?

For a given $E_1$, the quartic in $t_2$ has the same $c_1 = c_3$ structure (by the symmetry of the argument in the section on the quartic structure, applied with the roles of $t_1$ and $t_2$ exchanged). Its coefficients are pure numbers once $t_1$ is fixed. The quartic has real roots if and only if there exists a $t_2 in RR$ (corresponding to $E_2 in [0, 2 pi) without.{pi}$) satisfying it.

Rather than evaluating the full quartic discriminant (which is algebraically unwieldy), a practical approach is:

- *Evaluate $F$ at a few sample $t_2$ values.* If $F$ changes sign, a real root exists by the intermediate value theorem.
- *Check the minimum of $F$ over $t_2$.* Since $F$ is a degree-4 polynomial in $t_2$ with positive leading coefficient (for non-degenerate orbits), it has a global minimum. If $F_"min" <= 0$, real roots exist. The minimum can be found by solving $partial F \/ partial t_2 = 0$ — a cubic in $t_2$, solvable analytically.

This identifies the exact set of $E_1$ values for which the SOI sphere (centered at some planet position) geometrically intersects the spacecraft orbit. The boundary of this set — where the quartic transitions from having real roots to having none — corresponds to the quartic having a double root (tangency of the SOI sphere with the orbit ellipse).

== Solution Procedure

=== Step 1: Enumerate orbit-count pairs

Choose a time window $[t_"min", t_"max"]$. For each orbit, this bounds the number of complete revolutions:

$ k_i in {floor((t_"min" - tau_i) / T_i), ..., ceil((t_"max" - tau_i) / T_i)} $ <k_bounds>

where $T_i = 2 pi \/ n_i$ is the orbital period. For each pair $(k_1, k_2)$, we solve the system @system with the corresponding $Phi(k_1, k_2)$ from @Phi_def.

=== Step 2: Restrict the $E_1$ domain

Apply the coarse heliocentric radius filter @E1_bounds to obtain a candidate interval (or intervals) $cal(I)_1 subset [0, 2 pi)$. Optionally refine with the quartic real-root existence test to tighten $cal(I)_1$ further.

=== Step 3: Solve the distance quartic

For each sampled $E_1 in cal(I)_1$, compute $t_1 = tan(E_1\/2)$ and solve the quartic @quartic_c1c3 in $t_2$ (or equivalently, for each sampled $E_2$, solve the quartic in $t_1$). Use Ferrari's method, or the quadratic/biquadratic reductions from the special cases above when applicable, yielding up to 4 real branches $t_2^((j))(t_1)$, i.e. up to 4 curves $E_2(E_1)$ satisfying the distance constraint.

=== Step 4: Substitute into the time constraint

Along each branch $E_2^((j))(E_1)$, substitute into @G_eq to get a single-variable equation:

$ g^((j))(E_1) := G(E_1, E_2^((j))(E_1)) = 0 $ <branch_eq>

This reduces the $2 times 2$ system to a scalar root-finding problem. The function $g^((j))$ is continuous but transcendental (due to the $E_i$ and $sin E_i$ terms from Kepler's equation).

=== Step 5: Root finding on each branch

Sweep $E_1$ over $cal(I)_1$ on each branch $j$ and locate sign changes of $g^((j))(E_1)$. At each sign change, apply a scalar root-finding method (e.g. Brent's method or the ITP method) to find $E_1^*$ such that $g^((j))(E_1^*) = 0$.

For each root $E_1^*$, recover $E_2^* = 2 arctan(t_2^((j))(tan(E_1^* \/ 2)))$.

=== Step 6: Newton refinement

Ferrari's quartic formula is numerically unstable: it involves nested square and cube roots and suffers from catastrophic cancellation, particularly when roots are nearly degenerate or when polynomial coefficients span many orders of magnitude (typical in orbital mechanics, where distances in SI units produce coefficients differing by $>20$ orders). In practice, Ferrari often yields only $tilde 8$–$10$ correct digits out of $16$. Near branch bifurcation points (where roots merge or split during the $E_1$ sweep), the quartic has near-double roots and accuracy degrades further.

To recover full precision, apply Newton's method on the original $2 times 2$ system @system using the candidate $(E_1^*, E_2^*)$ from Step 5 as initial guess. The Jacobian is:

$ bold(J) = mat(
  (partial F) / (partial E_1), (partial F) / (partial E_2);
  (partial G) / (partial E_1), (partial G) / (partial E_2)
) $

From @G_eq:

$ (partial G) / (partial E_1) = n_2 (1 - e_1 cos E_1), quad (partial G) / (partial E_2) = -n_1 (1 - e_2 cos E_2) $

These are proportional to $r_i \/ a_i$, which is strictly positive for elliptical orbits ($e_i < 1$).

From @compact:

$ (partial F) / (partial E_1) = f'_1(E_1) - 2 (partial S) / (partial E_1), quad (partial F) / (partial E_2) = f'_2(E_2) - 2 (partial S) / (partial E_2) $

where, from @f1:
$ f'_1 = 2 a_1 e_1 (a_1 - a_2 e_2 C_11) sin E_1 + 2 a_2 b_1 e_2 C_21 cos E_1 - a_1^2 e_1^2 sin 2 E_1 $

and from @S_def:
$ (partial S) / (partial E_1) = -C_11 a_1 a_2 sin E_1 cos E_2 - C_12 a_1 b_2 sin E_1 sin E_2 + C_21 b_1 a_2 cos E_1 cos E_2 + C_22 b_1 b_2 cos E_1 sin E_2 $

(and analogously for $partial F \/ partial E_2$).

One or two Newton iterations typically suffice to reach machine precision.

=== Step 7: Recover encounter time

For each refined solution $(E_1^*, E_2^*)$, the encounter time is:

$ t^* = tau_i + (E_i^* - e_i sin E_i^* + 2 pi k_i) / n_i $ <encounter_time>

(using either $i = 1$ or $i = 2$ — both give the same $t^*$ at a true solution).

== Summary

+ *Precompute*: $bold(C) = bold(A)^top bold(B)$, constants $K$, $lambda_i$, $sigma_i$, $q_i$, mean motions $n_i$, epoch offset $Delta_tau$.
+ *Enumerate*: Loop over integer orbit pairs $(k_1, k_2)$ within the desired time window.
+ *Restrict domain*: Apply heliocentric radius bounds @E1_bounds (and optionally the quartic real-root existence test) to obtain a narrow candidate interval $cal(I)_1 subset [0, 2 pi)$.
+ *Distance polynomial*: For each $(k_1, k_2)$, construct the bidegree-$(4,4)$ polynomial @poly_dist in $(t_1, t_2)$.
+ *Branch extraction*: For each sampled $E_1 in cal(I)_1$, solve the quartic in $t_2$ analytically, obtaining up to 4 branches $E_2(E_1)$.
+ *Scalar root-finding*: On each branch, find zeros of $g^((j))(E_1) = G(E_1, E_2^((j))(E_1)) = 0$ via sign-change detection and Brent's/ITP method.
+ *Newton refinement*: Polish each candidate on the full $2 times 2$ system @system to recover precision lost by Ferrari's formula.
+ *Recover time*: Compute encounter epoch via @encounter_time.
