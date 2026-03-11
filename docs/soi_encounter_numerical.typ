#set document(title: "SOI Encounter Derivation")
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

== Newton's Method

We seek to minimise $f(E_1, E_2) := r_1^2 + r_2^2 - bold(r)_1^top bold(C) bold(r)_2 - r_"SOI"^2$ using Newton's method. Each iteration solves the $2 times 2$ linear system $bold(H) bold(delta) = -nabla f$ for the step $bold(delta) = (delta E_1, delta E_2)^top$, then updates $E_i <- E_i + delta E_i$.

=== Deriving the gradient and Hessian

The objective is $f = r_1^2 + r_2^2 - bold(r)_1^top bold(C) bold(r)_2 - r_"SOI"^2$, with $r_"SOI"^2$ constant. We differentiate each part with respect to $E_i$.

==== Self-terms $r_i^2$

Using $r_i = a_i (1 - e_i cos E_i)$:

$ (partial r_i^2) / (partial E_i) = 2 a_i^2 e_i sin E_i (1 - e_i cos E_i) $ <self_grad>

$ (partial^2 r_i^2) / (partial E_i^2) = 2 a_i^2 e_i (cos E_i - e_i + 2 e_i sin^2 E_i) $ <self_hess>

and $partial r_i^2 \/ partial E_j = 0$ for $i != j$.

==== Cross-term $bold(r)_1^top bold(C) bold(r)_2$

The position vector @orbital_pos can be factored as $bold(r)_i = op("diag")(a_i, b_i) bold(p)_i$, where $op("diag")(a_i, b_i) := mat(a_i, 0; 0, b_i)$ is a diagonal matrix and $bold(p)_i$ is a dimensionless position-direction vector:

$ bold(p)_i = vec(cos E_i - e_i, sin E_i) $ <p_def>

Substituting $bold(r)_i = op("diag")(a_i, b_i) bold(p)_i$ into the cross-term and using $(bold(X) bold(p))^top = bold(p)^top bold(X)^top$:

$ bold(r)_1^top bold(C) bold(r)_2
    = (op("diag")(a_1, b_1) bold(p)_1)^top bold(C) op("diag")(a_2, b_2) bold(p)_2
    = bold(p)_1^top op("diag")(a_1, b_1)^top bold(C) op("diag")(a_2, b_2) bold(p)_2 $

Since diagonal matrices are symmetric ($op("diag")(x, y)^top = op("diag")(x, y)$):

$ bold(r)_1^top bold(C) bold(r)_2 = bold(p)_1^top op("diag")(a_1, b_1) bold(C) op("diag")(a_2, b_2) bold(p)_2 $

We have a chain of five matrices. Applying associativity ($bold(A)(bold(B) bold(C)) = (bold(A) bold(B)) bold(C)$) twice — first to $(bold(p)_1^top dot op("diag")(a_1, b_1)) dot bold(C) = bold(p)_1^top dot (op("diag")(a_1, b_1) dot bold(C))$, then to $(bold(p)_1^top dot (op("diag")(a_1, b_1) dot bold(C))) dot op("diag")(a_2, b_2) = bold(p)_1^top dot ((op("diag")(a_1, b_1) dot bold(C)) dot op("diag")(a_2, b_2))$ — lets us group the three constant middle factors into a single matrix. We define the *scaled coupling matrix*:

$ bold(M) := op("diag")(a_1, b_1) bold(C) op("diag")(a_2, b_2) = mat(C_(11) a_1 a_2, C_(12) a_1 b_2; C_(21) a_2 b_1, C_(22) b_1 b_2) $ <M>

giving $bold(r)_1^top bold(C) bold(r)_2 = bold(p)_1^top bold(M) bold(p)_2$.

Since $bold(M)$ is constant, differentiation acts only on $bold(p)_1$ and $bold(p)_2$. Define the derivative vectors:

$ hat(bold(w))_i := dif bold(p)_i / (dif E_i) = vec(-sin E_i, cos E_i), quad hat(bold(u))_i := vec(cos E_i, sin E_i) $ <vecs>


To differentiate $bold(p)_1^top bold(M) bold(p)_2$ with respect to $E_1$, note that only $bold(p)_1$ depends on $E_1$. By associativity, $bold(p)_1^top bold(M) bold(p)_2 = bold(p)_1^top (bold(M) bold(p)_2)$, which is a dot product of $bold(p)_1$ with the constant vector $bold(M) bold(p)_2$. The derivative of a dot product with one constant factor is:

$ (partial (bold(p)_1^top (bold(M) bold(p)_2))) / (partial E_1) = hat(bold(w))_1^top (bold(M) bold(p)_2) = hat(bold(w))_1^top bold(M) bold(p)_2 $ <cross_grad>

Similarly, for $E_2$ only $bold(p)_2$ depends on $E_2$. By associativity, $bold(p)_1^top bold(M) bold(p)_2 = (bold(p)_1^top bold(M)) bold(p)_2$, a dot product of the constant row vector $bold(p)_1^top bold(M)$ with $bold(p)_2$:

$ (partial ((bold(p)_1^top bold(M)) bold(p)_2)) / (partial E_2) = (bold(p)_1^top bold(M)) hat(bold(w))_2 = bold(p)_1^top bold(M) hat(bold(w))_2 $

The same reasoning applies to higher derivatives, replacing $bold(p)_i$ by successive derivatives $hat(bold(w))_i$, $-hat(bold(u))_i$:

$ (partial^2 (bold(p)_1^top bold(M) bold(p)_2)) / (partial E_1^2) = -hat(bold(u))_1^top bold(M) bold(p)_2, quad (partial^2 (bold(p)_1^top bold(M) bold(p)_2)) / (partial E_2^2) = -bold(p)_1^top bold(M) hat(bold(u))_2 $ <cross_hess_diag>

$ (partial^2 (bold(p)_1^top bold(M) bold(p)_2)) / (partial E_1 partial E_2) = hat(bold(w))_1^top bold(M) hat(bold(w))_2 $ <cross_hess_off>

==== Combined expressions

Combining $f = r_1^2 + r_2^2 - bold(p)_1^top bold(M) bold(p)_2 - r_"SOI"^2$ using @self_grad with @cross_grad, and @self_hess with @cross_hess_diag and @cross_hess_off:

=== Gradient

$ (partial f) / (partial E_1) = 2 a_1^2 e_1 sin E_1 (1 - e_1 cos E_1) - hat(bold(w))_1^top bold(M) bold(p)_2 $ <grad1>

$ (partial f) / (partial E_2) = 2 a_2^2 e_2 sin E_2 (1 - e_2 cos E_2) - bold(p)_1^top bold(M) hat(bold(w))_2 $ <grad2>

=== Hessian

The Hessian $bold(H)$ is symmetric ($H_(12) = H_(21)$):

$ H_(11) = 2 a_1^2 e_1 (cos E_1 - e_1 + 2 e_1 sin^2 E_1) + hat(bold(u))_1^top bold(M) bold(p)_2 $ <H11>

$ H_(22) = 2 a_2^2 e_2 (cos E_2 - e_2 + 2 e_2 sin^2 E_2) + bold(p)_1^top bold(M) hat(bold(u))_2 $ <H22>

$ H_(12) = -hat(bold(w))_1^top bold(M) hat(bold(w))_2 $ <H12>

=== Solving the Newton step

The $2 times 2$ system is solved directly via Cramer's rule:

$ Delta = H_(11) H_(22) - H_(12)^2 $

$ delta E_1 = ((partial f) / (partial E_1) H_(22) - (partial f) / (partial E_2) H_(12)) / (-Delta), quad delta E_2 = ((partial f) / (partial E_2) H_(11) - (partial f) / (partial E_1) H_(12)) / (-Delta) $

All coupling terms are bilinear forms $bold(x)^top bold(M) bold(y)$ with 2D vectors, each requiring 4 multiplies and 3 adds. The five needed dot products ($hat(bold(w))_1^top bold(M) bold(p)_2$, $bold(p)_1^top bold(M) hat(bold(w))_2$, $hat(bold(u))_1^top bold(M) bold(p)_2$, $bold(p)_1^top bold(M) hat(bold(u))_2$, $hat(bold(w))_1^top bold(M) hat(bold(w))_2$) can share the intermediate products $bold(M) bold(p)_2$, $bold(M) hat(bold(w))_2$, $bold(M) hat(bold(u))_2$ (each a 2D matrix-vector multiply).

== Coarse Encounter Interval

Before running Newton's method, we restrict the search to $E_1$ values where the spacecraft's heliocentric distance overlaps with the planet's distance range $plus.minus r_"SOI"$.

=== Radial overlap condition

The heliocentric distance of the spacecraft is $r_1 = a_1 (1 - e_1 cos E_1)$. An encounter requires:

$ a_2 (1 - e_2) - r_"SOI" <= r_1 <= a_2 (1 + e_2) + r_"SOI" $

Substituting $r_1 = a_1 (1 - e_1 cos E_1)$ and solving for $cos E_1$:

$ (a_1 - a_2 (1 + e_2) - r_"SOI") / (a_1 e_1) <= cos E_1 <= (a_1 - a_2 (1 - e_2) + r_"SOI") / (a_1 e_1) $ <coarse_bounds>

Clamping both sides to $[-1, 1]$ and applying $arccos$ (which reverses the inequality) gives two symmetric intervals $[E_"lo", E_"hi"]$ and $[-E_"lo", -E_"hi"]$ in $E_1$. If no valid interval exists (i.e.~the clamped range is empty), no encounter is possible at any $E_1$.

=== Initial guess via parabolic interpolation

For each of the two symmetric $E_1$ intervals, we evaluate $f$ at three points: the endpoints and midpoint. Here $E_2$ is estimated by projecting the spacecraft's 3D position onto the planet's orbital plane and inverting the ellipse parametrisation:

$ E_2 = op("atan2")(q_y \/ b_2, quad q_x \/ a_2 + e_2) $

where $bold(q) = bold(B)^top bold(A) bold(r)_1$ is the projection. This matches the eccentric anomaly that $bold(q)$ would have if it lay on the planet's ellipse.

Given three samples $(x_0, f_0)$, $(x_1, f_1)$, $(x_2, f_2)$, the minimiser of the interpolating quadratic is:

$ E_1^* = x_1 - 1/2 ((x_1 - x_0)^2 (f_1 - f_2) - (x_1 - x_2)^2 (f_1 - f_0)) / ((x_1 - x_0)(f_1 - f_2) - (x_1 - x_2)(f_1 - f_0)) $ <parabolic>

Newton's method is then run from the best initial guess across both branches. If the first branch fails to converge to $f <= r_"SOI"^2$, the second branch is tried.

== Encounter Intervals

The goal is to find the set of $(E_1, E_2)$ pairs where the distance is at most $r_"SOI"$:

$ cal(R) := {(E_1, E_2) : f(E_1, E_2) <= r_"SOI"^2} $

=== $E_2$ interval for fixed $E_1$

For a fixed $E_1$, all terms depending on $E_1$ are constants. Define $bold(v) := bold(M)^top bold(p)_1$, so that $bold(p)_1^top bold(M) bold(p)_2 = bold(v)^top bold(p)_2 = v_1 (cos E_2 - e_2) + v_2 sin E_2$. Then:

$ f(E_2) = underbrace(r_1^2 + a_2^2 + v_1 e_2 - r_"SOI"^2, kappa) - (2 a_2^2 e_2 + v_1) cos E_2 - v_2 sin E_2 + a_2^2 e_2^2 cos^2 E_2 $ <f_fixed_E1>

Applying $cos^2 E_2 = (1 + cos 2 E_2) / 2$:

$ f(E_2) = kappa' - (2 a_2^2 e_2 + v_1) cos E_2 - v_2 sin E_2 + (a_2^2 e_2^2) / 2 cos 2 E_2 $ <f_fixed_E1_expanded>

where $kappa' = kappa + a_2^2 e_2^2 \/ 2$. This mixes first and second harmonics of $E_2$.

==== Weierstrass substitution

Substituting $t = tan(E_2 \/ 2)$, so that $cos E_2 = (1 - t^2) \/ (1 + t^2)$ and $sin E_2 = 2t \/ (1 + t^2)$, and using $cos 2 E_2 = 2 cos^2 E_2 - 1$:

$ cos 2 E_2 = (2(1 - t^2)^2) / (1 + t^2)^2 - 1 = (1 - 6t^2 + t^4) / (1 + t^2)^2 $

Multiplying @f_fixed_E1_expanded through by $(1 + t^2)^2$ yields a quartic polynomial in $t$:

$ P(t) = (1 + t^2)^2 f(E_2) = 0 $

The real roots $t_1, ..., t_k$ of $P(t) = 0$ (with $k <= 4$) correspond to the $E_2$ boundary values via $E_2 = 2 arctan t_i$. The encounter intervals are the segments between consecutive roots where $P(t) <= 0$.

=== $E_1$ interval for fixed $E_2$

By symmetry, fixing $E_2$ and defining $bold(w) := bold(M) bold(p)_2$ gives the analogous expression:

$ f(E_1) = underbrace(r_2^2 + a_1^2 + w_1 e_1 - r_"SOI"^2, lambda) - (2 a_1^2 e_1 + w_1) cos E_1 - w_2 sin E_1 + a_1^2 e_1^2 cos^2 E_1 $

The same Weierstrass substitution $s = tan(E_1 \/ 2)$ yields a quartic in $s$.

=== Boundary of $cal(R)$

The boundary $partial cal(R)$ is the level set $f(E_1, E_2) = r_"SOI"^2$. It can be traced by:

+ Finding any point on $partial cal(R)$ using Newton's method (§1.5).
+ Following the implicit curve $f = r_"SOI"^2$ via the tangent direction. The gradient $nabla f$ is normal to the level set, so the tangent is:

$ bold(t) = (-partial f \/ partial E_2, quad partial f \/ partial E_1) $

A predictor-corrector scheme (Euler step along $bold(t)$, then Newton correction back to $f = r_"SOI"^2$) traces the full boundary.

=== Time constraint

Eccentric anomalies are linked to time via Kepler's equation:

$ M_i = E_i - e_i sin E_i = n_i (t - t_(0,i)) $

where $n_i = 2 pi \/ T_i$ is the mean motion and $t_(0,i)$ is the epoch of periapsis passage. At a shared time $t$, the relationship between $E_1$ and $E_2$ is:

$ E_1 - e_1 sin E_1 - n_1 / n_2 (E_2 - e_2 sin E_2) = n_1 (t_(0,2) - t_(0,1)) + (n_1 - n_2) / n_2 M_(0,2) $

This is a curve in $(E_1, E_2)$ space. An encounter occurs where this curve intersects the feasible region $cal(R)$.

