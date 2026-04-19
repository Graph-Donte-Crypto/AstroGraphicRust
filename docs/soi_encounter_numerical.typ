#set document(title: "An algorithm for SOI encounter computation on Keplerian orbits")
#set page(margin: 1cm, numbering: "1")
#set text(size: 11pt)
#set heading(numbering: "1.")
#set math.equation(numbering: "(1)")
#set math.vec(delim: "[")

#page(numbering: none)[
  #v(1fr)
  #align(center)[
    #text(size: 20pt, weight: "bold")[An algorithm for SOI encounter computation\ on Keplerian orbits]
    #v(2em)
    #text(size: 13pt)[Artur Sinila]
    #v(0.5em)
    #text(size: 12pt)[Astro Graphic Rust Research Foundation]
    #v(1em)
    #text(size: 11pt)[March 2026]
  ]
  #v(1fr)
]

#page(numbering: none)[
  #outline()
]

#counter(page).update(1)

= Problem Statement

Given two Keplerian orbits (spacecraft orbit 1, planet orbit 2) in heliocentric ecliptic coordinates, find the eccentric anomalies $E_1, E_2$ such that the distance between the spacecraft and the planet equals the planet's sphere-of-influence radius $r_"SOI"$. It is possible that such $E_1, E_2$ don't exist, the algorithm needs to handle this case as well. The spacecraft orbit may be elliptic ($e_1 < 1$) or hyperbolic ($e_1 > 1$); the planet orbit is always elliptic.

$ norm(bold(A) bold(r)_1 (E_1) - bold(B) bold(r)_2 (E_2)) = r_"SOI" $

where $bold(A)$ and $bold(B)$ are known constant $3 times 2$ rotation matrices mapping from orbital-plane 2D coordinates to 3D heliocentric ecliptic coordinates.

We divide this hard problem into two easier ones:
1. Finding tight *ranges* of $E_1, E_2$ *where* encounters are *geometrically* possible, ignoring real positions of the bodies
2. Finding precise *values* of $E_1$ and time $t$ *when* the spacecraft *actually* enters sphere-of-influence of the planet

= Position in Orbital Plane

For an elliptic orbit, the 2D position vector in the orbital plane as a function of eccentric anomaly is:

$ bold(r)_i (E_i) = vec(a_i (cos E_i - e_i), b_i sin E_i) $ <orbital_pos>

For a hyperbolic spacecraft orbit:

$ bold(r)_1 (H_1) = vec(a_1 (cosh H_1 - e_1), b_1 sinh H_1) $ <orbital_pos_hyp>

= Distance Constraint

Square both sides of the constraint:

$ (bold(A) bold(r)_1 - bold(B) bold(r)_2)^top (bold(A) bold(r)_1 - bold(B) bold(r)_2) = r_"SOI"^2 $ <constraint_sq>

Expand the left-hand side:

$ bold(r)_1^top bold(A)^top bold(A) bold(r)_1 - 2 bold(r)_1^top bold(A)^top bold(B) bold(r)_2 + bold(r)_2^top bold(B)^top bold(B) bold(r)_2 = r_"SOI"^2 $ <expanded>

== Orthonormality of $bold(A)$ and $bold(B)$

Since $bold(A)$ and $bold(B)$ are $3 times 2$ matrices whose columns are orthonormal (they embed an orthonormal frame from the orbital plane into 3D space):

$ bold(A)^top bold(A) = bold(I)_(2 times 2), quad bold(B)^top bold(B) = bold(I)_(2 times 2) $ <ortho>

Therefore the self-terms simplify:

$ bold(r)_1^top bold(A)^top bold(A) bold(r)_1 = bold(r)_1^top bold(r)_1 = |bold(r)_1|^2 =: r_1^2 $
$ bold(r)_2^top bold(B)^top bold(B) bold(r)_2 = bold(r)_2^top bold(r)_2 = |bold(r)_2|^2 =: r_2^2 $

== Coupling matrix $bold(C)$

Define the $2 times 2$ matrix:

$ bold(C) := 2 bold(A)^top bold(B) $ <C_def>

This encodes the entire mutual orientation of the two orbital planes. @expanded becomes:

$ r_1^2 + r_2^2 - bold(r)_1^top bold(C) bold(r)_2 = r_"SOI"^2 $ <constraint_C>

Note: $bold(C)$ is *not* necessarily symmetric — it is a general $2 times 2$ real matrix with $|C_(j k)| <= 2$.

== Scaled coupling matrix $bold(M)$

The position vector @orbital_pos can be factored as $bold(r)_i = op("diag")(a_i, b_i) bold(p)_i$, where $op("diag")(a_i, b_i) := mat(a_i, 0; 0, b_i)$ is a diagonal matrix and $bold(p)_i$ is a dimensionless position-direction vector:

$ bold(p)_i = vec(cos E_i - e_i, sin E_i) $ <p_def>

For a hyperbolic spacecraft orbit:

$ bold(p)_1 = vec(cosh H_1 - e_1, sinh H_1) $

Substituting $bold(r)_i = op("diag")(a_i, b_i) bold(p)_i$ into the cross-term and using $(bold(X) bold(p))^top = bold(p)^top bold(X)^top$:

$ bold(r)_1^top bold(C) bold(r)_2
    = (op("diag")(a_1, b_1) bold(p)_1)^top bold(C) op("diag")(a_2, b_2) bold(p)_2
    = bold(p)_1^top op("diag")(a_1, b_1)^top bold(C) op("diag")(a_2, b_2) bold(p)_2 $

Since diagonal matrices are symmetric ($op("diag")(x, y)^top = op("diag")(x, y)$):

$ bold(r)_1^top bold(C) bold(r)_2 = bold(p)_1^top op("diag")(a_1, b_1) bold(C) op("diag")(a_2, b_2) bold(p)_2 $

We have a chain of five matrices. Applying associativity ($bold(A)(bold(B) bold(C)) = (bold(A) bold(B)) bold(C)$) twice — first to $(bold(p)_1^top dot op("diag")(a_1, b_1)) dot bold(C) = bold(p)_1^top dot (op("diag")(a_1, b_1) dot bold(C))$, then to $(bold(p)_1^top dot (op("diag")(a_1, b_1) dot bold(C))) dot op("diag")(a_2, b_2) = bold(p)_1^top dot ((op("diag")(a_1, b_1) dot bold(C)) dot op("diag")(a_2, b_2))$ — lets us group the three constant middle factors into a single *scaled coupling matrix* $bold(M)$.

$ bold(M) = op("diag")(a_1, b_1) bold(C) op("diag")(a_2, b_2) $
$ bold(r)_1^top bold(C) bold(r)_2 = bold(p)_1^top bold(M) bold(p)_2 $

Let's introduce matrix $bold(D)$ and express $bold(M)$ in terms of it ($circle.small$ denotes element-wise product):

$ bold(D) := vec(a_1, b_1) vec(a_2, b_2)^top = mat(a_1 a_2, a_1 b_2; b_1 a_2, b_1 b_2), quad bold(M) := bold(D) circle.small bold(C) $ <M>

= Coarse encounter intervals

We restrict the search to eccentric anomaly values where the spacecraft's heliocentric distance lies between planetary perihelion and aphelion $plus.minus r_"SOI"$.

== Radial overlap condition

An encounter requires the spacecraft distance to lie within:

$ a_2 (1 - e_2) - r_"SOI" <= r_1 <= a_2 (1 + e_2) + r_"SOI" $

*Elliptic spacecraft.* Using $r_1 = a_1 (1 - e_1 cos E_1)$ and solving for $cos E_1$:

$ (a_1 - a_2 (1 + e_2) - r_"SOI") / (a_1 e_1) <= cos E_1 <= (a_1 - a_2 (1 - e_2) + r_"SOI") / (a_1 e_1) $ <coarse_bounds>

Clamping both sides to $[-1, 1]$ and applying $arccos$ (which reverses the inequality) gives two symmetric intervals $[E_"lo", E_"hi"]$ and $[-E_"lo", -E_"hi"]$ in $E_1$. If no valid interval exists (i.e.~the clamped range is empty), no encounter is possible at any $E_1$.

*Hyperbolic spacecraft.* Using $r_1 = a_1 (1 - e_1 cosh H_1)$ and solving for $cosh H_1$ (noting $a_1 e_1 < 0$, so dividing reverses the inequality):

$ (a_1 - a_2 (1 - e_2) + r_"SOI") / (a_1 e_1) <= cosh H_1 <= (a_1 - a_2 (1 + e_2) - r_"SOI") / (a_1 e_1) $ <coarse_bounds_hyp>

Clamping the lower bound to $max(dots, 1)$ (since $cosh H_1 >= 1$) and applying $op("arcosh")$ gives a single symmetric interval $[-H_"hi", -H_"lo"] union [H_"lo", H_"hi"]$. If the clamped range is empty, no encounter is possible.

== Initial guess via midpoint and atan2 projection <sec_atan2_guess>

Newton's method (@sec_newton) converges rapidly once seeded near a local minimum of $f$. We produce the seed in two cheap steps: take the midpoint of the radial-overlap interval as $E_1^*$, then estimate the matching $E_2^*$ via a closed-form atan2 projection.

=== Selecting $E_1^*$ as the interval midpoint

The radial-overlap condition @coarse_bounds produces the symmetric intervals $[E_"lo", E_"hi"]$ and $[-E_"hi", -E_"lo"]$. For each branch we seed Newton's method at the midpoint:

$ E_1^* = (E_"lo" + E_"hi") \/ 2 $ <midpoint>

No $f$ evaluations are spent on the seed itself — the interval from the radial-overlap condition already brackets the region where encounters are geometrically possible, and its midpoint is within half the interval width of the true minimum. Newton's quadratic convergence absorbs the residual in a handful of iterations.

=== Estimating $E_2$ from $E_1$ via atan2 projection <atan2_section>

Given $E_1^*$, the spacecraft's position in the ecliptic frame is $bold(A) bold(r)_1 (E_1^*)$. Project it onto the planet's orbital plane and express the result in the planet's 2D coordinates:

$ bold(q) := bold(B)^top bold(A) bold(r)_1 (E_1^*) $ <proj_q>

We now want an $E_2$ such that $bold(r)_2 (E_2)$ is as close as possible to $bold(q)$. Finding the *exact* closest-point-on-ellipse has no closed form below degree 4 (@sec_nodal_guess notes the same quartic structure), so we look for a cheap approximation derived from the ellipse's natural parametrisation.

==== Deriving atan2 projection from an affine transformation

The ellipse has Sun (focus) at the origin and *centre* at $(-a_2 e_2, 0)$, with equation
$ (x + a_2 e_2)^2 \/ a_2^2 + y^2 \/ b_2^2 = 1. $

Define the affine map
$ T : (x, y) |-> (x \/ a_2 + e_2, quad y \/ b_2). $ <T_def>

$T$ shifts the ellipse centre onto the new origin (via $+ a_2 e_2$ in $x$ after the $1\/a_2$ scaling) and rescales both axes so the ellipse equation becomes
$ x'^2 + y'^2 = 1, $
a *unit circle centred at the new origin*. The Sun, which was at the original origin, is carried by $T$ to $(e_2, 0)$ in the new frame — it is no longer at the origin.

The parametrisation maps particularly cleanly. Applying $T$ to $bold(r)_2 (E_2) = (a_2 (cos E_2 - e_2), #h(0.2em) b_2 sin E_2)$:
$ T(bold(r)_2 (E_2)) = (cos E_2, quad sin E_2). $

So in the $T$-frame, $E_2$ is exactly the angular coordinate on the unit circle measured from its centre (which is also the new origin). This is the geometric meaning of eccentric anomaly, hiding in plain sight in the parametrisation.

Closest-point-on-unit-circle is now a genuinely solvable problem. Let $(p_x, p_y) := T(bold(q)) = (q_x \/ a_2 + e_2, #h(0.2em) q_y \/ b_2)$. A point on the unit circle is parametrised by $E_2$ as $(cos E_2, sin E_2)$. We seek the $E_2$ minimising

$ D^2 (E_2) := (cos E_2 - p_x)^2 + (sin E_2 - p_y)^2 $

Differentiating:

$ (d D^2) / (d E_2) = 2 (p_x sin E_2 - p_y cos E_2) $

Setting $d D^2 \/ d E_2 = 0$:

$ p_x sin E_2 = p_y cos E_2 quad ==> quad E_2 = op("atan2")(p_y, p_x) $

The second derivative $d^2 D^2 \/ d E_2^2 = 2 (p_x cos E_2 + p_y sin E_2)$ evaluated at this root is $2 sqrt(p_x^2 + p_y^2) > 0$, confirming it is a minimum (the other stationary point, at $E_2 + pi$, is the maximum). Substituting the definitions of $p_x, p_y$:

$ E_2 = op("atan2")(q_y \/ b_2, quad q_x \/ a_2 + e_2) $ <atan2_proj>

The derivation implicitly assumes $(p_x, p_y) != (0, 0)$, or equivalently $bold(q) != (-a_2 e_2, 0)$. This degenerate case occurs only when $bold(q)$ sits exactly at the ellipse centre, far inside the orbit — not geometrically relevant for encounter problems, where $bold(q)$ is on the order of the planet's heliocentric distance.

==== Why this works (and where it approximates)

When the two orbital planes coincide ($i_"mut" = 0$), $bold(q)$ lies on the planet's ellipse, $T(bold(q))$ lies on the unit circle, and the closest-point-on-circle is $T(bold(q))$ itself: @atan2_proj returns the exact $E_2$.

For inclined orbits, the orthogonal projection $bold(B)^top bold(A) bold(r)_1$ shortens $bold(q)$ relative to its true heliocentric extent, so $T(bold(q))$ moves off the unit circle and @atan2_proj returns only the *angle* of the closest circle point, not the true minimiser of the Euclidean distance on the original ellipse (the affine $T$ stretches the metric non-uniformly). This is the structural source of the approximation error.

@fig_atan2_geometry shows both frames. A naïve alternative — "pick the ellipse point along the Sun-ray through $bold(q)$" — is strictly worse: the focal ray is not perpendicular to the ellipse at the intersection because the focus is off-centre, so that intersection is not a closest point. Only the affine transformation $T$, which places the *ellipse centre* (not the Sun) at the new origin, makes ray-from-origin geometrically meaningful.

#figure(
  image("atan2_geometry.svg", width: 100%),
  caption: [Geometric derivation of the atan2 projection, for an ellipse with $a = 1$, $e = 0.5$. *Left:* original coordinates; Sun (yellow star) at the focus = origin, ellipse centre at $(-a e, 0)$ is a distinct point. An off-ellipse probe point $bold(q)$ (blue dot) has three candidate eccentric anomalies: the Sun-ray intersection (red triangle), the atan2 projection @atan2_proj (green square), and the true minimum-distance point (magenta diamond, numerical). *Right:* the affine transformation $T$ maps the ellipse to a unit circle. The new origin coincides with the *ellipse centre*; the Sun has moved to $(e, 0)$. The atan2 projection is now simply the ray from the new origin through $T(bold(q))$, hitting the unit circle at the closest circle point. Back-transforming recovers the green square.],
) <fig_atan2_geometry>

==== Accuracy

When the orbital planes coincide, the projection is exact and @atan2_proj returns the correct $E_2$. For inclined orbits, the error grows with the mutual inclination $i_"mut"$ and with the spacecraft eccentricity $e_1$ (which spreads the spacecraft trajectory further from the planet's plane). Numerical evaluation for a Pluto-like target ($i_"mut" = 17°$, $e_2 = 0.25$) shows the maximum $E_2$ error is under $2°$ for moderate $e_1 < 0.5$, growing to $~ 9°$ at favourable Hohmann-like geometries ($e_1 approx 0.95$) and up to $~ 63°$ at unfavourable ascending-node orientations. Newton's method converges reliably from any of these seeds, so the formula is adequate despite its worst-case degradation.

== Initial guess via mutual line of nodes <sec_nodal_guess>

An alternative initial guess exploits the geometry of the two orbital planes directly. The *mutual line of nodes* is the intersection of the two planes — a line through the common focus (the Sun). Its direction is:

$ bold(ell) = hat(bold(n))_1 times hat(bold(n))_2, quad hat(bold(n))_1 = bold(A)_(:,1) times bold(A)_(:,2), quad hat(bold(n))_2 = bold(B)_(:,1) times bold(B)_(:,2) $ <node_line>

where $hat(bold(n))_1$ is the unit normal to the spacecraft's orbital plane and $hat(bold(n))_2$ is the unit normal to the planet's orbital plane (each obtained as the cross product of the two orthonormal columns of $bold(A)$ and $bold(B)$ respectively). Hence $|bold(ell)| = sin(i_"mut")$ with $i_"mut"$ the mutual inclination between the two planes. When both bodies lie on the same ray from the Sun along $bold(ell)$, the out-of-plane component of their separation vanishes — this is a natural 3D minimum-distance configuration.

Project $bold(ell)$ into each orbital plane: $bold(lambda)_1 = bold(A)^top bold(ell)$, $bold(lambda)_2 = bold(B)^top bold(ell)$ (both 2D). The eccentric anomaly at which $bold(r)_i (E_i)$ is parallel to $bold(lambda)_i$ satisfies the cross-product equation $(bold(r)_i times bold(lambda)_i)_z = 0$:

$ a_i lambda_y cos E_i - b_i lambda_x sin E_i = a_i e_i lambda_y $ <node_ell_eq>

which using $A cos E - B sin E = R cos(E - phi.alt)$ with $R = sqrt((a_i lambda_y)^2 + (b_i lambda_x)^2)$, $phi.alt = op("atan2")(-b_i lambda_x, #h(0.2em) a_i lambda_y)$ reduces to:

$ E_i = phi.alt plus.minus arccos((a_i e_i lambda_y) / R) $ <node_ell_sol>

The two roots correspond to $bold(r)_i ∥ +bold(ell)$ and $bold(r)_i ∥ -bold(ell)$; classify by the sign of $bold(r)_i (E_i) dot bold(lambda)_i$. The *same-side* pairings $(E_1^+, E_2^+)$ and $(E_1^-, E_2^-)$ are the two candidate initial guesses.

*Hyperbolic spacecraft.* @node_ell_eq becomes $A cosh H - B sinh H = a_1 e_1 lambda_y$. When $A^2 > B^2$, set $R = op("sgn")(A) sqrt(A^2 - B^2)$, $phi.alt = op("atanh")(B\/A)$, giving $H_1 = phi.alt plus.minus op("arcosh")(a_1 e_1 lambda_y \/ R)$ whenever $a_1 e_1 lambda_y \/ R >= 1$. If the discriminant or $op("arcosh")$ argument fails, the line of nodes does not intersect the reachable branch of the hyperbola.

*Experimental comparison.* We evaluated both initial-guess strategies against four Voyager 2 encounters — three elliptic (Earth, Mars, Jupiter) and one hyperbolic (Saturn, using the Jupiter-to-Saturn leg) — and report the 3D separation $d$ at the guess alongside the separation $d^*$ at the converged true minimum.

#figure(
  table(
    columns: (auto, auto, auto, auto, auto, auto),
    align: (left, right, right, right, right, right),
    table.header[*Target*][*$i_"mut"$*][*$d^*$ (km)*][*$d_"atan2"$ (km)*][*$d_"nod"$ (km)*][*$d_"atan2" \/ d_"nod"$*],
    [Earth (ell.)],    [$5.02°$],  [$2.57 times 10^6$], [$3.17 times 10^6$], [$4.21 times 10^6$], [$0.75$],
    [Mars  (ell.)],    [$5.11°$],  [$1.38 times 10^7$], [$1.56 times 10^7$], [$5.24 times 10^7$], [$0.30$],
    [Jupiter (ell.)],  [$5.99°$],  [$2.59 times 10^5$], [$1.01 times 10^7$], [$2.00 times 10^6$], [$bold(5.05)$],
    [Saturn (hyp.)],   [$0.26°$],  [$6.45 times 10^5$], [$1.24 times 10^7$], [$1.29 times 10^8$], [$0.097$],
  ),
  caption: [Initial-guess 3D separation for the midpoint-plus-atan2 ($d_"atan2"$) and mutual-nodal ($d_"nod"$) strategies, versus the converged true minimum $d^*$. Earth/Mars are non-encounters (Voyager 2 does not enter their SOIs) and serve only as geometric probes. Jupiter is the intended close encounter, where the nodal guess wins by $5.05 times$. Saturn at $i_"mut" = 0.26°$ is a worst case for the nodal method: the line of nodes is poorly defined and the nodal root lands on the wrong asymptotic branch of the hyperbola.],
) <tab_initial_guess>

@tab_initial_guess supports a simple dispatch rule: compute both guesses (each is $cal(O)(1)$), evaluate $f$ at each, and seed Newton with whichever yields smaller $d$.


= Finding minimum distance using Newton's Method <sec_newton>

We seek to minimise $f(E_1, E_2) := r_1^2 + r_2^2 - bold(p)_1^top bold(M) bold(p)_2 - r_"SOI"^2$ using Newton's method. Each iteration solves the $2 times 2$ linear system $bold(H) bold(delta) = -nabla f$ for the step $bold(delta) = (delta E_1, delta E_2)^top$, then updates $E_i <- E_i + delta E_i$.

== Deriving the gradient and Hessian <sec_grad_hess>

We differentiate each term with respect to $E_i$.

=== Self-terms $r_i^2$

Using $r_i = a_i (1 - e_i cos E_i)$:

$ (partial r_i^2) / (partial E_i) = 2 a_i^2 e_i sin E_i (1 - e_i cos E_i) $ <self_grad>

$ (partial^2 r_i^2) / (partial E_i^2) = 2 a_i^2 e_i (cos E_i - e_i + 2 e_i sin^2 E_i) $ <self_hess>

and $partial r_i^2 \/ partial E_j = 0$ for $i != j$.

For a hyperbolic spacecraft orbit, $r_1 = a_1 (1 - e_1 cosh H_1)$ (positive since $a_1 < 0$ and $1 - e_1 cosh H_1 < 0$):

$ (partial r_1^2) / (partial H_1) = -2 a_1^2 e_1 sinh H_1 (1 - e_1 cosh H_1) $ <self_grad_hyp>

$ (partial^2 r_1^2) / (partial H_1^2) = 2 a_1^2 e_1 (e_1 - cosh H_1 + 2 e_1 sinh^2 H_1) $ <self_hess_hyp>

=== Cross-term $bold(p_1^top) bold(M) bold(p_2)$

Since $bold(M)$ is constant, differentiation acts only on $bold(p)_1$ and $bold(p)_2$. Define the derivative vectors:

$ hat(bold(w))_i := dif bold(p)_i / (dif E_i) = vec(-sin E_i, cos E_i), quad hat(bold(u))_i := vec(cos E_i, sin E_i) $ <vecs>

For a hyperbolic spacecraft orbit:

$ hat(bold(w))_1 := dif bold(p)_1 / (dif H_1) = vec(sinh H_1, cosh H_1), quad hat(bold(u))_1 := vec(cosh H_1, sinh H_1) $ <vecs_hyp>

To differentiate $bold(p)_1^top bold(M) bold(p)_2$ with respect to $E_1$, note that only $bold(p)_1$ depends on $E_1$. By associativity, $bold(p)_1^top bold(M) bold(p)_2 = bold(p)_1^top (bold(M) bold(p)_2)$, which is a dot product of $bold(p)_1$ with the constant vector $bold(M) bold(p)_2$. The derivative of a dot product with one constant factor is:

$ (partial (bold(p)_1^top (bold(M) bold(p)_2))) / (partial E_1) = hat(bold(w))_1^top (bold(M) bold(p)_2) = hat(bold(w))_1^top bold(M) bold(p)_2 $ <cross_grad>

Similarly, for $E_2$ only $bold(p)_2$ depends on $E_2$. By associativity, $bold(p)_1^top bold(M) bold(p)_2 = (bold(p)_1^top bold(M)) bold(p)_2$, a dot product of the constant row vector $bold(p)_1^top bold(M)$ with $bold(p)_2$:

$ (partial ((bold(p)_1^top bold(M)) bold(p)_2)) / (partial E_2) = (bold(p)_1^top bold(M)) hat(bold(w))_2 = bold(p)_1^top bold(M) hat(bold(w))_2 $

The same reasoning applies to higher derivatives, replacing $bold(p)_i$ by successive derivatives $hat(bold(w))_i$, $-hat(bold(u))_i$ (elliptic) or $+hat(bold(u))_1$ (hyperbolic, since $dif^2 bold(p)_1 \/ dif H_1^2 = +hat(bold(u))_1$):

$ (partial^2 (bold(p)_1^top bold(M) bold(p)_2)) / (partial E_1^2) = -hat(bold(u))_1^top bold(M) bold(p)_2, quad (partial^2 (bold(p)_1^top bold(M) bold(p)_2)) / (partial E_2^2) = -bold(p)_1^top bold(M) hat(bold(u))_2 $ <cross_hess_diag>

For a hyperbolic spacecraft orbit, the sign of the $E_1$ term flips:

$ (partial^2 (bold(p)_1^top bold(M) bold(p)_2)) / (partial H_1^2) = +hat(bold(u))_1^top bold(M) bold(p)_2 $ <cross_hess_diag_hyp>

$ (partial^2 (bold(p)_1^top bold(M) bold(p)_2)) / (partial E_1 partial E_2) = hat(bold(w))_1^top bold(M) hat(bold(w))_2 $ <cross_hess_off>

=== Gradient

$ (partial f) / (partial E_1) = 2 a_1^2 e_1 sin E_1 (1 - e_1 cos E_1) - hat(bold(w))_1^top bold(M) bold(p)_2 $ <grad1>

$ (partial f) / (partial E_2) = 2 a_2^2 e_2 sin E_2 (1 - e_2 cos E_2) - bold(p)_1^top bold(M) hat(bold(w))_2 $ <grad2>

For a hyperbolic spacecraft orbit, @grad1 becomes (using @self_grad_hyp and the hyperbolic $hat(bold(w))_1$, $bold(p)_1$ from @vecs_hyp):

$ (partial f) / (partial H_1) = -2 a_1^2 e_1 sinh H_1 (1 - e_1 cosh H_1) - hat(bold(w))_1^top bold(M) bold(p)_2 $ <grad1_hyp>

@grad2 is unchanged (the planet is always elliptic), but $bold(p)_1$ and $bold(M)$ use the hyperbolic definitions.

=== Hessian

The Hessian $bold(H)$ is symmetric ($H_(12) = H_(21)$):

$ H_(11) = 2 a_1^2 e_1 (cos E_1 - e_1 + 2 e_1 sin^2 E_1) + hat(bold(u))_1^top bold(M) bold(p)_2 $ <H11>

$ H_(22) = 2 a_2^2 e_2 (cos E_2 - e_2 + 2 e_2 sin^2 E_2) + bold(p)_1^top bold(M) hat(bold(u))_2 $ <H22>

$ H_(12) = -hat(bold(w))_1^top bold(M) hat(bold(w))_2 $ <H12>

For a hyperbolic spacecraft orbit, $H_(11)$ changes (using @self_hess_hyp and @cross_hess_diag_hyp):

$ H_(11) = 2 a_1^2 e_1 (e_1 - cosh H_1 + 2 e_1 sinh^2 H_1) - hat(bold(u))_1^top bold(M) bold(p)_2 $ <H11_hyp>

$H_(22)$ and $H_(12)$ retain the same form, with the hyperbolic $hat(bold(w))_1$, $hat(bold(u))_1$, $bold(p)_1$.

== Solving the Newton step

The $2 times 2$ system is solved directly via Cramer's rule:

$ Delta = H_(11) H_(22) - H_(12)^2 $

$ delta E_1 = ((partial f) / (partial E_1) H_(22) - (partial f) / (partial E_2) H_(12)) / (-Delta), quad delta E_2 = ((partial f) / (partial E_2) H_(11) - (partial f) / (partial E_1) H_(12)) / (-Delta) $

All coupling terms are bilinear forms $bold(x)^top bold(M) bold(y)$ with 2D vectors, each requiring 4 multiplies and 3 adds. The five needed dot products ($hat(bold(w))_1^top bold(M) bold(p)_2$, $bold(p)_1^top bold(M) hat(bold(w))_2$, $hat(bold(u))_1^top bold(M) bold(p)_2$, $bold(p)_1^top bold(M) hat(bold(u))_2$, $hat(bold(w))_1^top bold(M) hat(bold(w))_2$) can share the intermediate products $bold(M) bold(p)_2$, $bold(M) hat(bold(w))_2$, $bold(M) hat(bold(u))_2$ (each a 2D matrix-vector multiply).

= Time Constraint via Kepler's Equation

The preceding sections treated $E_1$ and $E_2$ as independent variables. In reality, both bodies obey Kepler's equation, which couples each eccentric anomaly to a common time $t$.

== Kepler's equation

The orbital period is $T_i = 2 pi sqrt(|a_i|^3 \/ mu)$, which applies to both elliptic and hyperbolic orbits since the derivation depends only on $|a_i|$.

For an elliptic orbit $i$ with mean anomaly at epoch $M_(i,0)$:

$ M_i (t) = M_(i,0) + (2 pi) / T_i t = E_i - e_i sin E_i $ <kepler>

For a hyperbolic spacecraft orbit:

$ M_1 (t) = M_(1,0) + (2 pi) / T_1 t = e_1 sinh H_1 - H_1 $ <kepler_hyp>

At a given time $t$, both anomalies are determined:

$ E_1 - e_1 sin E_1 = M_(1,0) + (2 pi) / T_1 t, quad E_2 - e_2 sin E_2 = M_(2,0) + (2 pi) / T_2 t $ <kepler_both>

(replacing the first equation with @kepler_hyp for a hyperbolic spacecraft).

== Eliminating time

The position on an ellipse is $2 pi$-periodic in $E_i$, so each body returns to the same point after each full orbit. The set of times at which elliptic orbit $i$ passes through eccentric anomaly $E_i in [-pi, pi]$ is:

$ t = T_i / (2 pi) (E_i - e_i sin E_i - M_(i,0) + 2 pi k_i) = T_i / (2 pi) (M_i - M_(i,0) + 2 pi k_i), quad k_i in ZZ $ <time_set>

where $M_i = E_i - e_i sin E_i$ is the mean anomaly. A hyperbolic orbit is not periodic — the spacecraft passes through each $H_1$ exactly once:

$ t = T_1 / (2 pi) (e_1 sinh H_1 - H_1 - M_(1,0)) $ <time_set_hyp>

For an encounter, both bodies must be at their respective positions *simultaneously*. For two elliptic orbits, equating the time expressions:

$ T_1 / (2 pi) (M_1 - M_(1,0) + 2 pi k_1) = T_2 / (2 pi) (M_2 - M_(2,0) + 2 pi k_2) $

Simplifying and rearranging gives a family of *time-coupling constraints*, one for each integer pair $(k_1, k_2)$:

$ h(E_1, E_2; alpha) := T_1 (M_1 - M_(1,0)) - T_2 (M_2 - M_(2,0)) + 2 pi alpha = 0 $ <time_constraint>

where:

$ alpha = T_1 k_1 - T_2 k_2 $ <alpha_def>

The parameter $alpha$ has units of time: $T_1 k_1$ is the time for body 1 to complete $k_1$ orbits, $T_2 k_2$ is the time for body 2 to complete $k_2$ orbits, and $alpha$ is the mismatch. Each choice of $(k_1, k_2)$ corresponds to a different encounter opportunity (i.e.~orbit $1$ on its $k_1$-th revolution meeting orbit $2$ on its $k_2$-th revolution).

In the $(M_1, M_2)$ plane, @time_constraint defines a straight line with slope $T_1 \/ T_2$:

$ M_2 = T_1 / T_2 (M_1 - M_(1,0)) + M_(2,0) + (2 pi alpha) / T_2 $

Different values of $alpha$ shift the line parallel to itself. The full encounter problem is: find $(E_1, E_2)$ satisfying both the distance constraint $f = 0$ (@constraint_C) and $h = 0$ (@time_constraint) for some admissible $alpha$.

*Hyperbolic spacecraft.* Since $k_1 = 0$ (@time_set_hyp), the time constraint simplifies to a family parametrised by $k_2$ alone, with $alpha = -T_2 k_2$:

$ h(H_1, E_2; k_2) := T_1 (e_1 sinh H_1 - H_1 - M_(1,0)) - T_2 (E_2 - e_2 sin E_2 - M_(2,0)) - 2 pi T_2 k_2 = 0 $ <time_constraint_hyp>

#figure(
  image("encounter_contours_M.svg", width: 90%),
  caption: [Distance constraint $f = 0$ and time-coupling lines for the Voyager 2 — Jupiter encounter, plotted in the $(M_1, M_2)$ mean-anomaly domain. The black closed curve is the locus where the spacecraft--planet separation equals $r_"SOI"$; the red dashed ellipse is the second-order Taylor expansion of $f$ about its minimum. Gray lines are the family of time constraints @time_constraint, labelled by $(k_1, k_2)$. An encounter exists wherever a gray line intersects the black curve. Blue stars mark the mutual-node same-side pairings (@sec_nodal_guess): the *ascending* pair lands at $(M_1, M_2) approx (96°, 118°)$ with spacecraft--planet separation $2.0 times 10^6 "km" approx 0.04 r_"SOI"$ (inside the SOI, as expected for this encounter), while the *descending* pair at $(-2°, -52°)$ has separation $6.1 times 10^8 "km" approx 12.6 r_"SOI"$ (orbits pass far apart on the opposite node).],
) <fig_encounter_M>

= Finding the actual encounter

The geometric minimisation of @sec_newton yields $(E_1^*, E_2^*)$ where $f$ attains its minimum $f^* <= 0$ — a *potential* encounter ignoring when the bodies are actually there. The time constraint @time_constraint selects a discrete family of straight lines in $(M_1, M_2)$, one per integer pair $(k_1, k_2)$. The actual SOI entry is the earliest point (in $t$) lying on both the $f = 0$ contour and one of these lines.

== Initial guess via line--ellipse intersection <sec_line_ellipse>

Near $(E_1^*, E_2^*)$, the second-order expansion of $f$ defines an ellipse in $E$-space, $(bold(E) - bold(E)^*)^top bold(H) (bold(E) - bold(E)^*) <= -2 f^*$, where $bold(H)$ is the Hessian from @sec_grad_hess. Since the time constraint lives in $(M_1, M_2)$, transform the Hessian into the $M$-domain using the diagonal Jacobian $bold(J) = op("diag")(1 - e_1 cos E_1^*, 1 - e_2 cos E_2^*)$:

$ bold(H)_M = bold(J)^(-1) bold(H) bold(J)^(-1), quad (bold(H)_M)_(i j) = H_(i j) / ((1 - e_i cos E_i^*)(1 - e_j cos E_j^*)) $ <H_M>

The Taylor ellipse in $(M_1, M_2)$ is then $(bold(M) - bold(M)^*)^top bold(H)_M (bold(M) - bold(M)^*) <= -2 f^*$.

Intersect the time line with this ellipse — a closed-form 2×2 quadratic in a single parameter. Take the intersection point with the smaller $M_1$ (earliest in time in this encounter zone). Convert back to eccentric anomalies by inverting Kepler's equation $M_i = E_i - e_i sin E_i$ (one Newton iteration from $E_i approx M_i$ is sufficient near the minimum).

== Candidate selection on the integer lattice

Define $alpha^*$ as the value of $alpha$ that places the time line through the geometric minimum. Evaluating @time_constraint at $(E_1^*, E_2^*)$:

$ alpha^* := (T_2 (M_2^* - M_(2,0)) - T_1 (M_1^* - M_(1,0))) / (2 pi), quad M_i^* := E_i^* - e_i sin E_i^* $ <alpha_star>

Admissible $alpha$ lie on the lattice $cal(L) := {T_1 k_1 - T_2 k_2 : (k_1, k_2) in ZZ^2}$. For each integer $k_1$, the $k_2$ that minimises $|alpha - alpha^*|$ is:

$ k_2 (k_1) = op("round")((T_1 k_1 - alpha^*) / T_2). $ <k2_of_k1>

With $k_2$ determined by $k_1$ this way, the search reduces to finding the smallest $k_1$ — subject to a mission-start floor — that puts the time line inside the Taylor ellipse around $(M_1^*, M_2^*)$. The mission-start floor is

$ k_1^"min" = ceil(t_"start" \/ T_1 - (M_1^* - M_(1,0)) \/ (2 pi)), $ <k1_min>

ensuring $t_"enc" >= t_"start"$ for the returned encounter. The remaining task is to locate the smallest $k_1 >= k_1^"min"$ whose $(k_1, k_2(k_1))$ satisfies $|alpha - alpha^*| <= Delta alpha$, where $Delta alpha$ is the admissibility half-width derived next.

=== Admissibility window $Delta alpha$ <sec_W>

As $alpha$ varies, the time line slides without tilting — the slope $s := T_1 \/ T_2$ is fixed, only the intercept moves. We want the largest drift of $alpha$ from $alpha^*$ before the line loses contact with the Taylor ellipse. At that drift, the line is *tangent* to the ellipse; call it $Delta alpha$.

Work in local coordinates $Delta M_i := M_i - M_i^*$ with Hessian entries abbreviated $H^M_(i j) := (bold(H)_M)_(i j)$. The ellipse is

$ H^M_(1 1) Delta M_1^2 + 2 H^M_(1 2) Delta M_1 Delta M_2 + H^M_(2 2) Delta M_2^2 = -2 f^*, $ <ellipse_local>

and the time line is $Delta M_2 = s #h(0.1em) Delta M_1 + d_0$ with $d_0 := (2 pi \/ T_2)(alpha - alpha^*)$.

*Substitute the line into the ellipse.* Plug $Delta M_2 = s Delta M_1 + d_0$ into @ellipse_local and expand, collecting powers of $Delta M_1$:

$ A #h(0.1em) Delta M_1^2 + B #h(0.1em) Delta M_1 + C = 0, $ <quadratic_eq>

where

$ A &:= H^M_(1 1) + 2 s #h(0.1em) H^M_(1 2) + s^2 H^M_(2 2), \
  B &:= 2 d_0 (H^M_(1 2) + s #h(0.1em) H^M_(2 2)), \
  C &:= d_0^2 H^M_(2 2) + 2 f^*. $

@quadratic_eq has two real roots (line crosses ellipse at two points), one real double root (tangent, single intersection), or no real roots (line misses ellipse), depending on the sign of its discriminant.

*Tangency = zero discriminant.* The largest admissible $|d_0|$ is the value at which the line is tangent, i.e.~$B^2 - 4 A C = 0$:

$ 4 d_0^2 (H^M_(1 2) + s #h(0.1em) H^M_(2 2))^2 - 4 A (d_0^2 H^M_(2 2) + 2 f^*) = 0. $

Dividing by $4$ and isolating the $d_0^2$ coefficient:

$ d_0^2 [(H^M_(1 2) + s #h(0.1em) H^M_(2 2))^2 - A #h(0.1em) H^M_(2 2)] = 2 A f^*. $ <disc_zero>

*Simplify the bracket.* Expanding $A$:

$ & (H^M_(1 2) + s H^M_(2 2))^2 - (H^M_(1 1) + 2 s H^M_(1 2) + s^2 H^M_(2 2)) H^M_(2 2) \
& = (H^M_(1 2))^2 + 2 s H^M_(1 2) H^M_(2 2) + s^2 (H^M_(2 2))^2 - H^M_(1 1) H^M_(2 2) - 2 s H^M_(1 2) H^M_(2 2) - s^2 (H^M_(2 2))^2 \
& = (H^M_(1 2))^2 - H^M_(1 1) H^M_(2 2) = -det(bold(H)_M). $

The $s$-linear and $s$-quadratic terms cancel pairwise, leaving just $-det(bold(H)_M)$, which is negative since $bold(H)_M$ is positive definite.

*Solve for $d_0^2$.* Substituting the simplified bracket into @disc_zero:

$ -d_0^2 det(bold(H)_M) = 2 A f^* quad arrow.r.double quad d_0^2 = -2 A f^* \/ det(bold(H)_M). $

Both sides are positive: $f^* < 0$ at a real encounter minimum, $det(bold(H)_M) > 0$, and $A > 0$ (quadratic form of $bold(H)_M$ evaluated along direction $(1, s)$).

*Convert back to $alpha$.* Using $d_0 = (2 pi \/ T_2)(alpha - alpha^*)$:

$ Delta alpha = (T_2 \/ (2 pi)) sqrt(-2 A f^* \/ det(bold(H)_M)), quad A = H^M_(1 1) + 2 s #h(0.1em) H^M_(1 2) + s^2 H^M_(2 2). $ <delta_alpha>

*Interpretation.* $alpha = alpha^*$: line through the minimum. $|alpha - alpha^*| = Delta alpha$: line tangent to the ellipse. $|alpha - alpha^*| > Delta alpha$: line misses it entirely, no SOI entry on this revolution pair. The admissibility band is shaded in @fig_parker_venus.

#figure(
  image("encounter_contours_parker_venus.svg", width: 90%),
  caption: [Parker-like spacecraft vs.~Venus in the $(M_1, M_2)$ mean-anomaly domain, cropped around the geometric minimum. The red dashed Taylor ellipse and the black $f = 0$ contour overlap to within line thickness. The green dotted line is $alpha = alpha^*$ (passing through the minimum); the dashed green lines at $alpha = alpha^* plus.minus Delta alpha$ are tangent to the Taylor ellipse and bound the admissibility band (shaded) derived in @sec_W. Here $Delta alpha = 0.372$ days, very narrow relative to the gray-line spacing, so the admissible $(k_1, k_2)$ is far from the nearest lattice line and is found by a continued-fraction search — the blue line is the first admissible pair, $(k_1, k_2) = (91, 41)$.],
) <fig_parker_venus>

=== Continued-fraction search for admissible $k_1$ <sec_cf_search>

Given $alpha^*$, $Delta alpha$, and the mission-start floor $k_1^"min"$ from @k1_min, the remaining task is: find the smallest integer $k_1 >= k_1^"min"$ such that @k2_of_k1 produces a lattice point $alpha := T_1 k_1 - T_2 k_2$ inside $[alpha^* - Delta alpha, #h(0.1em) alpha^* + Delta alpha]$. Define the residual

$ rho(k_1) := T_1 k_1 - T_2 dot op("round")((T_1 k_1 - alpha^*) \/ T_2) - alpha^* = alpha - alpha^*. $ <rho_def>

By construction $rho(k_1) in (-T_2\/2, #h(0.2em) T_2\/2]$, and $k_1$ is admissible iff $|rho(k_1)| <= Delta alpha$. When $Delta alpha >= T_2\/2$ every $k_1$ is admissible and the answer is simply $k_1^"min"$. The interesting case is $Delta alpha << T_2\/2$, where admissible $k_1$ are sparse and must be located directly — without enumerating non-admissible values.

==== Key observation: CF convergents as calibrated jumps

As $k_1$ increases by $1$, $rho$ advances by a single large step on the circle $RR \/ T_2 ZZ$. Stepping $k_1$ by the denominator $q_n$ of the $n$-th CF convergent $(p_n, q_n)$ of $tau := T_1 \/ T_2$ instead advances $rho$ by
$ eta_n dot T_2 := q_n T_1 - p_n T_2, $
which is *small* (alternating sign with $n$, shrinking roughly geometrically). Each convergent therefore gives a different calibrated step size in $rho$-space.

Any non-negative integer $Delta k$ has a unique *Ostrowski representation*

$ Delta k = sum_(i >= 0) c_i q_i, quad 0 <= c_(i+1) <= a_(i+1), $ <ostrowski>

where the $a_i$ are the partial quotients of $tau = [a_0; a_1, a_2, dots]$. Stepping $k_1$ by $Delta k$ shifts $rho$ by $sum_i c_i dot eta_i T_2$. Choosing the $c_i$ from large $i$ (tiny shifts) down to small $i$ (coarse shifts), we construct the smallest $Delta k$ that places $rho$ inside the admissibility band in $cal(O)(log(T_2 \/ Delta alpha))$ steps.

==== Algorithm ($cal(O)(log)$)

Following Visser (2023) §§3.4 and 6.1, adapted to our notation:

1. Expand $tau = T_1 \/ T_2$ as a continued fraction,
   $ tau = [a_0; a_1, a_2, dots], $
   and generate convergents $(p_n, q_n)$ by the recursion
   $ p_(-1) = 1, #h(0.5em) p_0 = a_0, #h(0.5em) p_(n+1) = a_(n+1) p_n + p_(n-1), quad q_(-1) = 0, #h(0.5em) q_0 = 1, #h(0.5em) q_(n+1) = a_(n+1) q_n + q_(n-1). $
   Continue until $|eta_n| dot T_2 <= Delta alpha$; call this depth $L$. Because $q_n$ grows at least at the Fibonacci rate, $L = cal(O)(log(T_2 \/ Delta alpha))$.

2. At each CF level $n = L, L-1, dots, 1$, test a *constant* number of integer candidates (typically 1–3) in the admissibility trapezoid at that level. The admissibility trapezoid is bounded by $((1 - delta) \/ q_n, #h(0.3em) (1 + delta) \/ q_n)$ along one basis direction, with the conjugate coordinate fixed by the ceiling rule of @k2_of_k1. Each candidate is one integer multiplication plus a comparison.

3. Whenever a candidate at level $n$ satisfies $|rho| <= Delta alpha$ *and* the corresponding $k_1 >= k_1^"min"$, translate it back to the standard basis,
   $ k_1 = x dot k_(2n) - y dot k_(2n+1), $
   with $(k_n)$ satisfying the same recursion as $(q_n)$ but seeded by $(k_0, k_1) = (1, 0)$. Return this $k_1$.

4. If no candidate at any level satisfies both constraints, $tau$ is rational (mean-motion resonance — the encounter does not recur at integer $(k_1, k_2)$) or the encounter is a geometric accident that cannot be converted into a real SOI crossing. Return failure.

Total cost: $cal(O)(L) = cal(O)(log(T_2 \/ Delta alpha))$ integer operations. The returned $k_1$ is the smallest admissible value.

Once $k_1$ is returned, the companion $k_2$ follows from @k2_of_k1:

$ k_2 = op("round")((T_1 k_1 - alpha^*) \/ T_2). $

==== Worked example: Parker-like / Venus

With $tau = T_1 \/ T_2 = 0.447208$, the continued-fraction expansion is $tau = [0; 2, 4, 4, 4, 12, 3, 1, dots]$. The first few convergents and their errors are:

#figure(
  table(
    columns: (auto, auto, auto, auto),
    align: (right, right, right, right),
    table.header[*$n$*][*$p_n \/ q_n$*][*$|eta_n| dot T_2$ (s)*][*within $Delta alpha$?*],
    [1], [$1\/2$],   [$2.05 times 10^6$], [no],
    [2], [$4\/9$],   [$4.83 times 10^5$], [no],
    [3], [$17\/38$], [$1.18 times 10^5$], [no],
    [4], [$72\/161$],[$9.64 times 10^3$], [*yes* — first time $<= Delta alpha = 3.21 times 10^4$ s],
  ),
) <tab_cf>

The Ostrowski construction terminates at depth $L = 4$, yielding $(k_1, k_2) = (91, 40)$ with $rho(91) = -3.17 times 10^4$ s $in [-Delta alpha, Delta alpha]$. The corresponding time line (blue in @fig_parker_venus) crosses the Taylor ellipse, and the Newton refinement of @sec_line_ellipse converges to the SOI entry point at $t_"enc" approx 25.1$ years from epoch.

==== Worked example: Voyager 2 / Jupiter

With $tau = 0.587689$, the CF expansion is $tau = [0; 1, 1, 2, 2, 1, 5, dots]$. Here $Delta alpha = 59.06$ days is $2.7 %$ of $T_2 \/ 2$ — wide enough that the admissibility condition $|rho(k_1^"min")| <= Delta alpha$ is already satisfied at $k_1 = 0$. No Ostrowski expansion is needed: the algorithm returns $(k_1, k_2) = (0, 0)$ in one step, and the encounter occurs within the first revolution.

==== Refinement and final encounter time

After $(k_1, k_2)$ is chosen, the specific time line
$ M_2 = (T_1 \/ T_2) M_1 + c_"line", quad c_"line" = -(T_1 \/ T_2) M_(1,0) + M_(2,0) + 2 pi alpha \/ T_2 $
is intersected with the Taylor ellipse (cf.~@sec_line_ellipse) to obtain a seed $E_1$ at the SOI entry side of the ellipse, and a 1D Newton iteration along the line converges to the exact $f = 0$ crossing. The encounter time is then recovered via Kepler's equation applied to body 1:

$ t_"enc" = (T_1 \/ (2 pi)) (M_(1,"enc") + 2 pi k_1 - M_(1,0)), quad M_(1,"enc") = E_1 - e_1 sin E_1. $ <t_enc_final>

Equivalently via body 2 through the time-coupling constraint @time_constraint; agreement to floating-point precision between the two is a useful invariant test.
