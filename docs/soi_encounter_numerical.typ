#set document(title: "SOI Encounter Derivation")
#set page(margin: 1cm, numbering: "1")
#set text(size: 11pt)
#set heading(numbering: "1.")
#set math.equation(numbering: "(1)")

= Analytical Derivation of Sphere-of-Influence Encounter Points

== Problem Statement

Given two Keplerian orbits (spacecraft orbit 1, planet orbit 2) in heliocentric ecliptic coordinates, find the anomaly parameters such that the distance between the spacecraft and the planet equals the planet's sphere-of-influence radius $r_"SOI"$. The spacecraft orbit may be elliptic ($e_1 < 1$) or hyperbolic ($e_1 > 1$); the planet orbit is always elliptic.

$ norm(bold(A) bold(r)_1 (E_1) - bold(B) bold(r)_2 (E_2)) = r_"SOI" $

where $bold(A)$ and $bold(B)$ are known constant $3 times 2$ rotation matrices mapping from orbital-plane 2D coordinates to 3D heliocentric ecliptic coordinates.

== Notation

- $a_i$, $b_i$, $e_i$: semi-major axis, semi-minor axis, and eccentricity of orbit $i$ (with $b_i = |a_i| sqrt(|1 - e_i^2|)$)
  - Elliptic ($e_i < 1$): $a_i > 0$
  - Hyperbolic ($e_1 > 1$, spacecraft only): $a_1 < 0$
- $E_i$: eccentric anomaly (elliptic orbit)
- $H_1$: hyperbolic eccentric anomaly (hyperbolic spacecraft orbit)
- $bold(A) in RR^(3 times 2)$, $bold(B) in RR^(3 times 2)$: orbital-to-ecliptic rotation matrices with components $A_(j k)$, $B_(j k)$
- $r_"SOI"$: radius of the planet's sphere of influence

== Position in Orbital Plane

For an elliptic orbit, the 2D position vector in the orbital plane as a function of eccentric anomaly is:

$ bold(r)_i (E_i) = vec(delim: "[",a_i (cos E_i - e_i), b_i sin E_i) $ <orbital_pos>

For a hyperbolic spacecraft orbit:

$ bold(r)_1 (H_1) = vec(delim: "[",a_1 (cosh H_1 - e_1), b_1 sinh H_1) $ <orbital_pos_hyp>

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

For a hyperbolic spacecraft orbit, $r_1 = a_1 (1 - e_1 cosh H_1)$ (positive since $a_1 < 0$ and $1 - e_1 cosh H_1 < 0$):

$ (partial r_1^2) / (partial H_1) = -2 a_1^2 e_1 sinh H_1 (1 - e_1 cosh H_1) $ <self_grad_hyp>

$ (partial^2 r_1^2) / (partial H_1^2) = 2 a_1^2 e_1 (e_1 - cosh H_1 + 2 e_1 sinh^2 H_1) $ <self_hess_hyp>

==== Cross-term $bold(r)_1^top bold(C) bold(r)_2$

The position vector @orbital_pos can be factored as $bold(r)_i = op("diag")(a_i, b_i) bold(p)_i$, where $op("diag")(a_i, b_i) := mat(a_i, 0; 0, b_i)$ is a diagonal matrix and $bold(p)_i$ is a dimensionless position-direction vector:

$ bold(p)_i = vec(delim: "[",cos E_i - e_i, sin E_i) $ <p_def>

For a hyperbolic spacecraft orbit: $bold(p)_1 = (cosh H_1 - e_1, sinh H_1)^top$.

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

$ hat(bold(w))_i := dif bold(p)_i / (dif E_i) = vec(delim: "[",-sin E_i, cos E_i), quad hat(bold(u))_i := vec(delim: "[",cos E_i, sin E_i) $ <vecs>

For a hyperbolic spacecraft orbit:

$ hat(bold(w))_1 := dif bold(p)_1 / (dif H_1) = vec(delim: "[",sinh H_1, cosh H_1), quad hat(bold(u))_1 := vec(delim: "[",cosh H_1, sinh H_1) $ <vecs_hyp>

To differentiate $bold(p)_1^top bold(M) bold(p)_2$ with respect to $E_1$, note that only $bold(p)_1$ depends on $E_1$. By associativity, $bold(p)_1^top bold(M) bold(p)_2 = bold(p)_1^top (bold(M) bold(p)_2)$, which is a dot product of $bold(p)_1$ with the constant vector $bold(M) bold(p)_2$. The derivative of a dot product with one constant factor is:

$ (partial (bold(p)_1^top (bold(M) bold(p)_2))) / (partial E_1) = hat(bold(w))_1^top (bold(M) bold(p)_2) = hat(bold(w))_1^top bold(M) bold(p)_2 $ <cross_grad>

Similarly, for $E_2$ only $bold(p)_2$ depends on $E_2$. By associativity, $bold(p)_1^top bold(M) bold(p)_2 = (bold(p)_1^top bold(M)) bold(p)_2$, a dot product of the constant row vector $bold(p)_1^top bold(M)$ with $bold(p)_2$:

$ (partial ((bold(p)_1^top bold(M)) bold(p)_2)) / (partial E_2) = (bold(p)_1^top bold(M)) hat(bold(w))_2 = bold(p)_1^top bold(M) hat(bold(w))_2 $

The same reasoning applies to higher derivatives, replacing $bold(p)_i$ by successive derivatives $hat(bold(w))_i$, $-hat(bold(u))_i$ (elliptic) or $+hat(bold(u))_1$ (hyperbolic, since $dif^2 bold(p)_1 \/ dif H_1^2 = +hat(bold(u))_1$):

$ (partial^2 (bold(p)_1^top bold(M) bold(p)_2)) / (partial E_1^2) = -hat(bold(u))_1^top bold(M) bold(p)_2, quad (partial^2 (bold(p)_1^top bold(M) bold(p)_2)) / (partial E_2^2) = -bold(p)_1^top bold(M) hat(bold(u))_2 $ <cross_hess_diag>

For a hyperbolic spacecraft orbit, the sign of the $E_1$ term flips:

$ (partial^2 (bold(p)_1^top bold(M) bold(p)_2)) / (partial H_1^2) = +hat(bold(u))_1^top bold(M) bold(p)_2 $ <cross_hess_diag_hyp>

$ (partial^2 (bold(p)_1^top bold(M) bold(p)_2)) / (partial E_1 partial E_2) = hat(bold(w))_1^top bold(M) hat(bold(w))_2 $ <cross_hess_off>

==== Combined expressions

Combining $f = r_1^2 + r_2^2 - bold(p)_1^top bold(M) bold(p)_2 - r_"SOI"^2$ using @self_grad with @cross_grad, and @self_hess with @cross_hess_diag and @cross_hess_off:

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

=== Solving the Newton step

The $2 times 2$ system is solved directly via Cramer's rule:

$ Delta = H_(11) H_(22) - H_(12)^2 $

$ delta E_1 = ((partial f) / (partial E_1) H_(22) - (partial f) / (partial E_2) H_(12)) / (-Delta), quad delta E_2 = ((partial f) / (partial E_2) H_(11) - (partial f) / (partial E_1) H_(12)) / (-Delta) $

All coupling terms are bilinear forms $bold(x)^top bold(M) bold(y)$ with 2D vectors, each requiring 4 multiplies and 3 adds. The five needed dot products ($hat(bold(w))_1^top bold(M) bold(p)_2$, $bold(p)_1^top bold(M) hat(bold(w))_2$, $hat(bold(u))_1^top bold(M) bold(p)_2$, $bold(p)_1^top bold(M) hat(bold(u))_2$, $hat(bold(w))_1^top bold(M) hat(bold(w))_2$) can share the intermediate products $bold(M) bold(p)_2$, $bold(M) hat(bold(w))_2$, $bold(M) hat(bold(u))_2$ (each a 2D matrix-vector multiply).

== Coarse Encounter Interval

Before running Newton's method, we restrict the search to anomaly values where the spacecraft's heliocentric distance overlaps with the planet's distance range $plus.minus r_"SOI"$.

=== Radial overlap condition

An encounter requires the spacecraft distance to lie within:

$ a_2 (1 - e_2) - r_"SOI" <= r_1 <= a_2 (1 + e_2) + r_"SOI" $

*Elliptic spacecraft.* Using $r_1 = a_1 (1 - e_1 cos E_1)$ and solving for $cos E_1$:

$ (a_1 - a_2 (1 + e_2) - r_"SOI") / (a_1 e_1) <= cos E_1 <= (a_1 - a_2 (1 - e_2) + r_"SOI") / (a_1 e_1) $ <coarse_bounds>

Clamping both sides to $[-1, 1]$ and applying $arccos$ (which reverses the inequality) gives two symmetric intervals $[E_"lo", E_"hi"]$ and $[-E_"lo", -E_"hi"]$ in $E_1$. If no valid interval exists (i.e.~the clamped range is empty), no encounter is possible at any $E_1$.

*Hyperbolic spacecraft.* Using $r_1 = a_1 (1 - e_1 cosh H_1)$ and solving for $cosh H_1$ (noting $a_1 e_1 < 0$, so dividing reverses the inequality):

$ (a_1 - a_2 (1 - e_2) + r_"SOI") / (a_1 e_1) <= cosh H_1 <= (a_1 - a_2 (1 + e_2) - r_"SOI") / (a_1 e_1) $ <coarse_bounds_hyp>

Clamping the lower bound to $max(dots, 1)$ (since $cosh H_1 >= 1$) and applying $op("arcosh")$ gives a single symmetric interval $[-H_"hi", -H_"lo"] union [H_"lo", H_"hi"]$. If the clamped range is empty, no encounter is possible.

=== Initial guess via parabolic interpolation

For each of the two symmetric $E_1$ intervals, we evaluate $f$ at three points: the endpoints and midpoint. Here $E_2$ is estimated by projecting the spacecraft's 3D position onto the planet's orbital plane and inverting the ellipse parametrisation:

$ E_2 = op("atan2")(q_y \/ b_2, quad q_x \/ a_2 + e_2) $

where $bold(q) = bold(B)^top bold(A) bold(r)_1$ is the projection. This matches the eccentric anomaly that $bold(q)$ would have if it lay on the planet's ellipse. The projection is exact when the orbital planes coincide; for inclined orbits, the out-of-plane component is lost. Numerical evaluation for a Pluto-like target ($i = 17°$, $e_2 = 0.25$) shows the maximum $E_2$ error is under $2°$ for moderate spacecraft eccentricities ($e_1 < 0.5$), but a direct Earth--Pluto near-Hohmann transfer ($e_1 approx 0.95$) reaches $~9°$ at favourable orientations and up to $~63°$ at unfavourable ascending-node orientations. Despite these large worst-case errors, the estimate remains adequate as an initial guess for Newton's method, which converges from any starting point in the correct half of the encounter region.

Given three samples $(x_0, f_0)$, $(x_1, f_1)$, $(x_2, f_2)$, the minimiser of the interpolating quadratic is:

$ E_1^* = x_1 - 1/2 ((x_1 - x_0)^2 (f_1 - f_2) - (x_1 - x_2)^2 (f_1 - f_0)) / ((x_1 - x_0)(f_1 - f_2) - (x_1 - x_2)(f_1 - f_0)) $ <parabolic>

Newton's method is then run from the best initial guess across both branches. If the first branch fails to converge to $f <= r_"SOI"^2$, the second branch is tried.

== Encounter Intervals in $E_1$ and $E_2$

The constraint $f(E_1, E_2) = 0$ defines an implicit curve (or set of curves) in the $(E_1, E_2)$ plane. The *encounter interval* $[E_(1,min), E_(1,max)]$ is the projection of this curve onto the $E_1$ axis, and $[E_(2,min), E_(2,max)]$ is the projection onto the $E_2$ axis. For every $E_1$ in the first interval there exists at least one $E_2$ in the second interval such that the distance equals $r_"SOI"$.

#import "@preview/cetz:0.3.4"

#figure(
  cetz.canvas(length: 2cm, {
    import cetz.draw: *

    let w = 7
    let h = 5

    // axes
    line((0, 0), (w + 0.6, 0), mark: (end: "stealth", fill: black, scale: 0.5))
    line((0, 0), (0, h + 0.6), mark: (end: "stealth", fill: black, scale: 0.5))
    content((w + 0.6, -0.35), $E_1$)
    content((-0.35, h + 0.6), $E_2$)

    // constraint curve (tilted ellipse)
    let cx = w / 2
    let cy = h / 2
    let a = 2.4
    let b = 1.4
    let theta = 30deg

    // points on tilted ellipse: parametric angle t
    let ellipse-pt(t) = {
      let ct = calc.cos(t)
      let st = calc.sin(t)
      let x = a * ct * calc.cos(theta) - b * st * calc.sin(theta) + cx
      let y = a * ct * calc.sin(theta) + b * st * calc.cos(theta) + cy
      (x, y)
    }

    // find extremal points analytically
    // dx/dt = 0: tan(t) = -b sin(theta) / (a cos(theta)) → vertical tangent (E1 extrema)
    // calc.atan2(x, y) in Typst returns atan(y/x)
    let tv = calc.atan2(a * calc.cos(theta), -b * calc.sin(theta))
    // dy/dt = 0: tan(t) = -b cos(theta) / (a sin(theta)) → horizontal tangent (E2 extrema)
    let th = calc.atan2(a * calc.sin(theta), b * calc.cos(theta))

    let e1-max-pt = ellipse-pt(tv)
    let e1-min-pt = ellipse-pt(tv + 180deg)
    let e2-max-pt = ellipse-pt(th)
    let e2-min-pt = ellipse-pt(th + 180deg)

    // draw the ellipse curve
    let npts = 80
    let pts = range(npts + 1).map(i => {
      let t = i / npts * 360deg
      ellipse-pt(t)
    })
    line(..pts, close: true, stroke: 1.2pt + black)

    // projection lines and labels for E1 bounds
    let dash-style = (dash: "dashed", paint: gray)

    // E1_min
    line(e1-min-pt, (e1-min-pt.at(0), 0), stroke: dash-style)
    content((e1-min-pt.at(0), -0.4), $E_(1,min)$, anchor: "north")

    // E1_max
    line(e1-max-pt, (e1-max-pt.at(0), 0), stroke: dash-style)
    content((e1-max-pt.at(0), -0.4), $E_(1,max)$, anchor: "north")

    // E2_min
    line(e2-min-pt, (0, e2-min-pt.at(1)), stroke: dash-style)
    content((-0.4, e2-min-pt.at(1)), $E_(2,min)$, anchor: "east")

    // E2_max
    line(e2-max-pt, (0, e2-max-pt.at(1)), stroke: dash-style)
    content((-0.4, e2-max-pt.at(1)), $E_(2,max)$, anchor: "east")

    // interval brackets on axes
    let bracket-color = blue
    line(
      (e1-min-pt.at(0), -0.05), (e1-max-pt.at(0), -0.05),
      stroke: 2pt + bracket-color,
    )
    line(
      (-0.05, e2-min-pt.at(1)), (-0.05, e2-max-pt.at(1)),
      stroke: 2pt + bracket-color,
    )

    // critical points
    let dot-radius = 0.08

    // vertical tangent points (∂f/∂E₂ = 0) → E1 extrema
    circle(e1-min-pt, radius: dot-radius, fill: red, stroke: none)
    circle(e1-max-pt, radius: dot-radius, fill: red, stroke: none)

    // horizontal tangent points (∂f/∂E₁ = 0) → E2 extrema
    circle(e2-min-pt, radius: dot-radius, fill: eastern, stroke: none)
    circle(e2-max-pt, radius: dot-radius, fill: eastern, stroke: none)

    // tangent lines at critical points
    let tang-len = 1.0

    // vertical tangents at E1 extrema
    line(
      (e1-min-pt.at(0), e1-min-pt.at(1) - tang-len),
      (e1-min-pt.at(0), e1-min-pt.at(1) + tang-len),
      stroke: (dash: "dotted", paint: red),
    )
    line(
      (e1-max-pt.at(0), e1-max-pt.at(1) - tang-len),
      (e1-max-pt.at(0), e1-max-pt.at(1) + tang-len),
      stroke: (dash: "dotted", paint: red),
    )

    // horizontal tangents at E2 extrema
    line(
      (e2-min-pt.at(0) - tang-len, e2-min-pt.at(1)),
      (e2-min-pt.at(0) + tang-len, e2-min-pt.at(1)),
      stroke: (dash: "dotted", paint: eastern),
    )
    line(
      (e2-max-pt.at(0) - tang-len, e2-max-pt.at(1)),
      (e2-max-pt.at(0) + tang-len, e2-max-pt.at(1)),
      stroke: (dash: "dotted", paint: eastern),
    )

    // curve label
    content((cx + a * 0.5 + 0.5, cy + b + 0.4), $f(E_1, E_2) = 0$)

    // legend
    let lx = w - 1.5
    let ly = 0.9
    circle((lx, ly), radius: dot-radius, fill: red, stroke: none)
    content((lx + 0.15, ly), $partial f \/ partial E_2 = 0$, anchor: "west")
    circle((lx, ly - 0.5), radius: dot-radius, fill: eastern, stroke: none)
    content((lx + 0.15, ly - 0.5), $partial f \/ partial E_1 = 0$, anchor: "west")
  }),
  caption: [
    The constraint curve $f = 0$ in the $(E_1, E_2)$ plane. Red dots mark vertical tangents ($partial f \/ partial E_2 = 0$), whose $E_1$ coordinates give $E_(1,min)$ and $E_(1,max)$. Teal dots mark horizontal tangents ($partial f \/ partial E_1 = 0$), whose $E_2$ coordinates give $E_(2,min)$ and $E_(2,max)$. Blue bars on the axes show the projected encounter intervals.
  ],
) <encounter_intervals>

#figure(
  placement: none,
  image("encounter_surface_3d.png", width: 100%),
  caption: [
    The surface $z = f(E_1, E_2) \/ r_"SOI"^2$ over the encounter region (Voyager 2 — Jupiter). The black curve is the $f = 0$ contour where the surface crosses the $z = 0$ plane. The black dot at $(E_1, E_2) approx (128.9°, 121.1°)$ marks the surface minimum (closest approach without accounting for the planet's gravity). Red dots mark $partial f \/ partial E_2 = 0$ at $E_1 approx 120.2°$ and $139.0°$ ($E_1$ extrema); green dots mark $partial f \/ partial E_1 = 0$ at $E_2 approx 115.3°$ and $126.9°$ ($E_2$ extrema).
  ],
) <encounter_surface>

=== Extrema via Lagrange multipliers

Finding the extreme values of $E_2$ subject to $f(E_1, E_2) = 0$ is a constrained optimisation problem. The Lagrangian is:

$ cal(L) = E_2 - lambda f(E_1, E_2) $

Setting partial derivatives to zero:

$ (partial cal(L)) / (partial E_1) = -lambda (partial f) / (partial E_1) = 0, quad (partial cal(L)) / (partial E_2) = 1 - lambda (partial f) / (partial E_2) = 0 $

The second equation gives $lambda = 1 \/ (partial f \/ partial E_2)$, which is necessarily nonzero. Substituting into the first:

$ -1 / (partial f \/ partial E_2) dot (partial f) / (partial E_1) = 0 $

Since the prefactor is nonzero, this requires:

$ (partial f) / (partial E_1) = 0 $ <E2_crit>

By the same argument with $cal(L) = E_1 - lambda f$, the extrema of $E_1$ require:

$ (partial f) / (partial E_2) = 0 $ <E1_crit>

Geometrically, consider the surface $z = f(E_1, E_2)$ over the $(E_1, E_2)$ plane (@encounter_surface). The constraint curve $f = 0$ is the intersection of this surface with the $z = 0$ plane. The $E_2$ extrema of this intersection occur where the curve runs parallel to the $E_1$ axis — at these points, the surface's gradient in the $E_1$ direction is zero along the constraint, giving $partial f \/ partial E_1 = 0$.

=== System for $E_1$ bounds

To find $E_(1,min)$ and $E_(1,max)$, solve the $2 times 2$ system:

$ cases(
  f(E_1, E_2) = 0,
  (partial f) / (partial E_2) = 0
) $ <E1_system>

Using @grad2, the second equation is:

$ 2 a_2^2 e_2 sin E_2 (1 - e_2 cos E_2) - bold(p)_1^top bold(M) hat(bold(w))_2 = 0 $

Newton's method for this system uses the Jacobian:

$ bold(J)_1 = mat(
  (partial f) / (partial E_1), (partial f) / (partial E_2);
  (partial^2 f) / (partial E_1 partial E_2), (partial^2 f) / (partial E_2^2)
) = mat(
  g_1, g_2;
  H_(12), H_(22)
) $ <J1>

where $g_1 := partial f \/ partial E_1$ (@grad1), $g_2 := partial f \/ partial E_2$ (@grad2), and $H_(12)$, $H_(22)$ are the Hessian entries (@H12, @H22). The Newton step $bold(delta) = (delta E_1, delta E_2)^top$ solves $bold(J)_1 bold(delta) = -(f, g_2)^top$ via Cramer's rule:

$ Delta_1 = g_1 H_(22) - g_2 H_(12) $

$ delta E_1 = (f H_(22) - g_2^2) / Delta_1, quad delta E_2 = (f H_(12) - g_1 g_2) / Delta_1 $ <E1_step>

Note that at convergence ($g_2 = 0$), the determinant simplifies to $Delta_1 = g_1 H_(22)$.

=== System for $E_2$ bounds

To find $E_(2,min)$ and $E_(2,max)$, solve:

$ cases(
  f(E_1, E_2) = 0,
  (partial f) / (partial E_1) = 0
) $ <E2_system>

Using @grad1, the second equation is:

$ 2 a_1^2 e_1 sin E_1 (1 - e_1 cos E_1) - hat(bold(w))_1^top bold(M) bold(p)_2 = 0 $

The Jacobian is:

$ bold(J)_2 = mat(
  (partial f) / (partial E_1), (partial f) / (partial E_2);
  (partial^2 f) / (partial E_1^2), (partial^2 f) / (partial E_1 partial E_2)
) = mat(
  g_1, g_2;
  H_(11), H_(12)
) $ <J2>

The Newton step solves $bold(J)_2 bold(delta) = -(f, g_1)^top$:

$ Delta_2 = g_1 H_(12) - g_2 H_(11) $

$ delta E_1 = (f H_(12) - g_1 g_2) / Delta_2, quad delta E_2 = (g_1^2 - f H_(11)) / Delta_2 $ <E2_step>

At convergence ($g_1 = 0$), the determinant simplifies to $Delta_2 = -g_2 H_(11)$.

= Time Constraint via Kepler's Equation

The preceding sections treated $E_1$ and $E_2$ as independent variables. In reality, both bodies obey Kepler's equation, which couples each eccentric anomaly to a common time $t$.

== Kepler's equation

The mean motion $n_i = sqrt(mu \/ |a_i|^3)$ applies to both elliptic and hyperbolic orbits, since the derivation from the vis-viva equation depends only on $|a_i|$.

For an elliptic orbit $i$ with mean anomaly at epoch $M_(i,0)$:

$ M_i (t) = M_(i,0) + n_i t = E_i - e_i sin E_i $ <kepler>

For a hyperbolic spacecraft orbit:

$ M_1 (t) = M_(1,0) + n_1 t = e_1 sinh H_1 - H_1 $ <kepler_hyp>

At a given time $t$, both anomalies are determined:

$ E_1 - e_1 sin E_1 = M_(1,0) + n_1 t, quad E_2 - e_2 sin E_2 = M_(2,0) + n_2 t $ <kepler_both>

(replacing the first equation with @kepler_hyp for a hyperbolic spacecraft).

== Eliminating time

The position on an ellipse is $2 pi$-periodic in $E_i$, so each body returns to the same point after each full orbit. The set of times at which elliptic orbit $i$ passes through eccentric anomaly $E_i in [-pi, pi]$ is:

$ t = (E_i - e_i sin E_i - M_(i,0) + 2 pi k_i) / n_i, quad k_i in ZZ $ <time_set>

A hyperbolic orbit is not periodic — the spacecraft passes through each $H_1$ exactly once:

$ t = (e_1 sinh H_1 - H_1 - M_(1,0)) / n_1 $ <time_set_hyp>

For an encounter, both bodies must be at their respective positions *simultaneously*. For two elliptic orbits, equating the time expressions:

$ (E_1 - e_1 sin E_1 - M_(1,0) + 2 pi k_1) / n_1 = (E_2 - e_2 sin E_2 - M_(2,0) + 2 pi k_2) / n_2 $

Rearranging gives a family of *time-coupling constraints*, one for each integer pair $(k_1, k_2)$:

$ h_(k_1,k_2) (E_1, E_2) := n_2 (E_1 - e_1 sin E_1 - M_(1,0)) - n_1 (E_2 - e_2 sin E_2 - M_(2,0)) + 2 pi (n_2 k_1 - n_1 k_2) = 0 $ <time_constraint>

Each choice of $(k_1, k_2)$ corresponds to a different encounter opportunity (i.e.~orbit $1$ on its $k_1$-th revolution meeting orbit $2$ on its $k_2$-th revolution). Since only the combination $n_2 k_1 - n_1 k_2$ appears, the distinct constraints are parametrised by a single offset:

$ h(E_1, E_2; alpha) := n_2 (E_1 - e_1 sin E_1 - M_(1,0)) - n_1 (E_2 - e_2 sin E_2 - M_(2,0)) + 2 pi alpha = 0 $ <time_constraint_alpha>

where $alpha = n_2 k_1 - n_1 k_2$ ranges over a discrete set. In the $(E_1, E_2)$ plane (with $E_i in [-pi, pi]$), each value of $alpha$ gives a monotone curve from bottom-left to top-right, and consecutive curves are spaced apart by one spacecraft orbital period $T_1 = 2 pi \/ n_1$. The full encounter problem is: find $(E_1, E_2)$ satisfying both the distance constraint $f = 0$ (@constraint_C) and $h = 0$ (@time_constraint_alpha) for some admissible $alpha$.

*Hyperbolic spacecraft.* Since $k_1 = 0$ (@time_set_hyp), the time constraint simplifies to a family parametrised by $k_2$ alone. Replacing the spacecraft's mean anomaly expression:

$ h(H_1, E_2; k_2) := n_2 (e_1 sinh H_1 - H_1 - M_(1,0)) - n_1 (E_2 - e_2 sin E_2 - M_(2,0)) - 2 pi n_1 k_2 = 0 $ <time_constraint_hyp>

== Finding the next encounter after $t_0$

After each gravity assist, the spacecraft's orbit changes, so it only makes sense to find the *next* encounter from a given epoch $t_0$ rather than enumerating all future encounters on a fixed orbit.

The distance constraint can produce up to two separate encounter zones (corresponding to the two symmetric coarse intervals from @coarse_bounds). Each zone $s in {1, 2}$ has its own encounter intervals $[E_(1,min)^((s)), E_(1,max)^((s))]$ and $[E_(2,min)^((s)), E_(2,max)^((s))]$ from @encounter_intervals. Since the mean anomaly $M_i = E_i - e_i sin E_i$ is monotone in $E_i$, each zone corresponds to mean-anomaly intervals:

$ M_(i,min)^((s)) = E_(i,min)^((s)) - e_i sin E_(i,min)^((s)), quad M_(i,max)^((s)) = E_(i,max)^((s)) - e_i sin E_(i,max)^((s)) $ <M_bounds>

For a hyperbolic spacecraft orbit: $M_(1,min)^((s)) = e_1 sinh H_(1,min)^((s)) - H_(1,min)^((s))$ (and likewise for $M_(1,max)^((s))$).

On its $k_i$-th elliptic orbit, body $i$ passes through encounter zone $s$ during the time interval (@time_set):

$ t in [(M_(i,min)^((s)) - M_(i,0) + 2 pi k_i) / n_i, quad (M_(i,max)^((s)) - M_(i,0) + 2 pi k_i) / n_i] $ <time_interval>

A hyperbolic spacecraft passes through each encounter zone at most once. Setting $k_1 = 0$ in the above (with the hyperbolic mean anomaly from @time_set_hyp):

$ t in [(M_(1,min)^((s)) - M_(1,0)) / n_1, quad (M_(1,max)^((s)) - M_(1,0)) / n_1] $ <time_interval_hyp>

An encounter requires both bodies to be in their respective intervals of the *same* zone *simultaneously*.

=== Choosing the iteration order

Label the body with the tighter encounter zone (smaller $M_(i,max)^((s)) - M_(i,min)^((s))$) as $i$ and the other as $j$. Iterating over passes of the tighter-zone body produces fewer candidates, since each pass occupies a shorter time interval and is less likely to overlap with the other body. The formulas below use $i$ for the outer loop and $j$ for the overlap check. For a hyperbolic spacecraft, always start with the spacecraft ($i = 1$): it has a single fixed time interval per zone (@time_interval_hyp), so just find which planet passes ($k_2$) overlap it.

=== First pass of body $i$ after $t_0$

For each zone $s$, body $i$ enters the encounter zone for the first time after $t_0$ on orbit number:

$ k_i^((s)) = ceil((n_i t_0 + M_(i,0) - M_(i,max)^((s))) / (2 pi)) $ <ki_start>

This is the smallest integer $k_i$ for which the encounter interval of body $i$ (@time_interval) has not ended before $t_0$. Start with $k_i^* = min_s k_i^((s))$, the earliest pass that enters *either* zone.

=== Checking for overlap with body $j$

On each pass $k_i^*$, check both zones $s in {1, 2}$. The encounter window of body $i$ in zone $s$, clamped to start no earlier than $t_0$, is:

$ t_(i,"lo")^((s)) = max((M_(i,min)^((s)) - M_(i,0) + 2 pi k_i^*) / n_i, quad t_0), quad t_(i,"hi")^((s)) = (M_(i,max)^((s)) - M_(i,0) + 2 pi k_i^*) / n_i $ <ti_window>

If $t_(i,"lo")^((s)) > t_(i,"hi")^((s))$, body $i$ is not in zone $s$ on this pass. Otherwise, the earliest orbit of body $j$ that could overlap is:

$ k_j = ceil((n_j t_(i,"lo")^((s)) + M_(j,0) - M_(j,max)^((s))) / (2 pi)) $ <kj_check>

Body $j$ enters zone $s$ on orbit $k_j$ at time $t_(j,"lo") = (M_(j,min)^((s)) - M_(j,0) + 2 pi k_j) / n_j$. If $t_(j,"lo") <= t_(i,"hi")^((s))$, the two windows overlap — this zone produces a candidate encounter. If both zones produce a match on the same pass, pick the one with the smaller $t_(j,"lo")$ (the earlier encounter). The resulting pair $(k_1, k_2)$ yields $alpha = n_2 k_1 - n_1 k_2$, which is the value to use in Newton's method on the combined system (@time_jacobian).

If neither zone produces a match on pass $k_i^*$, increment $k_i^* <- k_i^* + 1$ and repeat. For a hyperbolic spacecraft ($i = 1$), there is no incrementing — if no planet pass overlaps the spacecraft's single encounter window, no encounter exists on this orbit.

== Gradient of $h$

$ (partial h) / (partial E_1) = n_2 (1 - e_1 cos E_1), quad (partial h) / (partial E_2) = -n_1 (1 - e_2 cos E_2) $ <time_grad>

For a hyperbolic spacecraft orbit:

$ (partial h) / (partial H_1) = n_2 (e_1 cosh H_1 - 1) $ <time_grad_hyp>

This is always positive (since $e_1 > 1$ and $cosh H_1 >= 1$), matching the sign of the elliptic case.

== Newton's method on the combined system

For a given $alpha$, we solve the $2 times 2$ system $(f, h) = bold(0)$ using Newton's method. The Jacobian is:

$ bold(J) = mat(
  (partial f) / (partial E_1), (partial f) / (partial E_2);
  (partial h) / (partial E_1), (partial h) / (partial E_2)
) = mat(
  g_1, g_2;
  n_2 (1 - e_1 cos E_1), -n_1 (1 - e_2 cos E_2)
) $ <time_jacobian>

The Newton step $bold(delta) = (delta E_1, delta E_2)^top$ solves $bold(J) bold(delta) = -(f, h)^top$ via Cramer's rule:

$ Delta = -g_1 n_1 (1 - e_2 cos E_2) - g_2 n_2 (1 - e_1 cos E_1) $

$ delta E_1 = (f n_1 (1 - e_2 cos E_2) + g_2 h) / Delta $ <time_step1>

$ delta E_2 = (-f n_2 (1 - e_1 cos E_1) - g_1 h) / Delta $ <time_step2>

where $g_1$, $g_2$ are the distance-constraint gradient components (@grad1, @grad2) and $h$ is evaluated from @time_constraint_alpha (so $alpha$ enters the Newton step through $h$). Since $partial h \/ partial E_1 > 0$ and $partial h \/ partial E_2 < 0$ always, the Jacobian is non-singular whenever $(g_1, g_2)$ is not parallel to $(n_2 r_1 \/ a_1, -n_1 r_2 \/ a_2)$.

For a hyperbolic spacecraft orbit, the same Cramer's rule formulas apply with $(1 - e_1 cos E_1)$ replaced by $(e_1 cosh H_1 - 1)$ (@time_grad_hyp), $g_1$ from @grad1_hyp, and $h$ from @time_constraint_hyp.

#figure(
  placement: none,
  image("encounter_contours.svg", width: 100%),
  caption: [
    The distance constraint $f(E_1, E_2) = 0$ (solid black) and time constraints $h(E_1, E_2; alpha) = 0$ (dashed, coloured by year) for the Voyager 2 — Jupiter system (1970–1990). Each dashed curve corresponds to a different encounter opportunity, spaced by the spacecraft's orbital period ($T_1 approx 7$ years). Intersections of the dashed curves with the solid curve are solutions to the full encounter problem.
  ],
) <encounter_contours>


== Initial guess

The unconstrained minimisation of $f$ (from the Newton step in @constraint_C) yields the closest-approach point $(E_1^*, E_2^*)$ where $nabla f = 0$. At this point, evaluate $f_min := f(E_1^*, E_2^*)$ and the Hessian $bold(H)$ (@H11, @H22, @H12).

=== Quadratic approximation of $f = 0$

The second-order Taylor expansion of a multivariable function around a point $bold(a)$ is:

$ f(bold(x)) approx f(bold(a)) + nabla f(bold(a))^top (bold(x) - bold(a)) + 1/2 (bold(x) - bold(a))^top bold(H)(bold(a)) (bold(x) - bold(a)) $

Let $bold(a) = (E_1^*, E_2^*)$, $bold(delta) = (E_1^* - E_1, E_2^* - E_2)^top$, and $bold(x) = bold(a) - bold(delta)$. At the minimum, the gradient vanishes ($nabla f = 0$), so the linear term drops out (the sign of $bold(delta)$ does not matter in the quadratic term):

$ f approx f_min + 1/2 bold(delta)^top bold(H) bold(delta) $

Setting $f = 0$:

$ bold(delta)^top bold(H) bold(delta) = -2 f_min $ <ellipse_approx>

Written out, the left side is $H_(11) delta_1^2 + 2 H_(12) delta_1 delta_2 + H_(22) delta_2^2$ — the general equation of a conic section. Since $bold(H)$ is positive definite at a minimum, this conic is an ellipse. The right side $-2 f_min > 0$ when $f_min < 0$ (the orbits come within $r_"SOI"$), so the ellipse is valid.

=== Linearised time constraint

The time constraint $h(E_1, E_2; alpha) = 0$ is linearised around $(E_1^*, E_2^*)$:

$ h^* - h_1^* delta_1 - h_2^* delta_2 = 0 $

where $h^* = h(E_1^*, E_2^*; alpha)$, $h_1^* = n_2 (1 - e_1 cos E_1^*)$, $h_2^* = -n_1 (1 - e_2 cos E_2^*)$ (@time_grad). This gives $delta_2 = c + m delta_1$ with:

$ c = h^* / h_2^*, quad m = h_1^* / h_2^* $

=== Intersection

Substituting $delta_2 = c + m delta_1$ into @ellipse_approx yields a quadratic in $delta_1$:

$ A delta_1^2 + B delta_1 + C = 0 $

with coefficients:

$ A = H_(11) + 2 H_(12) m + H_(22) m^2 $
$ B = 2 c (H_(12) + H_(22) m) $
$ C = H_(22) c^2 + 2 f_min $

The two roots correspond to the two intersections of the time-constraint line with the encounter ellipse. The initial guess is the root with the larger $delta_1$ (smaller $E_1$, earlier encounter):

$ delta_1 = (-B + sqrt(B^2 - 4 A C)) / (2 A) $ <initial_guess>

giving $(E_1^((0)), E_2^((0))) = (E_1^* - delta_1, E_2^* - c - m delta_1)$. If the discriminant $B^2 - 4 A C < 0$, the time-constraint line does not intersect the encounter ellipse for this $alpha$, and this encounter opportunity can be skipped.
