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

== Encounter Intervals in $E_1$ and $E_2$

The constraint $f(E_1, E_2) = 0$ defines an implicit curve (or set of curves) in the $(E_1, E_2)$ plane. The *encounter interval* $[E_(1,min), E_(1,max)]$ is the projection of this curve onto the $E_1$ axis, and $[E_(2,min), E_(2,max)]$ is the projection onto the $E_2$ axis. For every $E_1$ in the first interval there exists at least one $E_2$ in the second interval such that the distance equals $r_"SOI"$.

=== Extrema via implicit differentiation

Along the constraint curve $f(E_1, E_2) = 0$, total differentiation gives:

$ (partial f) / (partial E_1) dif E_1 + (partial f) / (partial E_2) dif E_2 = 0 $

so the implicit derivatives are:

$ (dif E_1) / (dif E_2) = -(partial f \/ partial E_2) / (partial f \/ partial E_1), quad (dif E_2) / (dif E_1) = -(partial f \/ partial E_1) / (partial f \/ partial E_2) $ <implicit_deriv>

The extrema of $E_1$ along the constraint curve occur where the tangent is vertical, i.e. $dif E_1 \/ dif E_2 = 0$, which requires:

$ (partial f) / (partial E_2) = 0 $ <E1_crit>

Similarly, the extrema of $E_2$ occur where $dif E_2 \/ dif E_1 = 0$, requiring:

$ (partial f) / (partial E_1) = 0 $ <E2_crit>

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

=== Geometric interpretation

The constraint curve $f = 0$ forms a closed loop (or multiple loops) in the $(E_1, E_2)$ torus. The critical points @E1_crit are where the curve has a vertical tangent — $E_1$ reaches a turning point as $E_2$ varies. The two solutions give $E_(1,min)$ and $E_(1,max)$. Likewise, @E2_crit gives horizontal tangents where $E_2$ is extremal, yielding $E_(2,min)$ and $E_(2,max)$.

#import "@preview/cetz:0.3.4"

#figure(
  cetz.canvas(length: 1cm, {
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

= Time Constraint via Kepler's Equation

The preceding sections treated $E_1$ and $E_2$ as independent variables. In reality, both bodies obey Kepler's equation, which couples each eccentric anomaly to a common time $t$.

== Kepler's equation

For each orbit $i$ with mean motion $n_i = 2 pi \/ T_i$ (where $T_i$ is the orbital period) and mean anomaly at epoch $M_(i,0)$:

$ M_i (t) = M_(i,0) + n_i t = E_i - e_i sin E_i $ <kepler>

This implicitly defines $E_i (t)$. At a given time $t$, both eccentric anomalies are determined:

$ E_1 - e_1 sin E_1 = M_(1,0) + n_1 t, quad E_2 - e_2 sin E_2 = M_(2,0) + n_2 t $ <kepler_both>

== Eliminating time

The position on an ellipse is $2 pi$-periodic in $E_i$, so the spacecraft returns to the same point after each full orbit. The set of times at which orbit $i$ passes through eccentric anomaly $E_i in [-pi, pi]$ is:

$ t = (E_i - e_i sin E_i - M_(i,0) + 2 pi k_i) / n_i, quad k_i in ZZ $ <time_set>

For an encounter, both bodies must be at their respective positions *simultaneously*. Equating the two time expressions:

$ (E_1 - e_1 sin E_1 - M_(1,0) + 2 pi k_1) / n_1 = (E_2 - e_2 sin E_2 - M_(2,0) + 2 pi k_2) / n_2 $

Rearranging gives a family of *time-coupling constraints*, one for each integer pair $(k_1, k_2)$:

$ h_(k_1,k_2) (E_1, E_2) := n_2 (E_1 - e_1 sin E_1 - M_(1,0)) - n_1 (E_2 - e_2 sin E_2 - M_(2,0)) + 2 pi (n_2 k_1 - n_1 k_2) = 0 $ <time_constraint>

Each choice of $(k_1, k_2)$ corresponds to a different encounter opportunity (i.e.~orbit $1$ on its $k_1$-th revolution meeting orbit $2$ on its $k_2$-th revolution). Since only the combination $n_2 k_1 - n_1 k_2$ appears, the distinct constraints are parametrised by a single offset:

$ h(E_1, E_2; lambda) := n_2 (E_1 - e_1 sin E_1 - M_(1,0)) - n_1 (E_2 - e_2 sin E_2 - M_(2,0)) + 2 pi lambda = 0 $ <time_constraint_lambda>

where $lambda = n_2 k_1 - n_1 k_2$ ranges over a discrete set. In the $(E_1, E_2)$ plane (with $E_i in [-pi, pi]$), each value of $lambda$ gives a monotone curve from bottom-left to top-right, and these curves are spaced apart by the synodic offset. The full encounter problem is: find $(E_1, E_2)$ satisfying both the distance constraint $f = 0$ (@constraint_C) and $h = 0$ (@time_constraint_lambda) for some admissible $lambda$.

== Enumerating encounter opportunities

Since $E_i$ and $sin E_i$ are $2 pi$-periodic, shifting $k_2$ by $1$ shifts $lambda$ by $-n_1$, which shifts $E_2$ by exactly $2 pi$ — a full orbit. On the torus $E_i in [-pi, pi]$, different $k_2$ values for the same $k_1$ therefore produce the *same* curve. The distinct encounter opportunities are parametrised by $k_1$ alone.

=== Finding the $k_1$ range from a time window

Given a time window $[t_"lo", t_"hi"]$, the spacecraft passes through eccentric anomaly $E_1 in [-pi, pi]$ on its $k_1$-th orbit at time (@time_set):

$ t = (E_1 - e_1 sin E_1 - M_(1,0) + 2 pi k_1) / n_1 $

Since $E_1 - e_1 sin E_1 in [-pi, pi]$, the admissible $k_1$ values satisfy:

$ k_(1,min) = floor((n_1 t_"lo" + M_(1,0) - pi) / (2 pi)), quad k_(1,max) = ceil((n_1 t_"hi" + M_(1,0) + pi) / (2 pi)) $ <k1_range>

=== Choosing $k_2$ for each $k_1$

For each $k_1$, the approximate encounter time is $t approx (-M_(1,0) + 2 pi k_1) \/ n_1$ (at $E_1 approx 0$). The corresponding $k_2$ is simply:

$ k_2 = op("round")((n_2 t + M_(2,0)) / (2 pi)) $ <k2_choice>

This gives $lambda = n_2 k_1 - n_1 k_2$, and each $k_1$ in $[k_(1,min), k_(1,max)]$ is a candidate encounter opportunity that can be refined with Newton's method.

== Gradient of $h$

$ (partial h) / (partial E_1) = n_2 (1 - e_1 cos E_1), quad (partial h) / (partial E_2) = -n_1 (1 - e_2 cos E_2) $ <time_grad>

Note that $1 - e_i cos E_i = r_i \/ a_i > 0$ for elliptic orbits, so $partial h \/ partial E_1 > 0$ and $partial h \/ partial E_2 < 0$ everywhere. The time constraint is therefore a monotone relation between $E_1$ and $E_2$:

$ (dif E_2) / (dif E_1) = (n_2 (1 - e_1 cos E_1)) / (n_1 (1 - e_2 cos E_2)) = (n_2 r_1 \/ a_1) / (n_1 r_2 \/ a_2) > 0 $ <time_slope>

== Newton's method on the combined system

For a given $lambda$, we solve the $2 times 2$ system $(f, h) = bold(0)$ using Newton's method. The Jacobian is:

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

where $g_1$, $g_2$ are the distance-constraint gradient components (@grad1, @grad2). Since $partial h \/ partial E_1 > 0$ and $partial h \/ partial E_2 < 0$ always, the Jacobian is non-singular whenever $(g_1, g_2)$ is not parallel to $(n_2 r_1 \/ a_1, -n_1 r_2 \/ a_2)$.

== Initial guess

For a given $lambda$, pick $E_1^((0))$ as the midpoint of $[E_(1,min), E_(1,max)]$. Compute the corresponding time from @time_set (with $k_1 = 0$):

$ t = (E_1^((0)) - e_1 sin E_1^((0)) - M_(1,0)) / n_1 $

Rearranging the time constraint $h(E_1^((0)), E_2^((0)); lambda) = 0$ (@time_constraint_lambda) for $E_2$'s mean anomaly:

$ M_2^((0)) := E_2^((0)) - e_2 sin E_2^((0)) = M_(2,0) + n_2 t + (2 pi lambda) / n_1 $

Substituting the expression for $t$:

$ M_2^((0)) = M_(2,0) + (n_2) / (n_1) (E_1^((0)) - e_1 sin E_1^((0)) - M_(1,0)) + (2 pi lambda) / n_1 $

Then solve Kepler's equation $E_2^((0)) - e_2 sin E_2^((0)) = M_2^((0))$ (e.g.~by Newton iteration) to obtain $E_2^((0))$. This ensures the initial point lies exactly on the time-constraint curve for the chosen $lambda$.

#figure(
  placement: none,
  image("encounter_contours.svg", width: 80%),
  caption: [
    The distance constraint $f(E_1, E_2) = 0$ (solid black) and time constraints $h(E_1, E_2; lambda) = 0$ (dashed, coloured by year) for the Voyager 2 — Jupiter system (1970–1990). Each dashed curve corresponds to a different encounter opportunity (synodic period $approx 7$ years). Intersections of the dashed curves with the solid curve are solutions to the full encounter problem.
  ],
) <encounter_contours>

