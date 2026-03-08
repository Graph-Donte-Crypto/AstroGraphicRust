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

== Expanding the Scalar Terms

=== Heliocentric distances $r_i^2$

$ r_i^2 = a_i^2 (1 - e_i cos E_i)^2 $

=== Cross-term $bold(r)_1^top bold(C) bold(r)_2$

Decompose $bold(r)_i$ into its centered-ellipse part and eccentricity offset:

$ bold(r)_i (E_i) = bold(r)_(0i)(E_i) - bold(æ)_i, quad bold(r)_(0i) = vec(a_i cos E_i, b_i sin E_i), quad bold(æ)_i = vec(a_i e_i, 0) $ <crossterm>

$ bold(r)_1^top bold(C) bold(r)_2 = (bold(r)_(01) - bold(æ)_1)^top bold(C) (bold(r)_(02) - bold(æ)_2) = bold(r)_(01)^top bold(C) bold(r)_(02) - bold(æ)_1^top bold(C) bold(r)_(02) - bold(r)_(01)^top bold(C) bold(æ)_2 + bold(æ)_1^top bold(C) bold(æ)_2 $

=== Constant sub-expressions

Since $bold(C)$, $bold(æ)_1$, and $bold(æ)_2$ are all constants (independent of $E_1$, $E_2$), we introduce the following constants:

$ bold(p)^top := bold(æ)_1^top bold(C) in RR^2, quad bold(q) := bold(C) bold(æ)_2 in RR^2, quad s := bold(æ)_1^top bold(C) bold(æ)_2 $ <s_def>


$ bold(r)_1^top bold(C) bold(r)_2 = bold(r)_(01)^top bold(C) bold(r)_(02) - bold(p)^top bold(r)_(02) - bold(r)_(01)^top bold(q) + s $

$ r_1^2 + r_2^2 - bold(r)_1^top bold(C) bold(r)_2 =  a_1^2 (1 - e_1 cos E_1)^2 + a_2^2 (1 - e_2 cos E_2)^2 - bold(r)_(01)^top bold(C) bold(r)_(02) + bold(p)^top bold(r)_(02) + bold(r)_(01)^top bold(q) - s $

== Stationarity Conditions

To find the closest approach, minimize the squared distance:

$ f(E_1, E_2) := r_1^2 + r_2^2 - bold(r)_1^top bold(C) bold(r)_2 $

The eccentric-anomaly derivatives of the centered position vectors are:

$ dot(bold(r))_(0i) := (partial bold(r)_(0i)) / (partial E_i) = vec(-a_i sin E_i, b_i cos E_i) $

Since $bold(æ)_i$ is constant, $(partial bold(r)_i) / (partial E_i) = dot(bold(r))_(0i)$.

=== Partial derivatives

Setting the partial derivatives to zero and using $(partial r_i^2) / (partial E_i) = 2 a_i^2 e_i sin E_i (1 - e_i cos E_i)$:

$ (partial f) / (partial E_1) = 2 a_1^2 e_1 sin E_1 (1 - e_1 cos E_1) - dot(bold(r))_(01)^top bold(C) bold(r)_2 = 0 $ <stationary_E1>

$ (partial f) / (partial E_2) = 2 a_2^2 e_2 sin E_2 (1 - e_2 cos E_2) - bold(r)_1^top bold(C) dot(bold(r))_(02) = 0 $ <stationary_E2>

=== Scaled coupling constants

To expand the bilinear terms into trigonometric form, define the four scaled coupling constants:

$ alpha := a_1 a_2 C_(11), quad beta := a_1 b_2 C_(12), quad gamma := b_1 a_2 C_(21), quad delta := b_1 b_2 C_(22) $

The cross-terms in @stationary_E1 and @stationary_E2 expand as:

$ dot(bold(r))_(01)^top bold(C) bold(r)_2 = (-alpha sin E_1 + gamma cos E_1)(cos E_2 - e_2) + (-beta sin E_1 + delta cos E_1) sin E_2 $

$ bold(r)_1^top bold(C) dot(bold(r))_(02) = (-alpha sin E_2 + beta cos E_2)(cos E_1 - e_1) + (-gamma sin E_2 + delta cos E_2) sin E_1 $

The full stationarity conditions are therefore:

$ 2 a_1^2 e_1 sin E_1 (1 - e_1 cos E_1) = (-alpha sin E_1 + gamma cos E_1)(cos E_2 - e_2) + (-beta sin E_1 + delta cos E_1) sin E_2 $

$ 2 a_2^2 e_2 sin E_2 (1 - e_2 cos E_2) = (-alpha sin E_2 + beta cos E_2)(cos E_1 - e_1) + (-gamma sin E_2 + delta cos E_2) sin E_1 $
