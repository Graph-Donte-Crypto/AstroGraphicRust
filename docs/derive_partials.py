"""
Derive partial derivatives of the squared distance between two Keplerian orbits
with respect to eccentric anomalies E1 and E2, then simplify analytically.

The squared distance is (from soi_encounter_derivation.typ, eq. constraint_C):

    f(E1, E2) = r1^2 + r2^2 - r1^T C r2

where:
    r_i = (a_i (cos E_i - e_i), b_i sin E_i)
    C is a general 2x2 matrix encoding mutual orbital plane orientation
"""

from sympy import *

init_printing(use_unicode=True, wrap_line=False)

# Eccentric anomalies
E1, E2 = symbols("E_1 E_2")

# Orbital elements
a1, b1, e1 = symbols("a_1 b_1 e_1")
a2, b2, e2 = symbols("a_2 b_2 e_2")

# Coupling matrix elements (general 2x2, not necessarily symmetric)
C11, C12, C21, C22 = symbols("C_11 C_12 C_21 C_22")

# Position vectors in orbital planes
r1 = Matrix([a1 * (cos(E1) - e1), b1 * sin(E1)])
r2 = Matrix([a2 * (cos(E2) - e2), b2 * sin(E2)])

C = Matrix([[C11, C12], [C21, C22]])

# Squared norms
r1_sq = (a1 * (1 - e1 * cos(E1)))**2
r2_sq = (a2 * (1 - e2 * cos(E2)))**2

# Cross term
cross = (r1.T * C).dot(r2)

# Squared distance
f = r1_sq + r2_sq - cross

#pprint(simplify(f))

# Gradient
df_dE1, df_dE2 = [expand(g) for g in derive_by_array(f, [E1, E2])]

# Collect pure sin(Ei) terms (exclude cross-terms like sin(E1)*sin(E2))
def collect_pure_trig(expr, func, E_target, E_other):
    """Group pure func(E_target) terms in expr, leaving the rest untouched."""
    expr = expand(expr)
    other_func = cos if func == sin else sin
    coeff = S.Zero
    rest = S.Zero
    for term in Add.make_args(expr):
        c = term.coeff(func(E_target))
        if c != 0 and not c.has(E_other) and not c.has(other_func(E_target)):
            coeff += c
        else:
            rest += term
    return rest + UnevaluatedExpr(coeff) * func(E_target)

df_dE1 = collect_pure_trig(df_dE1, sin, E1, E2)
df_dE2 = collect_pure_trig(df_dE2, sin, E2, E1)

print("-----\ndf/dE1:")
pprint(trigsimp(df_dE1))
print("-----\ndf/dE2:")
pprint(trigsimp(df_dE2))

# Hessian
E = [E1, E2]
H = Matrix(2, 2, lambda i, j: expand(diff(f, E[i], E[j])))

H00 = trigsimp(H[0,0])
H00 = collect_pure_trig(H00, cos, E1, E2)
H00 = collect_pure_trig(H00, sin, E1, E2)
H11 = trigsimp(H[1,1])
H11 = collect_pure_trig(H11, cos, E2, E1)
H11 = collect_pure_trig(H11, sin, E2, E1)

print("\n-----\nd²f/dE1²:")
pprint(H00)
print("-----\nd²f/dE1dE2:")
pprint(trigsimp(H[0, 1]))
print("-----\nd²f/dE2²:")
pprint(H11)
