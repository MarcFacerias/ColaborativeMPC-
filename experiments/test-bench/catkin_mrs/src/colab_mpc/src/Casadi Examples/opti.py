# https://web.casadi.org/docs/#document-ocp
import casadi

opti = casadi.Opti() # class used as a suport to have easier syntaxis

x = opti.variable()
y = opti.variable()

opti.minimize(  (y-x**2)**2   )
opti.subject_to( x**2+y**2==1 )
opti.subject_to(       x+y>=1 )

opti.solver('ipopt')

sol = opti.solve()

print(sol.value(x))
print(sol.value(y))