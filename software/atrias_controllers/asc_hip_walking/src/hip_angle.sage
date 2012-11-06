#!/usr/bin/env python
# Not sure if the above is right... but who cares, we're running through Sage!

# Make these global, so setToePos() works
T_x = 0
T_y = 0
T_z = 0

# Sets T_x, T_y, and T_z to the position of the toe, substituting in variables where possible.
def setToePos():
	# This allows this function to modify the global copies of T_x, T_y, and T_z
	global T_x, T_y, T_z
	T_x = cos(a_l)*l
	T_y = -l_b*cos(a_b) - w*sin(a_h) - l*cos(a_h)*sin(a_l)
	T_z = h_b + l_b*sin(a_b) - cos(a_h)*w + l*sin(a_h)*sin(a_l)

# a_l = leg angle
# l   = leg length
# l_b = virtual boom length
# a_b = virtual boom angle
# w   = hip pivot to leg pivot length
# a_h = hip angle
# h_b = boom pivot height
var('a_l l l_b a_b w a_h h_b rad')

a_l = pi/2 - .2
l_b = 2.006
a_b = 3.1777
w   = -.16111
h_b = .915

setToePos()

# Solve for and set our leg length
A = solve([T_z == 0], l, solution_dict=True)
l = A[0][l]

# Substitute l into our equations
setToePos()

# Setting our target
rad = 2.164 + 2*w

error(a_h) = T_x^2 + T_y^2 - rad^2

a_h = find_root(error, 1.5*pi - 20*pi/180, 1.5*pi + 10*pi / 180)

l = var('l')

setToePos()
l = solve([T_z == 0], l, solution_dict=True)[0][l]

setToePos()

l = l.n()

# Failed attempt at an analytical solution
#A = solve([(T_x)^2 + (T_y)^2 == rad^2, T_z == 0], l, a_h)
