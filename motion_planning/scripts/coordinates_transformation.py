import math

boom_len = 48.2
stick_len = 25.5

'''
In all the functions, theta1 -> angle of boom; theta2 -> angle of stick
theta -> angle of the base
'''
#Cartesian => cylindrical
def cart2polar(x,y,z):
	polar = []
	polar.append(math.sqrt(x*x+y*y))
	polar.append(math.atan(y/x))
	polar.append(z)
	return polar

#Cylindrical => Cartesian
def polar2cart(r,theta,z):
	cart = []
	cart.append(r*math.cos(theta))
	cart.append(r*math.sin(theta))
	cart.append(z)
	return cart

#Angles known, find r an z
def angle2rz(theta1, theta2):
	r = boom_len*math.cos(theta1) + stick_len*math.cos(theta2)
	z = boom_len*math.sin(theta1) + stick_len*math.sin(theta2)
	rz = []
	rz.append(r)
	rz.append(z)
	return rz

#r and z known, find angles
def rz2angle(r,z):
	len_square = r*r + z*z
	A = len_square - boom_len*boom_len + stick_len * stick_len
	B = math.sqrt((len_square-(boom_len-stick_len)*
	(boom_len-stick_len))*((boom_len+stick_len)*(boom_len+stick_len)-len_square))
	angles = []
	b = (-r*A+z*B)/(2*len_square)
	angles.append(math.asin((z-b)/boom_len))
	if (A-2*z*b>0):
		angles.append(math.asin(b/stick_len))
	else:
		angles.append(-1*math.asin(b/stick_len)-math.pi)
	return angles

#Cartesian coordinates known, find angles			
def cart2angle(x,y,z):
	polar = cart2polar(x,y,z)
	arm_angles = rz2angle(polar[0], polar[2])
	angles = []
	angles.append(arm_angles[0])
	angles.append(arm_angles[1])
	angles.append(polar[1])
	return angles
#Angles known, find Cartesian coordinates
def angle2cart(theta1, theta2, theta):
	polar = []
	rz = angle2rz(theta1, theta2)
	polar.append(rz[0])
	polar.append(theta)
	polar.append(rz[1])
	cart = polar2cart(polar[0], polar[1], polar[2])
	return cart

#Do not forget to transfer the final answers from rads into degrees
def rad2degree(angles):
	degrees = [x*math.pi/180 for x in angles]
	return degrees
