#!/usr/bin/env python

import dvrk
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy
import PyKDL

p = dvrk.psm('PSM1')
x = raw_input("press 1 when ready for first point: ")
if x != '1':
	print("Wrong input: rerun script")
else:
	xyz1 = p.get_current_position()
print("XYZ:" + str(xyz1.p))
print("Rotation" + str(xyz1.M))

y = raw_input("press 2 when ready for second point: ")
if y != '2':
	print("Wrong input: rerun script")
else:
	xyz2 = p.get_current_position()
print("XYZ:" + str(xyz2.p))
print("Rotation" + str(xyz2.M))

z = raw_input("press 3 when ready for third point: ")
if z != '3':
	print("Wrong input: rerun script")
else:
	xyz3 = p.get_current_position()
print("XYZ:" + str(xyz3.p))
print("Rotation" + str(xyz3.M))

Xa = xyz2.p - xyz1.p
Xb = xyz3.p - xyz1.p
print(Xa)
print(Xb)

z = Xa*Xb
print(z)
y = Xa*z
print(y)


z_norm = z.Normalize()
y_norm = y.Normalize()
Xa_norm = Xa.Normalize()
print("X normalized: " + str(Xa))
print("Y normalized: " + str(y))
print("Z normalized: " + str(z))

#vectors = numpy.array([[xyz1.p[0],xyz1.p[1],xyz1.p[2],Xa[0],Xa[1],Xa[2]],
#					   [xyz1.p[0],xyz1.p[1],xyz1.p[2],y[0],y[1],y[2]],
#					   [xyz1.p[0],xyz1.p[1],xyz1.p[2],z[0],z[1],z[2]]])
#X,Y,Z,U,V,W = zip(*vectors)
#fig=plt.figure()
#ax = fig.add_subplot(111,projection='3d')
#ax.quiver(X,Y,Z,U,V,W)
#plt.show()

ori_v = PyKDL.Vector(xyz1.p[0],xyz1.p[1],xyz1.p[2])
ori_r = PyKDL.Rotation(Xa[0],y[0],z[0],Xa[1],y[1],z[1],Xa[2],y[2],z[2])
surgical_cs = PyKDL.Frame(ori_r,ori_v)
#ori_xyz_c = numpy.array( [[ Xa[0],  y[0],  z[0]] ,[Xa[1], y[1] ,z[1]], [Xa[2], y[2] ,z[2]]]) 
print(surgical_cs)
#print(ori_xyz_c)
