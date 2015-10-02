import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

points_array=numpy.loadtxt('./points.txt')
b_curve1 = numpy.loadtxt('./output.txt')

fig=plt.figure()

ax = fig.add_subplot(1,1,1, projection='3d')

ax.plot(points_array[:,0], points_array[:,1],points_array[:,2],'g*')

#plot bezier curve
ax.plot(b_curve1[:,0],b_curve1[:,1],b_curve1[:,2])

ax.set_title("Cubic Bezier Spline")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend(["Control Points","Bezier Curve"], loc=2)
plt.show()

