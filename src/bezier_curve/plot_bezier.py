import numpy
#points_array = numpy.array([[1, 2, 4],[6,5,9],[8,12,3],[21,12,13]])
#points_array=numpy.array([[1,2],[4,4],[6,4],[2,3],[5,6]])
points_array=numpy.loadtxt('./points.txt')
b_curve1 = numpy.loadtxt('./b_curve.txt')
#plotting example
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
fig=plt.figure()

ax2=fig.add_subplot(1,1,1, projection='3d')
#plot specified points
ax2.plot(points_array[:,0], points_array[:,1],points_array[:,2],'g*')
#plot bezier curve
ax2.plot(b_curve1[:,0],b_curve1[:,1],b_curve1[:,2])
ax2.set_title("Cubic Bezier Spline from C++")
ax2.set_xlabel("X")
ax2.set_ylabel("Y")
ax2.set_zlabel("Z")
ax2.legend(["Control Points","Bezier Curve"], loc=2)
plt.show()

