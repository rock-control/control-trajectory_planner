import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

points_array=numpy.loadtxt('./points.txt')
b_curve = numpy.loadtxt('./output.txt')
b_curve2 = numpy.loadtxt('./output2.txt')
b_curve3 = numpy.loadtxt('./output3.txt')
b_curve4 = numpy.loadtxt('./output4.txt')
b_curve5 = numpy.loadtxt('./output5.txt')
plt.plot(points_array, 'g*')
num_intervals=100
interval=1/num_intervals
t=numpy.linspace(0, len(points_array)-1, (len(points_array)-1)*num_intervals+1)
plt.plot(t,b_curve)
plt.plot(t,b_curve2)
plt.plot(t,2*b_curve2)
plt.plot(t,b_curve3)
plt.show()


