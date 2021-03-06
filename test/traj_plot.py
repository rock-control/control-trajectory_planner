from __future__ import division
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm


joints_traj=numpy.loadtxt('./joints_traj.txt')

pos_bCurve0 = numpy.loadtxt('./pos_bCurve0.txt')
pos_bCurve = numpy.loadtxt('./pos_bCurve.txt')
pos_bCurve00= numpy.loadtxt('./output.txt')
vel_bCurve = numpy.loadtxt('./vel_bCurve.txt')
acc_bCurve = numpy.loadtxt('./acc_bCurve.txt')
#vel_bCurve0 = numpy.loadtxt('./vel_bCurve0.txt')
vel_bCurve0_deriv = numpy.loadtxt('./vel_bCurve0_deriv.txt')
vel_bCurve_deriv = numpy.loadtxt('./vel_bCurve_deriv.txt')
acc_bCurve_deriv = numpy.loadtxt('./acc_bCurve_deriv.txt')
time=numpy.loadtxt('./time.txt')
#plt.plot(joints_traj, 'g*')
#plt.plot(joints_traj[:,1], 'b*')
#plt.plot(joints_traj[:,2], 'r*')



period=0.01
num_intervals=100
interval=1/num_intervals

noPts=len(joints_traj[:,0])
print"no Given Pts %d"%noPts
noSteps=len(time)-1
print"noSteps %d"%noSteps
time_interval=noSteps/((noPts-1)*num_intervals)
final_time=(noSteps/num_intervals)*1.0

t2=numpy.linspace(0, final_time, noPts)

print t2
print "time interval %d"%time_interval
print "final time %d"%final_time

#plt.plot(t,diff)
#plt.plot(t,vel_bCurve0)
#plt.plot(t,pos_bCurve0)

no_fig=3
fig = plt.figure()
ax1 = fig.add_subplot(no_fig,1,1)
ax1.plot(time,pos_bCurve[:,0])
ax1.plot(time,pos_bCurve[:,1])
ax1.plot(time,pos_bCurve[:,2])
ax1.set_title("Joint Positions versus time")
ax1.set_ylabel("Position in rad")
ax1.legend(["J1","J2", "J3"], loc=2)
#ax1.plot(t2,joints_traj[:,0], 'b*')
#ax1.plot(t2,joints_traj[:,1], 'g*')
#ax1.plot(t2,joints_traj[:,2], 'r*')

#ax2 = fig.add_subplot(no_fig,1,2)
#ax2.plot(time,vel_bCurve_deriv[:,0])
#ax2.plot(time,vel_bCurve_deriv[:,1])
#ax2.plot(time,vel_bCurve_deriv[:,2])
#ax2.set_title("Joint Velocities versus time (Numerical differentiation)")
#ax2.set_ylabel("Vel in rad/s")

#ax3 = fig.add_subplot(no_fig,1,3)
#ax3.plot(time,acc_bCurve_deriv[:,0])
#ax3.plot(time,acc_bCurve_deriv[:,1])
#ax3.plot(time,acc_bCurve_deriv[:,2])
#ax3.set_title("Joint Acc versus time (Numerical differentiation)")
#ax3.set_ylabel("Acc in rad/s2")

ax4 = fig.add_subplot(no_fig,1,2)
ax4.plot(time,vel_bCurve[:,0])
ax4.plot(time,vel_bCurve[:,1])
ax4.plot(time,vel_bCurve[:,2])
ax4.set_title("Joint Velocities versus time (Analytical)")
ax4.set_ylabel("Vel in rad/s")

ax5 = fig.add_subplot(no_fig,1,3)
ax5.plot(time,acc_bCurve[:,0])
ax5.plot(time,acc_bCurve[:,1])
ax5.plot(time,acc_bCurve[:,2])
ax5.set_title("Joint Acc versus time (Analytical)")
ax5.set_ylabel("Acc in rad/s2")
ax5.set_xlabel("time in seconds")
plt.show()
