#!/usr/bin/python
# coding: UTF-8

import numpy as np
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
from scipy import genfromtxt

import csv



step = 100
points = []
delta_t = 1./step
for i in range(step):
    angle = 2*np.pi*i/step
    c, s = np.cos(angle), np.sin(angle)
    R = np.matrix(((c,-s), (s, c)))
    p2 = R * np.matrix((1.,0.)).T
    p3 =  np.append(np.array(p2).ravel(),i*0.01)
    p3_with_angle =  np.append(p3,angle)
    points.append(p3_with_angle)
points = np.array(points)

with open('../build/node0.csv', 'w') as f:
    writer = csv.writer(f, lineterminator='\n')
    writer.writerows(points)


"""
fig = pyplot.figure()
ax = Axes3D(fig)
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
ax.plot(points[:,0], points[:,1], points[:,2], "o", ms=4, mew=0.5)
pyplot.show()
"""

for i in range(1,step-1):
    start_position = points[i-1,:]
    middle_position = points[i,:]
    end_position = points[i+1,:]
    start_velocity = (middle_position - start_position)/delta_t
    end_velocity = (end_position - middle_position)/delta_t
    delta_velocity = end_velocity - start_velocity
    #print i,delta_velocity

    

