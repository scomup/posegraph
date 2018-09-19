#!/usr/bin/python
# coding: UTF-8

import numpy as np
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
from scipy import genfromtxt

import csv
import pandas as pd
points0 = pd.read_csv('../build/node0.csv')
points0 = np.array(points0)

points1 = pd.read_csv('../build/node1.csv')
points1 = np.array(points1)

fig = pyplot.figure()
ax = Axes3D(fig)
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")


ax.plot(points0[:,0], points0[:,1], points0[:,2], "o", ms=4, mew=0.5,  label='before')
ax.plot(points1[:,0], points1[:,1], points1[:,2], "o", ms=4, mew=0.5,  label='after')
pyplot.legend()

pyplot.show()


    

