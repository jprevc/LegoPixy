# -*- coding: utf-8 -*-
"""
Created on Fri Dec 30 08:44:57 2016

@author: jostp
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl

from LegoSimFunctions import *
from LegoKinematics import *

Ts = 0.1  # sample time in s
Tfin = 10 # Time of simulation

t_vec = np.arange(0, Tfin, Ts)
NumOfSamples = len(t_vec)

fig1 = plt.figure(1)
plt.xlim(-500, 1000)
plt.ylim(-500, 1000)
plt.grid('on')
robotPose = np.zeros((NumOfSamples,3)) # n×3 matrix of robot pose, first two columns are x and y position in mm,
                                       # 3rd column is robot orientation in deg
# robot's estimated starting pose
qRobotEstStartPose = np.array([0, 0, 0])
# initialize particles for particle filter
nParticles = 10
qParticles = np.dot(np.diag(np.array([500, 500, 180])),np.random.rand(3,nParticles)-0.5)

# initialize weights for particles
particleWeights = np.ones((nParticles, 1)) / nParticles

# draw initial particle position
#plt.plot(qParticles[0,:],qParticles[1,:], 'rx')
plt.quiver(qParticles[0,:],qParticles[1,:], np.cos(qParticles[2,:] * np.pi / 180), np.sin(qParticles[2,:] * np. pi / 180))

## robot initial position and orientation
#robotPose[0,:] = [600,400,0]
#
#for i in range(1,NumOfSamples):
#    
#    # draw current robot position
#    plt.cla() # clear figure
#    drawRobot(fig1, robotPose[i-1,:])
#    plt.pause(0.05)
#    
#    # TODO: get real motor DC values from robot
#    
#    # get current speed of robot according to DC values    
#    curSpeed = robotExternalKinematics((20,30), robotPose[i-1,2])
#    
#    # compute new robot pose
#    robotPose[i,:] = robotComputeNewPose(robotPose[i-1,:], curSpeed, Ts)
#    