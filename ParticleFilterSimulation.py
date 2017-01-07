# -*- coding: utf-8 -*-
"""
Created on Sat Jan  7 13:33:07 2017

@author: jostp
"""

import numpy as np
import matplotlib.pyplot as plt
from LegoKinematics import *
from LegoSimFunctions import *
from ParticleFilter import * 

Ts = 0.1  # sample time in s
Tfin = 10 # Time of simulation

# sensor distance variance in mm
sensorVariance = 0.01

t_vec = np.arange(0, Tfin, Ts)
NumOfSamples = len(t_vec)

# object positions
objPos = np.array([[-200,-200], [200,600], [600,200]]).transpose()

# draw environment
fig1 = plt.figure(1)
plt.xlim(-500, 1000)
plt.ylim(-500, 1000)

# draw objects
drawRectangle(fig1, objPos[:,0], 40, 20, iCol="red")
drawRectangle(fig1, objPos[:,1], 40, 20, iCol="green")
drawRectangle(fig1, objPos[:,2], 40, 20, iCol="blue")

# simulated robot starting pose
qRobotRealPose = np.array([50, -50, 20], ndmin=2).transpose()
#drawRobot(fig1, qRobotRealPose)

# estimated robot start pose
qRobotEstStartPose = np.array([0, 0, 0], ndmin=2).transpose()

# initialize particles for particle filter
nParticles = 5
particlePosDeviation = 500    # particle position deviation from robot's initial estimated position in mm
particleOrientDeviation = 180 # particle orientation deviation from robot's initial estimated orientation in degrees
qParticles = qRobotEstStartPose + np.dot(np.diag(np.array([particlePosDeviation, particlePosDeviation, particleOrientDeviation])), np.random.rand(3,nParticles)-0.5)

#qParticlesTest = qRobotEstStartPose; 
# initialize weights for particles
particleWeights = np.ones(nParticles) / nParticles

# draw initial particle position and orientation


# *function test
#out = getParticleSensorValue(qParticlesTest, objPos)

for i in range(1,NumOfSamples):
    
    # get current speed of robot according to DC values    
    curSpeed = robotExternalKinematics((20,30), qRobotRealPose[2,0])
    
    # compute new robot pose
    qRobotRealPose = robotComputeNewPose(qRobotRealPose, curSpeed, Ts)
    
    # compute new particles pose (prediction step)
    qParticles = computeNewParticlesPose(qParticles, curSpeed, Ts)
    
    # corection step    
    # get simulated robot sensor value
    is_detected, detObjInd, objPosRobotLocal = getSimulatedRobotSensorValue(qRobotRealPose, objPos)  
    
    if is_detected:
        
        # get sensor values of particles
        particlesSensorVal = getParticleSensorValue(qParticles, objPos)        
        
        # compute innovation        
        innovMat = computeInnovation(detObjInd, objPosRobotLocal, particlesSensorVal)
        
        # compute particle weights
        particlesWeights = computeParticleWeights(innovMat, computeCovarianceMat(sensorVariance))
        
        # select new generation of particles
        selParticleIndexes = selectNewGeneration(particlesWeights)
        qParticles = qParticles[:,selParticleIndexes]

    # compute estimated robot position from particles
    qRobotEstPose = computeEstRobotPoseFromParticles(qParticles, particlesWeights)    
    
    plt.cla() # clear figure
    
    # draw objects
    drawRectangle(fig1, objPos[:,0], 40, 20, iCol="red")
    drawRectangle(fig1, objPos[:,1], 40, 20, iCol="green")
    drawRectangle(fig1, objPos[:,2], 40, 20, iCol="blue")
    
    # draw current robot position
    drawRobot(fig1, qRobotRealPose)
    
    # draw estimated robot position
    drawRobot(fig1, qRobotEstPose)
    
    # draw current particle positions
    drawParticles(qParticles, particleWeights)
    plt.pause(0.05)
    
    # TODO: get real motor DC values from robot
 