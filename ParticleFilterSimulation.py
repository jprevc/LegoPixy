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
Tfin = 30 # Time of simulation

# sensor distance variance
sensorVariance = 0.5

# actuator variance noise
actuatorVarNoise = np.diag(np.array([1,1,0.3]))*Ts

t_vec = np.arange(0, Tfin, Ts)
NumOfSamples = len(t_vec)

# object positions
objPos = np.array([[-200,-200], [200,600], [600,200]]).transpose()

# draw environment
fig1 = plt.figure(1)

# draw objects
drawRectangle(fig1, objPos[:,0], 40, 20, iCol="red")
drawRectangle(fig1, objPos[:,1], 40, 20, iCol="green")
drawRectangle(fig1, objPos[:,2], 40, 20, iCol="blue")

# simulated robot starting pose
qRobotRealPose = np.array([200, -100, -10], ndmin=2).transpose()

# commanded robot speed as DC values for left and right wheels
robotWheelSpeed = (40,20)
#drawRobot(fig1, qRobotRealPose)

# estimated robot start pose
qRobotEstPose = np.array([0, 0, 0], ndmin=2).transpose()

# initialize particles for particle filter
nParticles = 600
particlePosDeviation = 400    # particle position deviation from robot's initial estimated position in mm
particleOrientDeviation = 90 # particle orientation deviation from robot's initial estimated orientation in degrees
partDev = np.array([particlePosDeviation, particleOrientDeviation])

# initialize particles
qParticles = initializeParticles(nParticles, qRobotEstPose, partDev)

# initialize weights for particles
particlesWeights = np.ones(nParticles) / nParticles

# convergence threshold, if standard deviation of particle position is below this value, particles are considered converged
convergenceThreshold = 1e-5

innovThreshold = 150

# *function test
#out = getParticleSensorValue(qParticlesTest, objPos)

for i in range(1,NumOfSamples):
    
    # get current speed of robot according to DC values    
    curSpeedReal = robotExternalKinematics(robotWheelSpeed, qRobotRealPose[2,0])
    
    # get speed of estimated robot
    curSpeedEst = robotExternalKinematics(robotWheelSpeed, qRobotEstPose[2,0])
    
    # compute new robot pose
    qRobotRealPose = robotComputeNewPose(qRobotRealPose, curSpeedReal, Ts)
    
    # compute new particles pose (prediction step)
    qParticles = computeNewParticlesPose(qParticles, np.dot(actuatorVarNoise, np.random.rand(3,1)) + curSpeedEst, Ts)
    
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
        
        print(np.std(particlesWeights))
        print(np.linalg.norm(np.mean(innovMat, 1)))
        
        # find out if particles are converged        
        is_converged = np.std(particlesWeights) < convergenceThreshold
        if is_converged:
            # reinitialize particles if innovation has become too big
            if np.linalg.norm(np.mean(innovMat, 1)) > innovThreshold:
                qParticles = initializeParticles(nParticles, qRobotEstPose, partDev)
            
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
    drawParticles(qParticles, particlesWeights)
    
    plt.xlim(-500, 1000)
    plt.ylim(-500, 1000)

    plt.pause(0.05)

 