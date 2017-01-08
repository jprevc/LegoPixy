# -*- coding: utf-8 -*-
"""
Created on Mon Jan  2 17:36:33 2017

@author: jostp
"""

import numpy as np
import matplotlib.pyplot as plt
from LegoKinematics import *
from LegoSimFunctions import *

def computeNewParticlesPose(iParticlesPoseMat, iRobotSpeed, iSampleTime):
    '''
    Returns new predicted positions for all particles. This function represents prediction step of the particle filter.
    
    Parameters
    ----------
    iParticlesPoseMat : np.ndarray
        Input matrix 3×n , where n is number of particles. Each column is (xp,yp,phi) for particle in global coordinates
    iRobotSpeed : np.array, 3×1 
        Curent speed of the robot (vx,vy,w) in m/s and rad/s
    iSampleTime : float 
        Sample time in s
        
    Returns
    -----------
    oParticlesPoseMat : np.ndarray
        Input matrix 3×n , where n is number of particles. Each column is (xp,yp,phi) for particle in global coordinates
    '''
    
    # TODO : add random noise to motion    
        
    oParticlesMat = np.zeros_like(iParticlesPoseMat)
    
    numOfParticles = iParticlesPoseMat.shape[1]
    
    for particleInd in range(numOfParticles):
        
        # use robotComputeNewPose function for computing new particle pose
        oParticlesMat[:,particleInd] = robotComputeNewPose(iParticlesPoseMat[:,particleInd], iRobotSpeed, iSampleTime)[:,0]
        
    return oParticlesMat
    
def getParticleSensorValue(iParticlesPoseMat, iObjectPosMat):
    '''
    Returns positions of all objects, as would be measured if the camera was in the pose of coresponding particle.
    
    Parameters
    ----------
    iParticlesPoseMat : np.ndarray
        Input matrix 3×Np , where n is number of particles. 
        Each column is (xp,yp,phi_p) for particle pose in global coordinates
        phi_p parameter is in degrees
    iObjectPosMat : np.ndarray
        Input matrix 2×No, where No is the number of placed objects. 
        Each column is (xo,yo) for object position in global coordinates.
        
    Returns
    ----------
    np.ndarray
        3D matrix 2 × No × Np, where No is number of objects, and Np is number of particles, 
        vector [:,Oi,Pi] represents position of object Oi in particle Pi's local coordinate system.
    
    '''
    
    # get number of objects and number of particles
    numOfParticles = iParticlesPoseMat.shape[1]
    numOfObjects = iObjectPosMat.shape[1]
    
    # initialize output matrix    
    oParticleObjectPosMat = np.zeros((2,numOfObjects,numOfParticles))
    
    for particleInd in range(numOfParticles):
        
        # get current particle orientation
        partAngle = iParticlesPoseMat[2,particleInd]
        
        for objectInd in range(numOfObjects):
            
            # get current object's position as would be measured by current particle
            oParticleObjectPosMat[:,objectInd,particleInd] = np.dot(rotZ(partAngle).transpose(), iObjectPosMat[:,objectInd] - iParticlesPoseMat[:2, particleInd] )
            
    return oParticleObjectPosMat
        
        
def computeInnovation(iMeasuredObjectInd, iRobotSensorMeas, iParticleObjectPosMat):
    '''
    Computes innovation for each particle.
    
    Parameters
    ----------
    iMeasuredObjectInd : int
        Index of detected object
    iRobotSensorMeas : np.ndarray
        Vector 2×1 with robot's measured position of detected object in robot's camera coordinate system
    iParticleObjectPosMat : np.ndarray
        3D matrix 2×No×Np, where No is number of objects, and Np is number of particles, 
        vector [:,Oi,Pi] represents position of object Oi in particle Pi's local coordinate system.
        
    Returns
    ----------
    np.ndarray
        Matrix 2×Np, where Np is number of particles. Each column represents computed innovation for each particle.
    '''
    numOfParticles = iParticleObjectPosMat.shape[2]
    
    # get all particles measurements for only the one detected object
    detectedObjectParticlePosMat = iParticleObjectPosMat[:,iMeasuredObjectInd,:]
    
    return np.repeat(iRobotSensorMeas.reshape((2,1)), numOfParticles, axis=1) - detectedObjectParticlePosMat
    
def computeParticleWeights(iInnovationMat, iCovarianceMat):
    '''
    Computes particle weights
    Reference: AVTONOMNI MOBILNI SISTEMI, Gregor Klancar, section 9.5
    
    Parameters
    ----------
    iInnovationMat : np.ndarray
        Matrix 2×Np, where Np is number of particles. Each column represents computed innovation for each particle.
        
    iCovarianceMat : np.ndarray
        2×2 covariance matrix
        
    Returns
    ----------
    np.ndarray
        Matrix Np×1 of weigths, where Np is number of particles
    '''
    
    # TODO: uporabljena enačba v tej funkciji je drugacna kot v ucbeniku, je pa taka uporabljena tudi v Matlab primeru. Ugotovi, ce je prava   
    
    numOfParticles = iInnovationMat.shape[1]
    
    oParticleWeigths = np.zeros((numOfParticles, 1))
    
    for particleInd in range(numOfParticles):
        detCovMat = np.linalg.det(2*np.pi*iCovarianceMat)
        innovMat = np.array(iInnovationMat[:, particleInd], ndmin=2)
        InnovRRInnov = np.dot(np.dot(innovMat, np.linalg.inv(iCovarianceMat)), innovMat.transpose())
        #oParticleWeigths[particleInd] = detCovMat**(-0.5) * np.exp(-0.5 * InnovRRInnov) + 0.001
        #oParticleWeigths[particleInd] = np.exp(-0.5 * InnovRRInnov) + 0.0001
        oParticleWeigths[particleInd] = 1/InnovRRInnov + 0.0001
        
    return oParticleWeigths

def computeCovarianceMat(iSensorVariance):
    '''
    Returns covariance matrix of measurement
    
    Parameters
    ---------
    iSensorVariance : float
        Sensor's measurement variance of distance in mm
        
    Returns
    ---------
    np.ndarray
        2×2 covariance matrix
    
    '''
    
    sensorVarianceMat = np.diag(np.array([iSensorVariance, iSensorVariance]))    
    
    #oCovMat = np.diag(np.repeat(np.array(iSensorVariance,ndmin=2), [1, 2]))
    oCovMat = sensorVarianceMat
    return oCovMat
    
def selectNewGeneration(iParticleWeights):
    '''
    Returns indexes of selected particles for new generation
    
    Parameters
    -----------
    iParticleWeights : np.ndarray
        Matrix Np×1, where Np is number of particles
        
    Returns
    ----------
    np.ndarray
        Matrix of selected particle indexes Np×1, where Np is number of particles
    '''
    
    numOfParticles = iParticleWeights.shape[0]
    
    CDF = np.cumsum(iParticleWeights) / np.sum(iParticleWeights)
    indSelectRand = np.random.rand(numOfParticles)
    
    # prepend 0 to array CDF
    #CDFg = np.insert(CDF, 0, 0)
    CDFg = np.insert(CDF, 0, 0)
    
    indg = np.insert(np.arange(numOfParticles),0,0)
    
    indNextGen_float = np.interp(indSelectRand, CDFg, indg)
    
    indNextGen = np.ceil(indNextGen_float).astype('int') 
    
#    plt.figure(2)
#    plt.clf()
#    plt.plot(CDFg[:-1], indg)
#    plt.pause(0.05)
    
    return indNextGen
    
def getSimulatedRobotSensorValue(iRobotPose, iObjPos, iMaxCamAngle=38, iMaxDistance = 500):
    '''
    Get simulated robot sensor value. If ovject is in view, function returns True and coordinates of the object, 
    if no object is in view, function returns False.
    
    Parameter
    ---------
    iRobotPose : np.ndarray
        Matrix 3×1 of robot pose (xr,yr,phi_r) in mm and deg
    iObjPos : np.ndarray
        Matrix 2×No, where No is positions of the objects
    iMaxCamAngle : float
        Maximum angle, at which robot is still able to see the object in deg
    iMaxDistance : float
        Maximum distance, at which robot is still able to detect object in mm
        
    Returns
    ---------
    tuple of three elements
        1st element : bool
            True if object is detected, False if not
        2nd element : int
            Index of detected object
        3rd element : np.ndarray
            Matrix 2×1 (xo, yo), where xo and yo is position of detected object in mm
    '''
    # use getParticleSensorValue to compute all loactions of objects simulated robot's coordinate system
    objPosLocal = getParticleSensorValue(iRobotPose, iObjPos)[:,:,0]
    
    numOfObjects = iObjPos.shape[1]
    
    # for all objects check if any of them is in view
    for objInd in range(numOfObjects):
        
        xo = objPosLocal[0,objInd]
        yo = objPosLocal[1,objInd]
        
        dist = np.sqrt(xo**2 + yo**2)
        angle = np.arctan2(yo, xo) * 180 / np.pi   
        
        if dist <= iMaxDistance and np.abs(angle) <= iMaxCamAngle:
            return (True, objInd, np.array([xo, yo], ndmin=2).transpose())
            
    return (False, 0, np.zeros((2,1)))
    

def computeEstRobotPoseFromParticles(iParticlesPoseMat, iParticleWeigths):
    '''
    Computes estimated robot position from particles pose
    
    Parameters
    -----------
    iParticlesPoseMat : np.ndarray
        Input matrix 3×Np , where n is number of particles. 
        Each column is (xp,yp,phi_p) for particle pose in global coordinates
        phi_p parameter is in degrees
    iParticleWeights
        Weigths of the particles
    Returns
    -----------
    np.ndarray
        Matrix 3×1 of estimated robot pose (xe,ye,phi_e) in mm and deg
    '''
    oEstRobotPose = np.zeros((3,1))    
    
    particlePositionMat = iParticlesPoseMat[:2,:]
    particleAnglesMat = iParticlesPoseMat[2,:]
    
    # for estimated position we take mean of particle positions
    oEstRobotPose[:2,0] = np.mean(particlePositionMat, axis=1)
    
    # for orientation estimation we take angle of the particle with the highest weight
    oEstRobotPose[2,0] = particleAnglesMat[np.argmax(iParticleWeigths)]
    
    return oEstRobotPose