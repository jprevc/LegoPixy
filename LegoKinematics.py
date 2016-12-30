# -*- coding: utf-8 -*-
"""
Created on Fri Dec 30 17:00:36 2016

@author: jostp
"""
    
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl    
    
def largeMotorDC2RPM(iDCval):
    '''
    Compute RPM value of large motor from DC value.
    This function uses a model, acquired from website http://www.philohome.com/motors/motorcomp.htm
    
    Parameters
    ----------
    iDCval : float 
        DC value, applied to motor, in %
        
    Returns
    ---------
    oRPM : float 
        Speed of motor in RPM
        
    
    '''
    
    # koeficient between RPM and applied motor voltage, in RPM/V
    koef = 230.0 / 12;
    
    # maximum voltage, which can be applied, this should be at DC = 100%
    maxVolt = 12;
    
    oRPM = iDCval / 100.0 * maxVolt * koef
    
    return oRPM
    
def robotInternalKinematics(iDCvals, iWheelRadius = 0.034, iWheelDistance = 0.135):
    '''
    Returns robot translational (tangent) speed and rotational speed (rotation around ICR).
    Reference: AVTONOMNI MOBILNI SISTEMI, Gregor Klancar, section 3.2.1
    
    Parameters
    ----------
    iDCvals        : tuple, 2 values
        DC values on left and right wheels
    iWheelRadius   : float
        Wheel radius in m
    iWheelDistance : float
        Distance between wheels in m
        
    Returns
    ---------
    oInternalSpeed : tuple (oVx, oVy, oW)
        Translational (tangent) speed in m/s and rotational speed in rad/s of robot in local coordinate system.
        Note: for diferential drive, oVy should always be zero.
                    
    '''
     
    A = np.array([[iWheelRadius/2, iWheelRadius/2], [0, 0], [-iWheelRadius / iWheelDistance, iWheelRadius / iWheelDistance]])
    w_wheels = np.array([(largeMotorDC2RPM(dc_val) * 2 * np.pi / 60) for dc_val in iDCvals]).transpose()
    
    oInternalSpeed = np.dot(A, w_wheels)
    
    return oInternalSpeed
    
def robotExternalKinematics(iDCvals, iPhi, iWheelRadius = 0.034, iWheelDistance = 0.135):
    '''
    Returns robot tranlational (tangent) speed and rotational speed (rotation around ICR) in external coordinate system.
    Reference: AVTONOMNI MOBILNI SISTEMI, Gregor Klancar, section 3.2.1
    
    Parameters
    ----------
    
    iDCvals : tuple, (iDCleft, iDCright)
        DC values on left and right wheels
    iPhi : float 
        Current orientation of the robot, in degrees
    iWheelRadius   : float
        Wheel radius in m
    iWheelDistance : float 
        Distance between wheels in m
        
    Returns
    ----------
    
    (oVx, oVy, oW) : tuple 
        Translational (tangent) speed in m/s and rotational speed in rad/s of robot in local coordinate system.
                         Note: for diferential drive, oVy should always be zero.
                         
    '''
    
    (vx_local, vy_local, w_local) = robotInternalKinematics(iDCvals, iWheelRadius, iWheelDistance)
    v_abs = np.sqrt(vx_local**2 + vy_local**2)
    w_abs = w_local
   
    A = np.array([[np.cos(iPhi * np.pi / 180), 0], [np.sin(iPhi * np.pi / 180), 0], [0, 1]])
   
    robotAbsSpeed = np.array([v_abs, w_abs]).transpose()
   
    oExternalSpeed = np.dot(A, robotAbsSpeed)
   
    return oExternalSpeed

def robotComputeNewPose(iCurPose, iRobotSpeed, iSampleTime):
    '''
    Computes new robot pose, using curent pose and current speed of the robot. This function uses Euler integration method.
    Reference: AVTONOMNI MOBILNI SISTEMI, Gregor Klancar, section 3.2.1
    
    Paramters
    ---------
    
    iCurPose    : np.array, 3×1 
        Curent pose of the robot (px,py,phi) in mm and deg
    iRobotSpeed : np.array, 3×1 
        Curent speed of the robot (vx,vy,w) in m/s and rad/s
    iSampleTime : float 
        Sample time in s

    Returns
    ---------
    
    oNewPose : np.array, 3×1
        New robot pose (px_new,py_new,phi_new) in mm and deg   
        
    '''
    
    # compute absolute robot speed
    v_robotAbs = np.sqrt(iRobotSpeed[0]**2 + iRobotSpeed[1]**2)
    
    # compute new pose
    px_new = iCurPose[0] + v_robotAbs * iSampleTime * np.cos(iCurPose[2] * np.pi / 180) * 1000
    py_new = iCurPose[1] + v_robotAbs * iSampleTime * np.sin(iCurPose[2] * np.pi / 180) * 1000
    phi_new = iCurPose[2] + (iRobotSpeed[2] * iSampleTime) * 180 / np.pi
    
    oNewPose = np.array([px_new,py_new,phi_new])
    
    return oNewPose
