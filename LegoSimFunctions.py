# -*- coding: utf-8 -*-
"""
Created on Fri Dec 30 17:01:20 2016

@author: jostp
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl

def drawRectangle(iFig, iOrigin, iWidth, iHeight, iRot = 0, iCol = "blue", iAlpha = 0.5):
    '''
    Draws a rectangle on the figure.
    
    Parameters
    ----------
    iFig    : figure 
        Figure handle
    iOrigin : tuple 
        Location of the center of the rectangle
    iWidth  : float 
        Width of the rectangle
    iHeight : float 
        Height of the rectangle
    iRot    : float 
        Rotation of the rectangle in degrees (note: rectangle is rotated around the rectangle's center)
    iCol    : string 
        Color of the rectalngle
    iAlpha  : float 
        Transparency of the rectangle. Accepted values are 0 - 1
    
    '''
    
    ax = iFig.add_subplot(111)
    
    rect = patches.Rectangle((iOrigin[0] - iWidth/2.0, iOrigin[1] - iHeight/2.0), iWidth, iHeight, color=iCol, alpha=iAlpha)
    trans = mpl.transforms.Affine2D().translate(-iOrigin[0], -iOrigin[1]).rotate_deg(iRot).translate(iOrigin[0], iOrigin[1]) + ax.transData
    
    rect.set_transform(trans)
    
    ax.add_patch(rect)
    
    
def drawRobot(iFig, iRobotPose, iRobotLength=170, iRobotWidth=110, iWheelLength=50, iWheelWidth=35, iDistBetweenWheels=135, iCamLength=30, iCamWidth=20 ):
    '''
    Draws a diferential drive robot on figure.
    
    Parameters
    ---------
    iFig : figure
        Figure handle
    iRobotPose : np.array 3×1
        Current robot position px,py in mm and orientation in degrees
    iRobotLength : float
        Length of the robot
    iRobotWidth : float
        Width of the robot
    iWheelLength : float
        Length of the wheel
    iWheelWidth : float
        Width of the wheel
    iDistBetweenWheels : float
        Distance between centers of the wheels
        
    '''
    robotPosition = iRobotPose[:2]
    robotOrientation = iRobotPose[2]
    
    # compute right and left wheels positions
    rightWheelPosition = robotPosition + (iDistBetweenWheels/2.0) * np.array([np.sin(robotOrientation * np.pi / 180), -np.cos(robotOrientation * np.pi / 180)])
    leftWheelPosition = robotPosition - (iDistBetweenWheels/2.0) * np.array([np.sin(robotOrientation * np.pi / 180), -np.cos(robotOrientation * np.pi / 180)])
    
    # compute camera position    
    cameralPosition = robotPosition + (iRobotLength / 2) * np.array([np.cos(robotOrientation * np.pi / 180), np.sin(robotOrientation * np.pi / 180)])
    
    # draw robot rectangle
    drawRectangle(iFig, robotPosition, iRobotLength, iRobotWidth, robotOrientation)
    
    # draw wheel rectangles
    drawRectangle(iFig, rightWheelPosition, iWheelLength, iWheelWidth, robotOrientation, iCol = "black")
    drawRectangle(iFig, leftWheelPosition, iWheelLength, iWheelWidth, robotOrientation, iCol = "black")

    # draw camera rectangle
    drawRectangle(iFig, cameralPosition, iCamLength, iCamWidth, robotOrientation, iCol = "black")

    
    