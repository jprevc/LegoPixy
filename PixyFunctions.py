# -*- coding: utf-8 -*-
"""
Created on Sun Jan  1 17:34:26 2017

@author: jostp
"""

import numpy as np

def computeObjectPosition(iObjectWidthPx, iObjectWidth, iObjectXcoordPx, iFocalLength = 215, iCamMaxAngle = 38.65):
    '''
    Returns detected object's position in camera coordinate system, according to his width, and its center's x coordinate
    
    Parameters
    -----------
    iObjectWidthPx : int
        Object's width in pixels
    iObjectWidth : float
        Object's width in mm     
    iObjectXcoordPx : int
        Object's x coordinate on camera in pixels
    iFocalLength : float
        Focal length of the camera in mm
    iCamMaxAngle : float
        Maximum camera vision angle in degrees
        
    Returns
    ----------
    np.array 
        Computed postion (x,y) of object in camera coordinate system
        
    '''
    
    # compute object's distance from camera
    distance = iObjectWidth * iFocalLength / iObjectWidthPx
    
    # compute object's angle to camera    
    koef = -iCamMaxAngle / 128.0
    objectAngle = koef * (iObjectXcoordPx - 128)
    
    # compute object's coordinates
    oX = distance * np.cos(objectAngle * np.pi / 180)
    oY = distance * np.sin(objectAngle * np.pi / 180)
    
    return np.array((oX, oY))
    
    