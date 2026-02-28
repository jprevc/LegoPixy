# -*- coding: utf-8 -*-
"""
Created on Sun Jan  1 17:34:26 2017

@author: jostp
"""

from __future__ import annotations

import numpy as np

from legopixy.constants import (
    DEG_TO_RAD,
    PIXY_CAMERA_CENTER_PX,
    PIXY_CAMERA_DEFAULT_FOCAL_LENGTH,
    PIXY_CAMERA_DEFAULT_MAX_ANGLE,
)


def compute_object_position(
    object_width_px: int,
    object_width: float,
    object_x_coord_px: int,
    focal_length: float = PIXY_CAMERA_DEFAULT_FOCAL_LENGTH,
    cam_max_angle: float = PIXY_CAMERA_DEFAULT_MAX_ANGLE,
) -> np.ndarray:
    """
    Compute detected object position in camera coordinates.

    :param object_width_px: Detected object width in pixels.
    :param object_width: Real object width in millimeters.
    :param object_x_coord_px: Detected x-coordinate of object center in pixels.
    :param focal_length: Camera focal length in millimeters.
    :param cam_max_angle: Maximum camera field-of-view angle in degrees.
    :returns: Object position ``(x, y)`` in the camera frame, in millimeters.
    """

    # compute object's distance from camera
    distance = object_width * focal_length / object_width_px

    # compute object's angle to camera
    koef = -cam_max_angle / PIXY_CAMERA_CENTER_PX
    object_angle = koef * (object_x_coord_px - PIXY_CAMERA_CENTER_PX)

    # compute object's coordinates
    x_coord = distance * np.cos(object_angle * DEG_TO_RAD)
    y_coord = distance * np.sin(object_angle * DEG_TO_RAD)

    return np.array((x_coord, y_coord))
