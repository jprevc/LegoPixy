# -*- coding: utf-8 -*-
"""
Named constants for legopixy modules and scripts.

Replaces magic numbers to improve readability and maintainability.
"""

from __future__ import annotations

import numpy as np

# -----------------------------------------------------------------------------
# Angle conversion (degree <-> radian)
# -----------------------------------------------------------------------------
DEG_TO_RAD: float = np.pi / 180
RAD_TO_DEG: float = 180 / np.pi

# -----------------------------------------------------------------------------
# Pixy camera
# -----------------------------------------------------------------------------
PIXY_CAMERA_CENTER_PX: int = 128  # Half of 256 px image width
PIXY_CAMERA_DEFAULT_FOCAL_LENGTH: float = 215.0  # mm
PIXY_CAMERA_DEFAULT_MAX_ANGLE: float = 38.65  # degrees, horizontal FOV

# -----------------------------------------------------------------------------
# Pixy signature drawing (detected object rectangle in simulation)
# -----------------------------------------------------------------------------
PIXY_SIGNATURE_WIDTH: int = 64
PIXY_SIGNATURE_HEIGHT: int = 32
PIXY_SIGNATURE_ROTATION: int = 90  # degrees

# -----------------------------------------------------------------------------
# Kinematics and motors
# -----------------------------------------------------------------------------
MM_PER_M: float = 1000.0
SECONDS_PER_MINUTE: float = 60.0
LARGE_MOTOR_RPM_PER_VOLT: float = 230.0 / 12
LARGE_MOTOR_MAX_VOLT: float = 12.0

# -----------------------------------------------------------------------------
# Particle filter
# -----------------------------------------------------------------------------
PARTICLE_WEIGHT_FLOOR: float = 0.0001
SIMULATED_CAM_MAX_ANGLE: float = 38.0  # degrees, for get_simulated_robot_sensor_value
SIMULATED_CAM_MAX_DISTANCE: float = 500.0  # mm

# -----------------------------------------------------------------------------
# Communication protocol
# -----------------------------------------------------------------------------
ASCII_DOLLAR: int = 36  # Delimiter byte in Pixy data stream
