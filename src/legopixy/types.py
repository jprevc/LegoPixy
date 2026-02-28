# -*- coding: utf-8 -*-
"""
Type aliases for common array shapes used in the particle filter and kinematics APIs.

These aliases improve API readability by documenting expected array layouts
without runtime shape enforcement.
"""

from __future__ import annotations

from typing import TypeAlias

import numpy as np

# Single pose (x, y, phi) in mm, mm, deg. Shape: (3,) or (3, 1).
PoseArray: TypeAlias = np.ndarray

# Robot local-frame speed (vx, vy, w) in m/s, m/s, rad/s. Shape: (3,) or (3, 1).
RobotSpeedArray: TypeAlias = np.ndarray

# Particle pose matrix with columns (x, y, phi). Shape: (3, num_particles).
ParticlesArray: TypeAlias = np.ndarray

# Object position matrix with columns (x, y) in global frame. Shape: (2, num_objects).
ObjectPosMatrix: TypeAlias = np.ndarray

# Predicted object positions per particle. Shape: (2, num_objects, num_particles).
ParticleObjectPosMatrix: TypeAlias = np.ndarray

# Innovation vectors per particle. Shape: (2, num_particles).
InnovationMatrix: TypeAlias = np.ndarray

# Particle weight vector. Shape: (num_particles,) or (num_particles, 1).
ParticleWeightsArray: TypeAlias = np.ndarray

# Measurement covariance matrix. Shape: (2, 2).
CovarianceMatrix: TypeAlias = np.ndarray
