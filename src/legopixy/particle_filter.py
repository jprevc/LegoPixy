# -*- coding: utf-8 -*-
"""
Created on Mon Jan  2 17:36:33 2017

@author: jostp
"""

from __future__ import annotations

import numpy as np

from legopixy.constants import (
    PARTICLE_WEIGHT_FLOOR,
    RAD_TO_DEG,
    SIMULATED_CAM_MAX_ANGLE,
    SIMULATED_CAM_MAX_DISTANCE,
)
from legopixy.lego_kinematics import robot_compute_new_pose, rot_z
from legopixy.types import (
    CovarianceMatrix,
    InnovationMatrix,
    ObjectPosMatrix,
    ParticleObjectPosMatrix,
    ParticlesArray,
    ParticleWeightsArray,
    PoseArray,
    RobotSpeedArray,
)


def initialize_particles(
    num_particles: int,
    particle_pose: PoseArray,
    particles_deviation: np.ndarray,
) -> ParticlesArray:
    """
    Initialize particles in a certain position and deviation around that position.

    :param num_particles: Number of particles to initialize.
    :param particle_pose: Initial particle pose ``(x, y, phi)`` in ``mm, mm, deg``.
    :param particles_deviation: Position and angle deviation around initial pose.
    :returns: Matrix of particle poses with shape ``(3, num_particles)``.
    """

    particles_pose_matrix = particle_pose + np.dot(
        np.diag(
            np.array(
                [particles_deviation[0], particles_deviation[0], particles_deviation[1]]
            )
        ),
        np.random.rand(3, num_particles) - 0.5,
    )

    return particles_pose_matrix


def compute_new_particles_pose(
    particles_pose_matrix: ParticlesArray,
    robot_speed: RobotSpeedArray,
    sample_time: float,
) -> ParticlesArray:
    """
    Predict new positions for all particles.

    This function represents the particle filter prediction step.

    :param particles_pose_matrix: Input particle pose matrix with columns ``(x, y, phi)``.
    :param robot_speed: Robot local-frame speed ``(vx, vy, w)`` in ``m/s, m/s, rad/s``.
    :param sample_time: Sample time in seconds.
    :returns: Predicted particle pose matrix with columns ``(x, y, phi)``.
    """

    # TODO : add random noise to motion

    particles_matrix = np.zeros_like(particles_pose_matrix)

    num_of_particles = particles_pose_matrix.shape[1]

    for particle_index in range(num_of_particles):

        # use robot_compute_new_pose function for computing new particle pose
        particles_matrix[:, particle_index] = robot_compute_new_pose(
            particles_pose_matrix[:, particle_index], robot_speed, sample_time
        )[:, 0]

    return particles_matrix


def get_particle_sensor_value(
    particles_pose_matrix: ParticlesArray, object_pos_matrix: ObjectPosMatrix
) -> ParticleObjectPosMatrix:
    """
    Compute object positions as observed from each particle pose.

    :param particles_pose_matrix: Particle pose matrix with columns ``(x, y, phi)`` in global frame.
    :param object_pos_matrix: Object position matrix with columns ``(x, y)`` in global frame.
    :returns: Tensor with shape ``(2, num_objects, num_particles)`` in particle-local frames.
    """

    # get number of objects and number of particles
    num_of_particles = particles_pose_matrix.shape[1]
    num_of_objects = object_pos_matrix.shape[1]

    # initialize output matrix
    particle_object_pos_matrix = np.zeros((2, num_of_objects, num_of_particles))

    for particle_index in range(num_of_particles):

        # get current particle orientation
        part_angle = particles_pose_matrix[2, particle_index]

        for object_index in range(num_of_objects):

            # get current object's position as would be measured by current particle
            particle_object_pos_matrix[:, object_index, particle_index] = np.dot(
                rot_z(part_angle).transpose(),
                object_pos_matrix[:, object_index]
                - particles_pose_matrix[:2, particle_index],
            )

    return particle_object_pos_matrix


def compute_innovation(
    measured_object_index: int,
    robot_sensor_measurement: np.ndarray,
    particle_object_pos_matrix: ParticleObjectPosMatrix,
) -> InnovationMatrix:
    """
    Computes innovation for each particle.

    :param measured_object_index: Index of the detected object.
    :param robot_sensor_measurement: Measured object position in robot camera coordinates.
    :param particle_object_pos_matrix: Predicted object positions for each particle.
    :returns: Innovation matrix with one column per particle.
    """
    num_of_particles = particle_object_pos_matrix.shape[2]

    # get all particles measurements for only the one detected object
    detected_object_particle_pos_matrix = particle_object_pos_matrix[
        :, measured_object_index, :
    ]

    return (
        np.repeat(robot_sensor_measurement.reshape((2, 1)), num_of_particles, axis=1)
        - detected_object_particle_pos_matrix
    )


def compute_particle_weights(
    innovation_matrix: InnovationMatrix, covariance_matrix: CovarianceMatrix
) -> ParticleWeightsArray:
    """
    Compute particle weights from innovation vectors.

    Reference: AVTONOMNI MOBILNI SISTEMI, Gregor Klancar, section 9.5.

    :param innovation_matrix: Innovation matrix with one column per particle.
    :param covariance_matrix: Measurement covariance matrix.
    :returns: Particle weight vector with one row per particle.
    """

    # TODO: uporabljena enačba v tej funkciji je drugacna kot v ucbeniku, je pa taka uporabljena tudi v Matlab primeru. Ugotovi, ce je prava

    num_of_particles = innovation_matrix.shape[1]

    particle_weights = np.zeros((num_of_particles, 1))

    for particle_index in range(num_of_particles):
        innovation_vector = np.array(innovation_matrix[:, particle_index], ndmin=2)
        innov_rr_innov = np.dot(
            np.dot(innovation_vector, np.linalg.inv(covariance_matrix)),
            innovation_vector.transpose(),
        )
        # particle_weights[particle_index] = det_cov_mat**(-0.5) * np.exp(-0.5 * innov_rr_innov) + 0.001
        # particle_weights[particle_index] = np.exp(-0.5 * innov_rr_innov) + 0.0001
        particle_weights[particle_index] = 1 / innov_rr_innov + PARTICLE_WEIGHT_FLOOR

    return particle_weights


def compute_covariance_matrix(sensor_variance: float) -> CovarianceMatrix:
    """
    Build the measurement covariance matrix.

    :param sensor_variance: Sensor distance-measurement variance in millimeters.
    :returns: Diagonal 2x2 covariance matrix.
    """

    sensor_variance_matrix = np.diag(np.array([sensor_variance, sensor_variance]))

    # covariance_matrix = np.diag(np.repeat(np.array(sensor_variance, ndmin=2), [1, 2]))
    covariance_matrix = sensor_variance_matrix
    return covariance_matrix


def select_new_generation(particle_weights: ParticleWeightsArray) -> np.ndarray:
    """
    Sample particle indices for the next generation.

    :param particle_weights: Particle weight vector.
    :returns: Selected particle indices for resampling.
    """

    num_of_particles = particle_weights.shape[0]

    cdf = np.cumsum(particle_weights) / np.sum(particle_weights)
    random_selection_indices = np.random.rand(num_of_particles)

    # prepend 0 to array CDF
    # cdf_with_zero = np.insert(cdf, 0, 0)
    cdf_with_zero = np.insert(cdf, 0, 0)

    index_grid = np.insert(np.arange(num_of_particles), 0, 0)

    next_generation_float = np.interp(
        random_selection_indices, cdf_with_zero, index_grid
    )

    next_generation_indices = np.ceil(next_generation_float).astype("int")

    #    plt.figure(2)
    #    plt.clf()
    #    plt.plot(CDFg[:-1], indg)
    #    plt.pause(0.05)

    return next_generation_indices


def get_simulated_robot_sensor_value(
    robot_pose: PoseArray,
    object_positions: ObjectPosMatrix,
    max_cam_angle: float = SIMULATED_CAM_MAX_ANGLE,
    max_distance: float = SIMULATED_CAM_MAX_DISTANCE,
) -> tuple[bool, int, np.ndarray]:
    """
    Simulate camera measurement from robot pose and known object positions.

    If an object is in view, returns detection state and object coordinates.

    :param robot_pose: Robot pose ``(x, y, phi)`` in ``mm, mm, deg``.
    :param object_positions: Object position matrix with columns ``(x, y)`` in millimeters.
    :param max_cam_angle: Maximum detection angle in degrees.
    :param max_distance: Maximum detection distance in millimeters.
    :returns: Tuple ``(detected, object_index, object_position_local)``.
    """
    # use get_particle_sensor_value to compute all locations of objects in robot coordinates
    object_positions_local = get_particle_sensor_value(robot_pose, object_positions)[
        :, :, 0
    ]

    num_of_objects = object_positions.shape[1]

    # for all objects check if any of them is in view
    for object_index in range(num_of_objects):

        x_obj = object_positions_local[0, object_index]
        y_obj = object_positions_local[1, object_index]

        distance = np.sqrt(x_obj**2 + y_obj**2)
        angle = np.arctan2(y_obj, x_obj) * RAD_TO_DEG

        if distance <= max_distance and np.abs(angle) <= max_cam_angle:
            return (True, object_index, np.array([x_obj, y_obj], ndmin=2).transpose())

    return (False, 0, np.zeros((2, 1)))


def compute_est_robot_pose_from_particles(
    particles_pose_matrix: ParticlesArray, particle_weights: ParticleWeightsArray
) -> PoseArray:
    """
    Estimate robot pose from particle set and weights.

    :param particles_pose_matrix: Particle pose matrix with columns ``(x, y, phi)``.
    :param particle_weights: Particle weights.
    :returns: Estimated robot pose ``(x, y, phi)`` in ``mm, mm, deg``.
    """
    estimated_robot_pose = np.zeros((3, 1))

    particle_position_matrix = particles_pose_matrix[:2, :]
    particle_angles = particles_pose_matrix[2, :]

    # for estimated position we take mean of particle positions
    estimated_robot_pose[:2, 0] = np.mean(particle_position_matrix, axis=1)

    # for orientation estimation we take angle of the particle with the highest weight
    estimated_robot_pose[2, 0] = particle_angles[np.argmax(particle_weights)]

    return estimated_robot_pose
