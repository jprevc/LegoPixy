# -*- coding: utf-8 -*-
"""
Created on Fri Dec 30 17:00:36 2016

@author: jostp
"""

from __future__ import annotations

import numpy as np

from legopixy.constants import (
    DEG_TO_RAD,
    LARGE_MOTOR_MAX_VOLT,
    LARGE_MOTOR_RPM_PER_VOLT,
    MM_PER_M,
    RAD_TO_DEG,
    SECONDS_PER_MINUTE,
)
from legopixy.types import PoseArray, RobotSpeedArray


def large_motor_dc_to_rpm(dc_value: float) -> float:
    """
    Compute large motor speed from duty cycle.

    This model is based on data from http://www.philohome.com/motors/motorcomp.htm.

    :param dc_value: Duty-cycle command applied to the motor, in percent.
    :returns: Motor speed in revolutions per minute.
    """

    rpm = dc_value / 100.0 * LARGE_MOTOR_MAX_VOLT * LARGE_MOTOR_RPM_PER_VOLT

    return rpm


def robot_internal_kinematics(
    dc_values: tuple[float, float],
    wheel_radius: float = 0.034,
    wheel_distance: float = 0.135,
) -> np.ndarray:
    """
    Compute differential-drive local-frame velocity from wheel duty cycles.

    Reference: AVTONOMNI MOBILNI SISTEMI, Gregor Klancar, section 3.2.1.

    :param dc_values: Duty-cycle values for left and right wheels, in percent.
    :param wheel_radius: Wheel radius in meters.
    :param wheel_distance: Distance between wheels in meters.
    :returns: Robot local velocity vector ``(vx, vy, w)`` in ``m/s, m/s, rad/s``.
    """

    coefficient_matrix = np.array(
        [
            [wheel_radius / 2, wheel_radius / 2],
            [0, 0],
            [-wheel_radius / wheel_distance, wheel_radius / wheel_distance],
        ]
    )
    w_wheels = np.array(
        [
            (large_motor_dc_to_rpm(dc_val) * 2 * np.pi / SECONDS_PER_MINUTE)
            for dc_val in dc_values
        ]
    ).transpose()

    internal_speed = np.dot(coefficient_matrix, w_wheels)

    return internal_speed


def robot_external_kinematics(
    dc_values: tuple[float, float],
    phi_angle: float,
    wheel_radius: float = 0.034,
    wheel_distance: float = 0.135,
) -> np.ndarray:
    """
    Compute differential-drive velocity in the global frame.

    Reference: AVTONOMNI MOBILNI SISTEMI, Gregor Klancar, section 3.2.1.

    :param dc_values: Duty-cycle values for left and right wheels, in percent.
    :param phi_angle: Current robot orientation in degrees.
    :param wheel_radius: Wheel radius in meters.
    :param wheel_distance: Distance between wheels in meters.
    :returns: Robot global velocity vector ``(vx, vy, w)`` in ``m/s, m/s, rad/s``.
    """

    vx_local, vy_local, w_local = robot_internal_kinematics(
        dc_values, wheel_radius, wheel_distance
    )
    v_abs = np.sqrt(vx_local**2 + vy_local**2)
    w_abs = w_local

    transform_matrix = np.array(
        [
            [np.cos(phi_angle * DEG_TO_RAD), 0],
            [np.sin(phi_angle * DEG_TO_RAD), 0],
            [0, 1],
        ]
    )

    robot_abs_speed = np.array([v_abs, w_abs], ndmin=2).transpose()

    external_speed = np.dot(transform_matrix, robot_abs_speed)

    return external_speed


def robot_compute_new_pose(
    current_pose: PoseArray, robot_speed: RobotSpeedArray, sample_time: float
) -> PoseArray:
    """
    Integrate robot motion over one sample to obtain a new pose.

    Euler integration is used.
    Reference: AVTONOMNI MOBILNI SISTEMI, Gregor Klancar, section 3.2.1.

    :param current_pose: Current robot pose ``(x, y, phi)`` in ``mm, mm, deg``.
    :param robot_speed: Robot local-frame speed ``(vx, vy, w)`` in ``m/s, m/s, rad/s``.
    :param sample_time: Sample time in seconds.
    :returns: New robot pose ``(x, y, phi)`` in ``mm, mm, deg``.
    """

    # compute absolute robot speed
    robot_abs_speed = np.sqrt(robot_speed[0] ** 2 + robot_speed[1] ** 2)

    # compute new pose
    px_new = (
        current_pose[0]
        + robot_abs_speed
        * sample_time
        * np.cos(current_pose[2] * DEG_TO_RAD)
        * MM_PER_M
    )
    py_new = (
        current_pose[1]
        + robot_abs_speed
        * sample_time
        * np.sin(current_pose[2] * DEG_TO_RAD)
        * MM_PER_M
    )
    phi_new = current_pose[2] + (robot_speed[2] * sample_time) * RAD_TO_DEG

    # check angle cycle
    phi_new = repair_angle_cycle(phi_new)

    new_pose = np.array([px_new, py_new, phi_new])

    return new_pose


def repair_angle_cycle(angle: float) -> float:
    """
    Wrap angle to the ``[-180, 180]`` interval.

    :param angle: Input angle in degrees.
    :returns: Wrapped angle in degrees.
    """

    return (
        np.arctan2(np.sin(angle * DEG_TO_RAD), np.cos(angle * DEG_TO_RAD)) * RAD_TO_DEG
    )


def rot_z(angle: float) -> np.ndarray:
    """
    Compute a 2x2 rotation matrix around the z axis.

    :param angle: Rotation angle in degrees.
    :returns: Rotation matrix.
    """

    angle = angle * DEG_TO_RAD

    matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])

    return matrix
