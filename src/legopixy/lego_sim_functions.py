# -*- coding: utf-8 -*-
"""
Created on Fri Dec 30 17:01:20 2016

@author: jostp
"""

import matplotlib as mpl
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

from legopixy.constants import DEG_TO_RAD

# Robot drawing defaults (mm)
ROBOT_LENGTH = 170
ROBOT_WIDTH = 110
WHEEL_LENGTH = 50
WHEEL_WIDTH = 35
DISTANCE_BETWEEN_WHEELS = 135
CAM_LENGTH = 30
CAM_WIDTH = 20


def draw_rectangle(fig, origin, width, height, rotation=0, color="blue", alpha=0.5):
    """
    Draws a rectangle on the figure.

    :param fig: Figure handle where the rectangle is drawn.
    :param origin: Rectangle center ``(x, y)`` in plot coordinates.
    :param width: Rectangle width.
    :param height: Rectangle height.
    :param rotation: Rotation in degrees around rectangle center.
    :param color: Rectangle color.
    :param alpha: Rectangle transparency in the ``[0, 1]`` range.
    """
    origin = np.asarray(origin).flatten()
    origin_x, origin_y = float(origin[0]), float(origin[1])
    rotation = float(rotation)

    ax = fig.add_subplot(111)

    rect = patches.Rectangle(
        (origin_x - width / 2.0, origin_y - height / 2.0),
        width,
        height,
        color=color,
        alpha=alpha,
    )
    trans = (
        mpl.transforms.Affine2D()
        .translate(-origin_x, -origin_y)
        .rotate_deg(rotation)
        .translate(origin_x, origin_y)
        + ax.transData
    )

    rect.set_transform(trans)

    ax.add_patch(rect)


def draw_robot(
    fig,
    robot_pose,
    robot_length=ROBOT_LENGTH,
    robot_width=ROBOT_WIDTH,
    wheel_length=WHEEL_LENGTH,
    wheel_width=WHEEL_WIDTH,
    distance_between_wheels=DISTANCE_BETWEEN_WHEELS,
    cam_length=CAM_LENGTH,
    cam_width=CAM_WIDTH,
):
    """
    Draws a differential drive robot on figure.

    :param fig: Figure handle where the robot is drawn.
    :param robot_pose: Robot pose ``(x, y, phi)`` in ``mm, mm, deg``.
    :param robot_length: Robot body length.
    :param robot_width: Robot body width.
    :param wheel_length: Wheel length.
    :param wheel_width: Wheel width.
    :param distance_between_wheels: Distance between wheel centers.
    :param cam_length: Camera marker length.
    :param cam_width: Camera marker width.
    """
    robot_pose = np.asarray(robot_pose).flatten()
    robot_position = robot_pose[:2]
    robot_orientation = float(robot_pose[2])

    # compute right and left wheels positions
    phi_rad = robot_orientation * DEG_TO_RAD
    right_wheel_position = robot_position + (distance_between_wheels / 2.0) * np.array(
        [np.sin(phi_rad), -np.cos(phi_rad)]
    )
    left_wheel_position = robot_position - (distance_between_wheels / 2.0) * np.array(
        [np.sin(phi_rad), -np.cos(phi_rad)]
    )

    # compute camera position
    camera_position = robot_position + (robot_length / 2) * np.array(
        [np.cos(phi_rad), np.sin(phi_rad)]
    )

    # draw robot rectangle
    draw_rectangle(fig, robot_position, robot_length, robot_width, robot_orientation)

    # draw wheel rectangles
    draw_rectangle(
        fig,
        right_wheel_position,
        wheel_length,
        wheel_width,
        robot_orientation,
        color="black",
    )
    draw_rectangle(
        fig,
        left_wheel_position,
        wheel_length,
        wheel_width,
        robot_orientation,
        color="black",
    )

    # draw camera rectangle
    draw_rectangle(
        fig, camera_position, cam_length, cam_width, robot_orientation, color="black"
    )


def draw_particles(particles_pose_matrix, particle_weights):
    """
    Draws particles as arrows to current axis.

    :param particles_pose_matrix: Particle pose matrix with columns ``(x, y, phi)``.
    :param particle_weights: Particle weights used to color arrows.
    """

    plt.quiver(
        particles_pose_matrix[0, :],
        particles_pose_matrix[1, :],
        np.cos(particles_pose_matrix[2, :] * DEG_TO_RAD),
        np.sin(particles_pose_matrix[2, :] * DEG_TO_RAD),
        particle_weights,
    )
