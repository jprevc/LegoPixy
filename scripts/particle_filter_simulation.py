# -*- coding: utf-8 -*-
"""
Created on Sat Jan  7 13:33:07 2017

@author: jostp
"""

import matplotlib.pyplot as plt
import numpy as np

from legopixy.lego_kinematics import (
    robot_compute_new_pose,
    robot_external_kinematics,
)
from legopixy.lego_sim_functions import draw_particles, draw_rectangle, draw_robot
from legopixy.particle_filter import (
    compute_covariance_matrix,
    compute_est_robot_pose_from_particles,
    compute_innovation,
    compute_new_particles_pose,
    compute_particle_weights,
    get_particle_sensor_value,
    get_simulated_robot_sensor_value,
    initialize_particles,
    select_new_generation,
)

ts = 0.1  # sample time in s
t_fin = 30  # Time of simulation

# sensor distance variance
sensor_variance = 0.5

# actuator variance noise
actuator_var_noise = np.diag(np.array([1, 1, 0.3])) * ts

t_vec = np.arange(0, t_fin, ts)
num_of_samples = len(t_vec)

# object positions
obj_pos = np.array([[-200, -200], [200, 600], [600, 200]]).transpose()

# draw environment
fig1 = plt.figure(1)

# draw objects (landmark rectangles)
obj_rect_width = 40
obj_rect_height = 20
draw_rectangle(fig1, obj_pos[:, 0], obj_rect_width, obj_rect_height, color="red")
draw_rectangle(fig1, obj_pos[:, 1], obj_rect_width, obj_rect_height, color="green")
draw_rectangle(fig1, obj_pos[:, 2], obj_rect_width, obj_rect_height, color="blue")

# simulated robot starting pose
q_robot_real_pose = np.array([200, -100, -10], ndmin=2).transpose()

# commanded robot speed as DC values for left and right wheels
robot_wheel_speed = (20, 40)

# estimated robot start pose
q_robot_est_pose = np.array([0, 0, 0], ndmin=2).transpose()

# initialize particles for particle filter
n_particles = 600
particle_pos_deviation = (
    400  # particle position deviation from robot's initial estimated position in mm
)
particle_orient_deviation = 90  # particle orientation deviation from robot's initial estimated orientation in degrees
part_dev = np.array([particle_pos_deviation, particle_orient_deviation])

# initialize particles
q_particles = initialize_particles(n_particles, q_robot_est_pose, part_dev)

# initialize weights for particles
particles_weights = np.ones(n_particles) / n_particles

# convergence threshold, if standard deviation of particle position is below this value, particles are considered converged
convergence_threshold = 1e-5

innov_threshold = 150

for _ in range(1, num_of_samples):

    # get current speed of robot according to DC values
    cur_speed_real = robot_external_kinematics(
        robot_wheel_speed, q_robot_real_pose[2, 0]
    )

    # get speed of estimated robot
    cur_speed_est = robot_external_kinematics(robot_wheel_speed, q_robot_est_pose[2, 0])

    # compute new robot pose
    q_robot_real_pose = robot_compute_new_pose(q_robot_real_pose, cur_speed_real, ts)

    # compute new particles pose (prediction step)
    q_particles = compute_new_particles_pose(
        q_particles,
        np.dot(actuator_var_noise, np.random.rand(3, 1)) + cur_speed_est,
        ts,
    )

    # correction step
    # get simulated robot sensor value
    is_detected, det_obj_ind, obj_pos_robot_local = get_simulated_robot_sensor_value(
        q_robot_real_pose, obj_pos
    )

    if is_detected:

        # get sensor values of particles
        particles_sensor_val = get_particle_sensor_value(q_particles, obj_pos)

        # compute innovation
        innov_mat = compute_innovation(
            det_obj_ind, obj_pos_robot_local, particles_sensor_val
        )

        # compute particle weights
        particles_weights = compute_particle_weights(
            innov_mat, compute_covariance_matrix(sensor_variance)
        )

        print(np.std(particles_weights))
        print(np.linalg.norm(np.mean(innov_mat, 1)))

        # find out if particles are converged
        is_converged = np.std(particles_weights) < convergence_threshold
        if is_converged:
            # reinitialize particles if innovation has become too big
            if np.linalg.norm(np.mean(innov_mat, 1)) > innov_threshold:
                q_particles = initialize_particles(
                    n_particles, q_robot_est_pose, part_dev
                )

        # select new generation of particles
        sel_particle_indexes = select_new_generation(particles_weights)
        q_particles = q_particles[:, sel_particle_indexes]

    # compute estimated robot position from particles
    q_robot_est_pose = compute_est_robot_pose_from_particles(
        q_particles, particles_weights
    )

    plt.cla()  # clear figure

    # draw objects
    draw_rectangle(fig1, obj_pos[:, 0], obj_rect_width, obj_rect_height, color="red")
    draw_rectangle(fig1, obj_pos[:, 1], obj_rect_width, obj_rect_height, color="green")
    draw_rectangle(fig1, obj_pos[:, 2], obj_rect_width, obj_rect_height, color="blue")

    # draw current robot position
    draw_robot(fig1, q_robot_real_pose)

    # draw estimated robot position
    draw_robot(fig1, q_robot_est_pose)

    # draw current particle positions
    draw_particles(q_particles, particles_weights)

    plt.xlim(-500, 1000)
    plt.ylim(-500, 1000)

    plt.pause(0.05)
