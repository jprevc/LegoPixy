"""Lightweight tests for lego_kinematics module."""

import numpy as np
import pytest

from legopixy.lego_kinematics import (
    largeMotorDC2RPM,
    repairAngleCycle,
    robotComputeNewPose,
    robotInternalKinematics,
    rotZ,
)


class TestLargeMotorDC2RPM:
    """Tests for largeMotorDC2RPM: DC value -> RPM mapping."""

    def test_zero_dc_gives_zero_rpm(self):
        assert largeMotorDC2RPM(0) == 0.0

    def test_full_dc_gives_230_rpm(self):
        """At DC=100%, model yields 230 RPM (koef=230/12, maxVolt=12)."""
        assert largeMotorDC2RPM(100) == 230.0

    def test_half_dc_gives_half_rpm(self):
        assert largeMotorDC2RPM(50) == 115.0

    def test_negative_dc_reverses_rpm(self):
        assert largeMotorDC2RPM(-50) == -115.0


class TestRobotInternalKinematics:
    """Tests for robotInternalKinematics: DC tuple -> (vx, vy, w)."""

    def test_symmetric_dc_gives_forward_motion_no_rotation(self):
        """Equal left/right DC yields vx>0, vy=0, w=0."""
        vx, vy, w = robotInternalKinematics((50, 50))
        np.testing.assert_almost_equal(vy, 0.0)
        np.testing.assert_almost_equal(w, 0.0)
        assert vx > 0

    def test_zero_dc_gives_zero_speed(self):
        vx, vy, w = robotInternalKinematics((0, 0))
        assert vx == 0.0
        assert vy == 0.0
        assert w == 0.0

    def test_opposite_dc_gives_rotation_only(self):
        """Left forward, right backward: vx≈0, w≠0."""
        vx, vy, w = robotInternalKinematics((50, -50))
        assert abs(vx) < 0.01
        assert vy == 0.0
        assert w != 0.0

    def test_custom_wheel_params(self):
        """Custom wheel radius and distance produce expected scaling."""
        v1, _, _ = robotInternalKinematics(
            (50, 50), iWheelRadius=0.034, iWheelDistance=0.135
        )
        v2, _, _ = robotInternalKinematics(
            (50, 50), iWheelRadius=0.068, iWheelDistance=0.135
        )
        np.testing.assert_almost_equal(v2, 2 * v1)


class TestRobotComputeNewPose:
    """Tests for robotComputeNewPose: pose + speed + dt -> new pose (Euler)."""

    def test_stationary_robot_unchanged(self):
        pose = np.array([100.0, 200.0, 45.0])
        speed = np.array([0.0, 0.0, 0.0])
        new = robotComputeNewPose(pose, speed, 1.0)
        np.testing.assert_array_almost_equal(new, pose)

    def test_forward_motion_at_zero_heading(self):
        """vx=1 m/s, dt=1s, heading=0° -> px increases by 1000 mm."""
        pose = np.array([0.0, 0.0, 0.0])
        speed = np.array([1.0, 0.0, 0.0])
        new = robotComputeNewPose(pose, speed, 1.0)
        np.testing.assert_almost_equal(new[0], 1000.0)
        np.testing.assert_almost_equal(new[1], 0.0)
        np.testing.assert_almost_equal(new[2], 0.0)

    def test_forward_motion_at_90_heading(self):
        """vx=1 m/s in local frame, heading=90° -> py increases by 1000 mm."""
        pose = np.array([0.0, 0.0, 90.0])
        speed = np.array([1.0, 0.0, 0.0])
        new = robotComputeNewPose(pose, speed, 1.0)
        np.testing.assert_almost_equal(new[0], 0.0)
        np.testing.assert_almost_equal(new[1], 1000.0)
        np.testing.assert_almost_equal(new[2], 90.0)

    def test_rotation_only_changes_phi(self):
        """w=π rad/s, dt=1s -> phi increases by 180°."""
        pose = np.array([0.0, 0.0, 0.0])
        speed = np.array([0.0, 0.0, np.pi])
        new = robotComputeNewPose(pose, speed, 1.0)
        np.testing.assert_almost_equal(new[0], 0.0)
        np.testing.assert_almost_equal(new[1], 0.0)
        np.testing.assert_almost_equal(new[2], 180.0)


class TestRepairAngleCycle:
    """Tests for repairAngleCycle: angle wrapping to [-180, 180]."""

    @pytest.mark.parametrize("angle", [0, 90, 180, -90])
    def test_angles_in_range_unchanged(self, angle):
        assert repairAngleCycle(angle) == angle

    def test_360_wraps_to_0(self):
        np.testing.assert_almost_equal(repairAngleCycle(360), 0.0)

    def test_270_equals_minus_90(self):
        np.testing.assert_almost_equal(repairAngleCycle(270), -90.0)

    def test_181_wraps_to_minus_179(self):
        np.testing.assert_almost_equal(repairAngleCycle(181), -179.0)

    def test_minus_181_wraps_to_179(self):
        np.testing.assert_almost_equal(repairAngleCycle(-181), 179.0)


class TestRotZ:
    """Tests for rotZ: 2×2 rotation matrix around z-axis."""

    def test_zero_degrees_identity(self):
        R = rotZ(0)
        np.testing.assert_array_almost_equal(R, np.eye(2))

    def test_90_degrees(self):
        R = rotZ(90)
        expected = np.array([[0, -1], [1, 0]])
        np.testing.assert_array_almost_equal(R, expected)

    def test_180_degrees(self):
        R = rotZ(180)
        expected = np.array([[-1, 0], [0, -1]])
        np.testing.assert_array_almost_equal(R, expected)

    def test_rotates_vector(self):
        """(1,0) rotated by 90° -> (0,1)."""
        v = np.array([1.0, 0.0])
        R = rotZ(90)
        result = R @ v
        np.testing.assert_array_almost_equal(result, [0, 1])
