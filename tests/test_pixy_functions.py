"""Lightweight tests for pixy_functions module."""

import numpy as np

from legopixy.constants import PIXY_CAMERA_CENTER_PX
from legopixy.pixy_functions import compute_object_position


class TestComputeObjectPosition:
    """Tests for compute_object_position: pixel width + x-coord -> camera-frame (x, y)."""

    def test_object_at_center(self):
        """Object at x=128 (center) has angle=0, so x=distance, y=0."""
        # distance = objectWidth * focalLength / objectWidthPx = 100 * 215 / 100 = 215 mm
        result = compute_object_position(
            object_width_px=100,
            object_width=100.0,
            object_x_coord_px=128,
        )
        np.testing.assert_array_almost_equal(result, [215.0, 0.0])

    def test_object_left_of_center(self):
        """Object left of center (x<center) has positive angle, oY>0."""
        result = compute_object_position(
            object_width_px=100,
            object_width=100.0,
            object_x_coord_px=0,
        )
        # objectAngle = -38.65/128 * (0-128) = 38.65°
        # oX = 215 * cos(38.65°), oY = 215 * sin(38.65°)
        assert result[0] > 0
        assert result[1] > 0

    def test_object_right_of_center(self):
        """Object right of center (x>128) has negative angle, oY<0."""
        result = compute_object_position(
            object_width_px=100,
            object_width=100.0,
            object_x_coord_px=256,
        )
        # objectAngle = -38.65/128 * (256-128) = -38.65°
        assert result[0] > 0
        assert result[1] < 0

    def test_distance_scales_with_object_width(self):
        """Larger real object at same pixel width -> larger distance."""
        near = compute_object_position(100, 50.0, PIXY_CAMERA_CENTER_PX)
        far = compute_object_position(100, 200.0, PIXY_CAMERA_CENTER_PX)
        assert far[0] > near[0]
        np.testing.assert_almost_equal(near[1], 0.0)
        np.testing.assert_almost_equal(far[1], 0.0)

    def test_distance_scales_inversely_with_pixel_width(self):
        """Same real object, smaller pixel width -> larger distance (closer)."""
        near = compute_object_position(200, 100.0, 128)
        far = compute_object_position(50, 100.0, 128)
        assert far[0] > near[0]

    def test_custom_focal_length(self):
        """Custom focal length changes distance."""
        default = compute_object_position(
            100, 100.0, PIXY_CAMERA_CENTER_PX, focal_length=215
        )
        custom = compute_object_position(
            100, 100.0, PIXY_CAMERA_CENTER_PX, focal_length=430
        )
        np.testing.assert_almost_equal(custom[0], 2 * default[0])
        np.testing.assert_almost_equal(custom[1], 0.0)
