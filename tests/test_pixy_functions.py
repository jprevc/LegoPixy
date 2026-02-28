"""Lightweight tests for pixy_functions module."""

import numpy as np

from legopixy.pixy_functions import computeObjectPosition


class TestComputeObjectPosition:
    """Tests for computeObjectPosition: pixel width + x-coord -> camera-frame (x, y)."""

    def test_object_at_center(self):
        """Object at x=128 (center) has angle=0, so oX=distance, oY=0."""
        # distance = objectWidth * focalLength / objectWidthPx = 100 * 215 / 100 = 215 mm
        result = computeObjectPosition(
            iObjectWidthPx=100,
            iObjectWidth=100.0,
            iObjectXcoordPx=128,
        )
        np.testing.assert_array_almost_equal(result, [215.0, 0.0])

    def test_object_left_of_center(self):
        """Object left of center (x<128) has positive angle, oY>0."""
        result = computeObjectPosition(
            iObjectWidthPx=100,
            iObjectWidth=100.0,
            iObjectXcoordPx=0,
        )
        # objectAngle = -38.65/128 * (0-128) = 38.65°
        # oX = 215 * cos(38.65°), oY = 215 * sin(38.65°)
        assert result[0] > 0
        assert result[1] > 0

    def test_object_right_of_center(self):
        """Object right of center (x>128) has negative angle, oY<0."""
        result = computeObjectPosition(
            iObjectWidthPx=100,
            iObjectWidth=100.0,
            iObjectXcoordPx=256,
        )
        # objectAngle = -38.65/128 * (256-128) = -38.65°
        assert result[0] > 0
        assert result[1] < 0

    def test_distance_scales_with_object_width(self):
        """Larger real object at same pixel width -> larger distance."""
        near = computeObjectPosition(100, 50.0, 128)
        far = computeObjectPosition(100, 200.0, 128)
        assert far[0] > near[0]
        np.testing.assert_almost_equal(near[1], 0.0)
        np.testing.assert_almost_equal(far[1], 0.0)

    def test_distance_scales_inversely_with_pixel_width(self):
        """Same real object, smaller pixel width -> larger distance (closer)."""
        near = computeObjectPosition(200, 100.0, 128)
        far = computeObjectPosition(50, 100.0, 128)
        assert far[0] > near[0]

    def test_custom_focal_length(self):
        """Custom focal length changes distance."""
        default = computeObjectPosition(100, 100.0, 128, iFocalLength=215)
        custom = computeObjectPosition(100, 100.0, 128, iFocalLength=430)
        np.testing.assert_almost_equal(custom[0], 2 * default[0])
        np.testing.assert_almost_equal(custom[1], 0.0)
