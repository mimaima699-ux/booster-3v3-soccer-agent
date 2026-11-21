"""Round-trip tests for tactics.geometry coordinate transforms."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.soccer_framework import Pose2D  # noqa: E402
from src.tactics.geometry import (  # noqa: E402
    field_to_relative,
    relative_to_field,
)


class GeometryTests(unittest.TestCase):
    def test_ball_coordinate_round_trip(self):
        pose = Pose2D(1.0, 2.0, math.pi / 4.0)
        relative = field_to_relative(0.5, 0.0, pose)
        recovered = relative_to_field(relative.x, relative.y, pose)
        self.assertAlmostEqual(recovered.x, 0.5, places=6)
        self.assertAlmostEqual(recovered.y, 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
