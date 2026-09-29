import unittest

from modules.controller.controller import MotionState
from modules.thrusters.mixer import (
    DEFAULT_MIXING,
    load_thruster_config,
    mix_motion,
    normalize_polarity,
)


class MixerTests(unittest.TestCase):
    def test_surge_mix_uses_shared_layout(self):
        values = mix_motion(MotionState(surge=1.0))
        self.assertEqual(values["M2"], 1.0)
        self.assertEqual(values["M3"], 1.0)
        self.assertEqual(values["M1"], 0.0)

    def test_polarity_is_applied_after_mixing(self):
        values = mix_motion(
            MotionState(heave=1.0),
            polarity={"M1": -1},
        )
        self.assertEqual(values["M1"], -1.0)

    def test_boost_scale_limits_output(self):
        values = mix_motion(
            MotionState(surge=1.0, boost=True),
            boost_scale=0.5,
        )
        self.assertEqual(values["M2"], 0.5)
        self.assertEqual(values["M3"], 0.5)

    def test_invalid_polarity_is_rejected(self):
        with self.assertRaises(ValueError):
            normalize_polarity({"M1": 0})

    def test_config_contains_all_five_motors(self):
        config = load_thruster_config()
        self.assertEqual(set(config["mixing"]), set(DEFAULT_MIXING))
        self.assertEqual(config["port"], "/dev/ttyUSB0")
        self.assertEqual(config["deadband"]["negative"], 176)


if __name__ == "__main__":
    unittest.main()
