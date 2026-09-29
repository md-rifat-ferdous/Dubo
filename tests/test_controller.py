import unittest

from modules.controller.controller import ControllerModule, MotionState
from modules.telemetry.telemetry_manager import SimulatedLink, TelemetryManager


class TelemetryCommandTests(unittest.TestCase):
    def test_inject_command_is_forwarded_to_the_link(self):
        telemetry = TelemetryManager()
        link = SimulatedLink()
        telemetry.link = link

        self.assertTrue(telemetry.inject_command("FORWARD 0.5"))
        self.assertEqual(link.receive(), ["FORWARD 0.5"])


class FakeThrusters:
    def __init__(self):
        self.ready = True
        self.motion = MotionState()
        self.direct = None
        self.estop = False
        self.failsafe = False

    def can_arm(self):
        return self.ready

    def set_motion(self, motion):
        self.motion = motion

    def set_test_values(self, values):
        self.direct = dict(values)

    def clear_test_values(self):
        self.direct = None

    def zero(self):
        self.motion = MotionState()
        self.direct = None

    def emergency_stop(self):
        self.estop = True
        self.motion = MotionState()

    def recover(self):
        return not self.estop

    def status(self):
        return {
            "ready": self.ready,
            "failsafe": self.failsafe,
            "estop": self.estop,
        }

    def health_check(self):
        return self.ready


class ControllerTests(unittest.TestCase):
    def make_controller(self):
        thrusters = FakeThrusters()
        controller = ControllerModule(thrusters=thrusters)
        controller.start()
        return controller, thrusters

    def test_starts_disarmed_and_requires_ready_thrusters(self):
        controller, thrusters = self.make_controller()
        self.assertFalse(controller.armed)
        thrusters.ready = False
        self.assertFalse(controller.arm())
        self.assertFalse(controller.armed)

    def test_source_merge_uses_largest_axis_command(self):
        controller, thrusters = self.make_controller()
        self.assertTrue(controller.arm())
        controller.set_input("pad", MotionState(surge=0.25))
        controller.set_input("keyboard", MotionState(surge=-0.75))
        controller.update()
        self.assertEqual(thrusters.motion.surge, -0.75)
        self.assertEqual(controller.last_motion.surge, -0.75)

    def test_dashboard_sources_are_merged_independently(self):
        controller, thrusters = self.make_controller()
        controller.arm()
        controller.set_input("pad", MotionState(surge=0.25))
        controller.set_input("full_motion", MotionState(heave=0.5))
        controller.update()
        self.assertEqual(thrusters.motion.surge, 0.25)
        self.assertEqual(thrusters.motion.heave, 0.5)
        controller.set_input("full_motion", MotionState())
        controller.update()
        self.assertEqual(thrusters.motion.surge, 0.25)
        self.assertEqual(thrusters.motion.heave, 0.0)

    def test_disarm_clears_all_motion_and_test_override(self):
        controller, thrusters = self.make_controller()
        controller.arm()
        controller.set_test_motor_values({"M1": 0.5})
        controller.disarm("test")
        self.assertFalse(controller.armed)
        self.assertIsNone(thrusters.direct)
        self.assertEqual(thrusters.motion, MotionState())

    def test_expired_test_motion_is_cleared(self):
        controller, thrusters = self.make_controller()
        controller.arm()
        controller.set_test_motion(MotionState(surge=0.5), duration=1.0)
        controller._test_until = 0.0
        controller.update()
        self.assertEqual(thrusters.motion, MotionState())

    def test_estop_latches_and_rejects_recovery(self):
        controller, thrusters = self.make_controller()
        controller.arm()
        controller.emergency_stop()
        self.assertTrue(controller.estop_latched)
        self.assertTrue(thrusters.estop)
        self.assertFalse(controller.recover())
        self.assertFalse(controller.arm())

    def test_initial_failsafe_has_a_short_arm_grace(self):
        controller, thrusters = self.make_controller()
        thrusters.failsafe = True
        self.assertTrue(controller.arm())
        controller.update()
        self.assertTrue(controller.armed)
        controller._failsafe_grace_until = 0.0
        controller.update()
        self.assertFalse(controller.armed)

    def test_estop_clears_after_provider_reports_power_cycle(self):
        controller, thrusters = self.make_controller()
        controller.arm()
        controller.emergency_stop()
        thrusters.estop = False
        controller.update()
        self.assertTrue(controller.estop_latched)
        thrusters.estop = True
        controller.update()
        thrusters.estop = False
        controller.update()
        self.assertFalse(controller.estop_latched)
        self.assertFalse(controller.armed)

    def test_motion_test_clears_a_previous_direct_override(self):
        controller, thrusters = self.make_controller()
        controller.arm()
        self.assertTrue(controller.set_test_motor_values({"M1": 0.5}))
        self.assertIsNotNone(thrusters.direct)
        self.assertTrue(controller.set_test_motion(MotionState(surge=0.5)))
        self.assertIsNone(thrusters.direct)


if __name__ == "__main__":
    unittest.main()
