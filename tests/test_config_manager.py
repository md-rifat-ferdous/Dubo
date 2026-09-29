import os
import tempfile
import unittest

import yaml

from modules.config.config_manager import ConfigManager


class ConfigManagerTests(unittest.TestCase):
    def test_update_section_writes_valid_yaml(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "thrusters.yaml")
            with open(path, "w") as stream:
                yaml.safe_dump({"thrusters": {"port": "/dev/ttyUSB0"}}, stream)
            manager = ConfigManager(directory)
            manager.initialize()
            manager.update_section("thrusters", {"port": "/dev/ttyACM0", "baud": 57600})
            with open(path) as stream:
                data = yaml.safe_load(stream)
            self.assertEqual(data["thrusters"]["port"], "/dev/ttyACM0")
            self.assertEqual(manager.get("thrusters", "baud"), 57600)


if __name__ == "__main__":
    unittest.main()
