import os
import tempfile
import time
import unittest
from unittest import mock

from modules.thrusters import uart_provider
from modules.thrusters.uart_provider import UartThrusterProvider, find_serial_port


class FakePortInfo:
    def __init__(self, device, vid=None, pid=None):
        self.device = device
        self.vid = vid
        self.pid = pid


class FindSerialPortTests(unittest.TestCase):
    def test_existing_configured_port_wins(self):
        with tempfile.NamedTemporaryFile() as handle:
            found = find_serial_port(handle.name)
        self.assertEqual(found, handle.name)

    def test_missing_port_falls_back_to_matching_usb_serial_id(self):
        found = find_serial_port("/dev/definitely-not-here")
        self.assertIsNotNone(found)
        self.assertTrue(found.startswith("/dev/"))

    def test_missing_and_no_devices_returns_configured_value(self):
        with mock.patch.object(uart_provider, "list_ports", None):
            with mock.patch.object(uart_provider, "_by_id_links", list):
                with mock.patch(
                    "modules.thrusters.uart_provider.glob.glob",
                    mock.Mock(return_value=[]),
                ):
                    self.assertEqual(find_serial_port("/dev/nope"), "/dev/nope")

    def test_known_usb_serial_id_is_preferred_over_generic_glob(self):
        with tempfile.TemporaryDirectory() as folder:
            ch340 = os.path.join(folder, "ttyUSB0")
            other = os.path.join(folder, "ttyUSB1")
            for path in (ch340, other):
                open(path, "w").close()
            infos = [
                FakePortInfo(other, vid=0x1234, pid=0x5678),
                FakePortInfo(ch340, vid=0x1A86, pid=0x7523),
            ]
            with mock.patch.object(uart_provider, "list_ports", mock.Mock(comports=lambda: infos)):
                with mock.patch.object(uart_provider, "_by_id_links", list):
                    self.assertEqual(find_serial_port("/dev/nope"), ch340)

    def test_resolve_port_updates_provider_port(self):
        with tempfile.TemporaryDirectory() as folder:
            device = os.path.join(folder, "ttyUSB0")
            open(device, "w").close()
            provider = UartThrusterProvider({"port": "/dev/nope"})
            with mock.patch.object(
                uart_provider, "find_serial_port", return_value=device
            ) as finder:
                self.assertEqual(provider.resolve_port(), device)
            self.assertEqual(provider.port, device)
            self.assertTrue(finder.called)


class ReconnectTests(unittest.TestCase):
    def test_open_re_resolves_port_before_connecting(self):
        with tempfile.TemporaryDirectory() as folder:
            device = os.path.join(folder, "ttyUSB0")
            open(device, "w").close()
            provider = UartThrusterProvider({"port": "/dev/nope", "baud": 115200})
            fake = mock.Mock()
            with mock.patch.object(uart_provider, "find_serial_port", return_value=device):
                with mock.patch.object(uart_provider, "serial", fake):
                    with mock.patch("modules.thrusters.uart_provider.time.sleep"):
                        self.assertTrue(provider.open())
            self.assertEqual(provider.port, device)
            self.assertEqual(fake.Serial.call_args[0], (device, 115200))
            provider.close()

    def test_failed_open_leaves_provider_not_ok(self):
        provider = UartThrusterProvider({"port": "/dev/definitely-not-here"})
        with mock.patch.object(uart_provider, "find_serial_port", return_value="/dev/definitely-not-here"):
            with mock.patch.object(
                uart_provider, "serial", mock.Mock(Serial=mock.Mock(side_effect=OSError("boom")))
            ):
                self.assertFalse(provider.open())
        self.assertFalse(provider.ok)
        self.assertEqual(provider._open_error, "boom")

    def test_write_failure_closes_the_handle(self):
        provider = UartThrusterProvider({})
        handle = mock.Mock()
        handle.write.side_effect = OSError("cable pulled")
        provider._ser = handle
        self.assertFalse(provider._write_frame(b"\x00"))
        self.assertIsNone(provider._ser)
        self.assertTrue(handle.close.called)
        self.assertEqual(provider._open_error, "cable pulled")

    def test_not_settled_immediately_after_open(self):
        provider = UartThrusterProvider({})
        handle = mock.Mock()
        handle.in_waiting = 0
        with mock.patch.object(uart_provider, "find_serial_port", return_value="/dev/null"):
            with mock.patch.object(uart_provider, "serial", mock.Mock(Serial=mock.Mock(return_value=handle))):
                with mock.patch("modules.thrusters.uart_provider.time.sleep"):
                    self.assertTrue(provider.open())
        self.assertTrue(provider.ok)
        self.assertFalse(provider.settled)
        self.assertFalse(provider.responding)
        provider._opened_at -= uart_provider.RX_TIMEOUT_S + 0.01
        self.assertTrue(provider.settled)
        provider.close()
        self.assertFalse(provider.settled)

    def test_responding_window_uses_shared_timeout(self):
        provider = UartThrusterProvider({})
        provider._last_rx_time = time.monotonic()
        self.assertTrue(provider.responding)
        provider._last_rx_time -= uart_provider.RX_TIMEOUT_S + 0.01
        self.assertFalse(provider.responding)


if __name__ == "__main__":
    unittest.main()
