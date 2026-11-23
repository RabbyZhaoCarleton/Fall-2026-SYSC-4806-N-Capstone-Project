"""Unit tests for usb_attach (no WSL, usbipd or hardware needed: everything is mocked).

Run:  python3 -m unittest discover -s test -v      (from the lidar_bringup package folder)
"""
import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lidar_bringup import usb_attach as ua  # noqa: E402

# Verbatim output of `usbipd list` from the team's laptop (BUSID 1-1 is the LiDAR).
SAMPLE = """Connected:
BUSID  VID:PID    DEVICE                                                        STATE
1-1    10c4:ea60  Silicon Labs CP210x USB to UART Bridge (COM5)                 Shared
1-3    04f2:b828  Integrated Camera, Camera DFU Device                          Not shared
1-4    048d:c195  USB Input Device                                              Not shared
1-5    0bda:4853  Realtek Bluetooth Adapter                                     Not shared

Persisted:
GUID                                  DEVICE
"""


class ParseTests(unittest.TestCase):
    def test_parses_all_connected_rows(self):
        devs = ua.parse_usbipd_list(SAMPLE)
        self.assertEqual([d.busid for d in devs], ['1-1', '1-3', '1-4', '1-5'])
        self.assertEqual(devs[0].vid_pid, '10c4:ea60')
        self.assertEqual(devs[0].state, 'Shared')
        self.assertEqual(devs[1].state, 'Not shared')
        self.assertIn('COM5', devs[0].description)

    def test_ignores_persisted_table(self):
        text = SAMPLE + "aaaa-bbbb  1-9    10c4:ea60  Ghost device   Not shared\n"
        self.assertEqual(len(ua.parse_usbipd_list(text)), 4)

    def test_attached_and_crlf(self):
        text = SAMPLE.replace('Shared\n', 'Attached\n', 1).replace('\n', '\r\n')
        devs = ua.parse_usbipd_list(ua._decode(text.encode('utf-16')))
        self.assertTrue(devs[0].is_attached and devs[0].is_shared)


class SelectTests(unittest.TestCase):
    def setUp(self):
        self.devs = ua.parse_usbipd_list(SAMPLE)

    def test_selects_by_vid_pid_even_if_busid_changes(self):
        moved = SAMPLE.replace('1-1 ', '3-2 ')
        self.assertEqual(ua.select_lidar(ua.parse_usbipd_list(moved)).busid, '3-2')

    def test_no_match(self):
        with self.assertRaises(ua.LidarPortError):
            ua.select_lidar(self.devs, vid_pid='dead:beef')

    def test_ambiguous_refuses_to_guess(self):
        two = self.devs + [ua.UsbDevice('1-7', '10c4:ea60', 'CP210x (ESP32)', 'Not shared')]
        with self.assertRaises(ua.LidarPortError) as ctx:
            ua.select_lidar(two)
        self.assertIn('lidar_busid', str(ctx.exception))
        self.assertEqual(ua.select_lidar(two, busid='1-7').busid, '1-7')
        self.assertEqual(ua.select_lidar(two, description_contains='COM5').busid, '1-1')

    def test_bad_busid_rejected(self):
        with self.assertRaises(ua.LidarPortError):
            ua._check_busid("1-1'; rm -rf /")


class FlowTests(unittest.TestCase):
    """ensure_lidar_port on a fake WSL: records which usbipd commands are issued."""

    def run_flow(self, list_text, port_exists=lambda p: False, run_rc=0, **kw):
        calls = []

        def fake_run(cmd, timeout=30.0):
            calls.append(cmd[1:3] if 'usbipd' in cmd[0] else cmd[:1])
            return (0, list_text) if cmd[1] == 'list' else (run_rc, '')

        with mock.patch.object(ua, 'is_wsl', return_value=True), \
             mock.patch.object(ua, 'find_usbipd', return_value='usbipd.exe'), \
             mock.patch.object(ua, '_run', side_effect=fake_run), \
             mock.patch.object(ua.os.path, 'exists', side_effect=port_exists), \
             mock.patch.object(ua, 'wait_for_port', return_value='/dev/rplidar'):
            return ua.ensure_lidar_port(log=lambda m: None, **kw), calls

    def test_shared_device_is_just_attached(self):
        port, calls = self.run_flow(SAMPLE)
        self.assertEqual(port, '/dev/rplidar')
        self.assertIn(['attach', '--wsl'], calls)
        self.assertNotIn(['bind', '--busid'], calls)

    def test_unshared_device_is_bound_then_attached(self):
        text = SAMPLE.replace('Shared\n', 'Not shared\n', 1)
        _, calls = self.run_flow(text)
        self.assertLess(calls.index(['bind', '--busid']), calls.index(['attach', '--wsl']))

    def test_already_present_port_skips_everything(self):
        port, calls = self.run_flow(SAMPLE, port_exists=lambda p: True)
        self.assertEqual((port, calls), ('/dev/rplidar', []))

    def test_already_attached_not_reattached(self):
        _, calls = self.run_flow(SAMPLE.replace('Shared\n', 'Attached\n', 1))
        self.assertNotIn(['attach', '--wsl'], calls)

    def test_native_linux_only_waits(self):
        with mock.patch.object(ua, 'is_wsl', return_value=False), \
             mock.patch.object(ua, 'wait_for_port', return_value='/dev/rplidar'), \
             mock.patch.object(ua, '_run') as run:
            self.assertEqual(ua.ensure_lidar_port(log=lambda m: None), '/dev/rplidar')
            run.assert_not_called()


if __name__ == '__main__':
    unittest.main()
