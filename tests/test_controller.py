import asyncio
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from dbus_next import DBusError
from evdev import AbsInfo, InputEvent, ecodes as e

from apptirover.controller import (ADAPTER, DEVICE, HID_UUIDS, ControllerMonitor,
                                  PairingAgent, choose_device, is_lite2)
from apptirover.gamepad import InputState, matches_input, normalize, stick_axes


ADDRESS = 'AA:BB:CC:DD:EE:01'
PAD_PATH = '/org/bluez/hci0/dev_AA_BB_CC_DD_EE_01'
PAD = dict(Address=ADDRESS, Name='8BitDo Lite 2', Class=0x2508,
           Paired=False, Connected=False, Trusted=False)


class FakeInput:
    info = SimpleNamespace(bustype=e.BUS_BLUETOOTH)
    uniq = ADDRESS.lower()
    path = '/dev/input/event99'

    def __init__(self):
        self.values = {code: 128 for code in (e.ABS_X, e.ABS_Y, e.ABS_Z, e.ABS_RZ)}
        self.keys = []

    def capabilities(self, absinfo=True):
        axes = [(code, self.absinfo(code)) for code in self.values] if absinfo else list(self.values)
        return {e.EV_ABS: axes, e.EV_KEY: [e.BTN_SOUTH]}

    def absinfo(self, code):
        return AbsInfo(self.values[code], 0, 255, 0, 15, 0)

    def active_keys(self):
        return self.keys


class InputTests(unittest.TestCase):
    def test_normalization_and_supported_axis_layouts(self):
        self.assertEqual(normalize(128, 0, 255), 0)
        self.assertEqual(normalize(0, 0, 255), -1)
        self.assertEqual(normalize(255, 0, 255), 1)
        self.assertEqual(normalize(500, 0, 255), 1)
        self.assertEqual(normalize(10, 10, 10), 0)
        self.assertEqual(stick_axes(FakeInput().capabilities(absinfo=False)), (e.ABS_X, e.ABS_Y, e.ABS_Z, e.ABS_RZ))
        self.assertEqual(stick_axes({e.EV_ABS: [e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_RY]}),
                         (e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_RY))
        self.assertIsNone(stick_axes({e.EV_ABS: [e.ABS_X, e.ABS_Y]}))

    def test_input_binding_requires_bluetooth_and_saved_mac(self):
        device = FakeInput()
        self.assertTrue(matches_input(device, ADDRESS))
        self.assertFalse(matches_input(device, '00:00:00:00:00:00'))
        device.info = SimpleNamespace(bustype=e.BUS_USB)
        self.assertFalse(matches_input(device, ADDRESS))

    def test_axis_button_frames_and_dropped_event_resync(self):
        device = FakeInput()
        state = InputState(device)

        def send(kind, code, value):
            return state.accept(InputEvent(0, 0, kind, code, value), 123, device)

        send(e.EV_ABS, e.ABS_X, 255)
        send(e.EV_KEY, e.BTN_SOUTH, 1)
        self.assertTrue(send(e.EV_SYN, e.SYN_REPORT, 0))
        self.assertEqual(state.snapshot()['sticks']['left_x'], 1)
        self.assertEqual(state.buttons, {e.BTN_SOUTH})
        send(e.EV_KEY, e.BTN_SOUTH, 0)
        self.assertFalse(state.buttons)
        send(e.EV_SYN, e.SYN_DROPPED, 0)
        send(e.EV_ABS, e.ABS_X, 0)  # Ignore events until the next complete frame.
        device.values[e.ABS_X] = 128
        device.keys = [e.BTN_EAST]
        send(e.EV_SYN, e.SYN_REPORT, 0)
        self.assertEqual(state.snapshot()['sticks']['left_x'], 0)
        self.assertEqual(state.buttons, {e.BTN_EAST})
        self.assertEqual(state.dropped, 1)
        self.assertEqual(state.events, 3)
        self.assertEqual(state.axes[e.ABS_X]['observed_max'], 255)


class SelectionTests(unittest.TestCase):
    def test_scoped_selection_and_ambiguous_enrollment(self):
        self.assertTrue(is_lite2(PAD))
        self.assertFalse(is_lite2(dict(PAD, Name='Other Gamepad')))
        self.assertFalse(is_lite2(dict(PAD, Class=0)))
        self.assertTrue(is_lite2(dict(PAD, Class=0, UUIDs=list(HID_UUIDS))))
        objects = {PAD_PATH: {DEVICE: dict(PAD)}}
        self.assertEqual(choose_device(objects, None)[0], PAD_PATH)
        objects['/other'] = {DEVICE: dict(PAD, Address='AA:BB:CC:DD:EE:02')}
        with self.assertRaises(ValueError):
            choose_device(objects, None)
        self.assertEqual(choose_device(objects, ADDRESS)[0], PAD_PATH)
        self.assertEqual(choose_device(objects, '00:00:00:00:00:00'), (None, None))
        objects[PAD_PATH][DEVICE]['Paired'] = True
        self.assertEqual(choose_device(objects, None)[0], PAD_PATH)

    def test_agent_rejects_unselected_devices_and_non_hid_services(self):
        agent = PairingAgent()
        with self.assertRaises(DBusError):
            agent.check(PAD_PATH)
        agent.target = PAD_PATH
        agent.check(PAD_PATH)
        with self.assertRaises(DBusError):
            agent.check('/unrelated')
        # dbus-next's decorator wraps return values; invoke the actual service handler.
        methods = {item.name: item.fn for item in agent._get_methods(agent)}
        methods['AuthorizeService'](agent, PAD_PATH, next(iter(HID_UUIDS)))
        with self.assertRaises(DBusError):
            methods['AuthorizeService'](agent, PAD_PATH, 'unrelated-service')


class LifecycleTests(unittest.IsolatedAsyncioTestCase):
    async def exercise(self, paired, connected=False):
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'controller.json'
            monitor = ControllerMonitor(state_path=path)
            props = dict(PAD, Paired=paired, Connected=connected)
            objects = {'/org/bluez/hci0': {ADAPTER: {'Powered': True}}, PAD_PATH: {DEVICE: props}}
            calls = []
            async def call(path, interface, member, signature='', body=None, timeout=5):
                calls.append(member)
                if member == 'GetManagedObjects':
                    return [objects]
                if member == 'Pair':
                    props['Paired'] = True
                elif member == 'Connect':
                    props['Connected'] = True
                elif member == 'Set':
                    props[body[1]] = body[2].value
                return []
            async def attach(address):
                self.assertEqual(address, ADDRESS)
                monitor.update(input_ready=True)
                monitor.stop.set()
            real_sleep = asyncio.sleep
            async def quick_sleep(delay):
                await real_sleep(0)
            monitor.call, monitor.attach_input = call, attach
            with patch('apptirover.controller.asyncio.sleep', quick_sleep):
                await asyncio.wait_for(monitor.manage(ADDRESS if paired else None), 1)
            self.assertEqual(calls.count('Pair'), 0 if paired else 1)
            self.assertEqual(calls.count('Connect'), 0 if connected else 1)
            self.assertTrue(props['Trusted'])
            self.assertTrue(monitor.snapshot()['input_ready'])
            self.assertEqual(json.loads(path.read_text())['address'], ADDRESS)
            self.assertEqual(ControllerMonitor(state_path=path)._load_address(), ADDRESS)

    async def test_first_pair_persists_designated_address(self):
        await self.exercise(False)

    async def test_existing_bond_reconnects_without_pairing_again(self):
        await self.exercise(True)

    async def test_already_connected_explicit_selection_is_persisted(self):
        await self.exercise(True, connected=True)

    async def test_disconnect_clears_controls_and_cancels_reader(self):
        monitor = ControllerMonitor()
        monitor.update(input_ready=True, device='/dev/input/event99', input={'sticks': {'left_x': 1}})
        monitor.input_task = asyncio.create_task(asyncio.sleep(10))
        task = monitor.input_task
        await monitor.clear_input()
        self.assertTrue(task.cancelled())
        self.assertFalse(monitor.snapshot()['input_ready'])
        self.assertIsNone(monitor.snapshot()['input'])


if __name__ == '__main__':
    unittest.main()
