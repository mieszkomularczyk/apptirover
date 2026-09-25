"""Scoped BlueZ enrollment/reconnection and read-only evdev controller monitoring."""

import asyncio
from contextlib import suppress
import copy
import fcntl
import json
import os
from pathlib import Path
import re
import threading
import time

from dbus_next import BusType, DBusError, Message, MessageType, Variant
from dbus_next.aio import MessageBus
from dbus_next.service import ServiceInterface, method
from evdev import InputDevice, list_devices

from .gamepad import InputState, matches_input


DEVICE = 'org.bluez.Device1'
ADAPTER = 'org.bluez.Adapter1'
HID_UUIDS = {'00001124-0000-1000-8000-00805f9b34fb', '00001812-0000-1000-8000-00805f9b34fb'}
AGENT_PATH = '/org/apptirover/agent'
MAC = re.compile(r'(?:[0-9A-F]{2}:){5}[0-9A-F]{2}')


def unwrap(value):
    if isinstance(value, Variant):
        return unwrap(value.value)
    if isinstance(value, dict):
        return {key: unwrap(item) for key, item in value.items()}
    if isinstance(value, list):
        return [unwrap(item) for item in value]
    return value


def is_lite2(props):
    name = ''.join(props.get('Name', '').lower().split())
    device_class = props.get('Class', 0)
    gamepad = device_class & 0x1F00 == 0x0500 and device_class & 0x0C in (0x04, 0x08)
    return name == '8bitdolite2' and (gamepad or bool(HID_UUIDS.intersection(props.get('UUIDs', []))))


def choose_device(objects, address):
    candidates = [(path, interfaces[DEVICE]) for path, interfaces in objects.items()
                  if DEVICE in interfaces and
                  (interfaces[DEVICE].get('Address', '').upper() == address if address else is_lite2(interfaces[DEVICE]))]
    if not address:
        bonded = [item for item in candidates if item[1].get('Paired')]
        candidates = bonded or candidates
    if len(candidates) > 1:
        raise ValueError('Multiple Lite 2 controllers found; specify --controller-address MAC')
    return candidates[0] if candidates else (None, None)


class PairingAgent(ServiceInterface):
    """Approve only the device this process has deliberately selected."""

    def __init__(self):
        super().__init__('org.bluez.Agent1')
        self.target = None

    def check(self, device):
        if not self.target or device != self.target:
            raise DBusError('org.bluez.Error.Rejected', 'Not the designated Lite 2 controller')

    @method()
    def Release(self):
        self.target = None

    @method()
    def RequestConfirmation(self, device: 'o', passkey: 'u'):
        self.check(device)

    @method()
    def RequestAuthorization(self, device: 'o'):
        self.check(device)

    @method()
    def AuthorizeService(self, device: 'o', uuid: 's'):
        self.check(device)
        if uuid.lower() not in HID_UUIDS:
            raise DBusError('org.bluez.Error.Rejected', 'Only gamepad HID services are allowed')

    @method()
    def RequestPinCode(self, device: 'o') -> 's':
        raise DBusError('org.bluez.Error.Rejected', 'Legacy PIN pairing is unsupported; use D mode')

    @method()
    def RequestPasskey(self, device: 'o') -> 'u':
        raise DBusError('org.bluez.Error.Rejected', 'Passkey entry is unsupported; use D mode')

    @method()
    def DisplayPinCode(self, device: 'o', pincode: 's'):
        self.check(device)

    @method()
    def DisplayPasskey(self, device: 'o', passkey: 'u', entered: 'q'):
        self.check(device)

    @method()
    def Cancel(self):
        pass


class ControllerMonitor:
    def __init__(self, address=None, state_path=None):
        root = Path(os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local/state')))
        self.state_path = Path(state_path) if state_path else root / 'apptirover/controller.json'
        self.explicit_address = address.upper() if address else None
        self.persisted_address = None
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.loop = self.task = None
        self.bus = None
        self.agent = PairingAgent()
        self.scanning_adapter = None
        self.input_task = None
        self.input_device = None
        self.data = dict(state='starting', address=None, name=None, paired=False, connected=False,
                         input_ready=False, device=None, input=None, battery_percent=None,
                         last_error='', connections=0)
        self.thread = threading.Thread(target=self._thread_main, name='controller', daemon=True)

    def update(self, **values):
        with self.lock:
            self.data.update(values)

    def snapshot(self):
        with self.lock:
            snapshot = copy.deepcopy(self.data)
        if snapshot['input']:
            last = snapshot['input'].pop('last_event_at')
            snapshot['input']['last_event_age_s'] = round(time.monotonic() - last, 3) if last else None
        return snapshot

    def start(self):
        self.thread.start()

    def close(self):
        self.stop.set()
        if self.loop and not self.loop.is_closed() and self.task:
            with suppress(RuntimeError):
                self.loop.call_soon_threadsafe(self.task.cancel)
        self.thread.join(timeout=5)

    def _load_address(self):
        if self.explicit_address:
            return self.explicit_address
        try:
            address = json.loads(self.state_path.read_text())['address'].upper()
        except FileNotFoundError:
            return None
        if not MAC.fullmatch(address):
            raise ValueError(f'Invalid controller address in {self.state_path}')
        return address

    def _save_address(self, address):
        if address == self.persisted_address:
            return
        temporary = self.state_path.with_suffix('.tmp')
        temporary.write_text(json.dumps({'address': address, 'name': '8BitDo Lite 2'}) + '\n')
        temporary.replace(self.state_path)
        self.persisted_address = address

    def _thread_main(self):
        try:
            self.state_path.parent.mkdir(parents=True, exist_ok=True)
            with self.state_path.with_suffix('.lock').open('w') as lockfile:
                try:
                    fcntl.flock(lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    raise RuntimeError('Another apptirover controller monitor is already running') from None
                asyncio.run(self._run())
        except Exception as error:
            self.update(state='error', input_ready=False, input=None, last_error=str(error))

    async def call(self, path, interface, member, signature='', body=None, timeout=5):
        reply = await asyncio.wait_for(self.bus.call(Message(
            destination='org.bluez', path=path, interface=interface, member=member,
            signature=signature, body=body or [])), timeout)
        if reply.message_type == MessageType.ERROR:
            raise DBusError(reply.error_name, reply.body[0] if reply.body else member)
        return unwrap(reply.body)

    async def set_property(self, path, interface, name, value):
        await self.call(path, 'org.freedesktop.DBus.Properties', 'Set', 'ssv',
                        [interface, name, Variant('b', value)])

    async def discovery(self, adapter, enabled):
        if enabled and not self.scanning_adapter:
            await self.call(adapter, ADAPTER, 'SetDiscoveryFilter', 'a{sv}',
                            [{'Transport': Variant('s', 'auto')}])
            await self.call(adapter, ADAPTER, 'StartDiscovery')
            self.scanning_adapter = adapter
        elif not enabled and self.scanning_adapter:
            path, self.scanning_adapter = self.scanning_adapter, None
            await self.call(path, ADAPTER, 'StopDiscovery')

    async def clear_input(self):
        if self.input_task:
            self.input_task.cancel()
            with suppress(asyncio.CancelledError, OSError):
                await self.input_task
            self.input_task = None
        self.input_device = None
        self.update(input_ready=False, device=None, input=None)

    async def read_input(self, device):
        try:
            state = InputState(device)
            self.update(input_ready=True, device=device.path, input=state.snapshot(),
                        state='input ready', last_error='')
            async for event in device.async_read_loop():
                if state.accept(event, time.monotonic(), device):
                    self.update(input=state.snapshot())
        except OSError as error:
            self.update(last_error=f'Input device: {error}')
        finally:
            device.close()
            with self.lock:
                self.data.update(input_ready=False, device=None, input=None)
                if self.data['state'] == 'input ready':
                    self.data['state'] = 'input device unavailable'

    async def attach_input(self, address):
        if self.input_task and not self.input_task.done():
            return
        if self.input_task:
            # Retrieve any unexpected reader exception instead of hiding a dead task.
            await self.input_task
            self.input_task = None
        for path in sorted(list_devices()):
            device = None
            try:
                device = InputDevice(path)
                if matches_input(device, address):
                    self.input_device = device
                    self.input_task = asyncio.create_task(self.read_input(device))
                    return
            except OSError as error:
                self.update(last_error=f'Cannot inspect {path}: {error}')
            finally:
                if device and device is not self.input_device:
                    device.close()

    async def _run(self):
        self.loop, self.task = asyncio.get_running_loop(), asyncio.current_task()
        address = self._load_address()
        self.update(address=address)
        try:
            while not self.stop.is_set():
                try:
                    address = self._load_address()
                    self.bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
                    self.bus.export(AGENT_PATH, self.agent)
                    await self.call('/org/bluez', 'org.bluez.AgentManager1', 'RegisterAgent', 'os',
                                    [AGENT_PATH, 'NoInputNoOutput'])
                    address = await self.manage(address)
                except (OSError, DBusError, asyncio.TimeoutError, EOFError) as error:
                    self.update(state='Bluetooth unavailable', connected=False, last_error=str(error) or type(error).__name__)
                finally:
                    await self.clear_input()
                    if self.bus:
                        with suppress(Exception):
                            await asyncio.wait_for(self.discovery(None, False), 1)
                        with suppress(Exception):
                            await self.call('/org/bluez', 'org.bluez.AgentManager1', 'UnregisterAgent', 'o', [AGENT_PATH], timeout=1)
                        self.bus.disconnect()
                        self.bus = None
                    self.scanning_adapter = None
                await asyncio.sleep(2)
        except asyncio.CancelledError:
            pass

    async def manage(self, address):
        next_attempt = 0
        retry = 2
        while not self.stop.is_set():
            objects = (await self.call('/', 'org.freedesktop.DBus.ObjectManager', 'GetManagedObjects'))[0]
            adapters = [(path, interfaces[ADAPTER]) for path, interfaces in objects.items() if ADAPTER in interfaces]
            if not adapters:
                self.scanning_adapter = None
                self.update(state='Bluetooth adapter missing', connected=False)
                await self.clear_input()
                await asyncio.sleep(2)
                continue
            adapter, props = adapters[0]
            if not props.get('Powered'):
                await self.set_property(adapter, ADAPTER, 'Powered', True)
            try:
                path, device = choose_device(objects, address)
            except ValueError as error:
                self.update(state='ambiguous controllers', last_error=str(error))
                await asyncio.sleep(1)
                continue
            if not device:
                await self.clear_input()
                self.update(state='searching for saved controller' if address else 'searching; put Lite 2 in D pairing mode',
                            connected=False, battery_percent=None)
                await self.discovery(adapter, True)
                await asyncio.sleep(1)
                continue
            if not device.get('Paired') and not is_lite2(device):
                self.update(state='waiting for Lite 2 identity', connected=False,
                            last_error='Selected address must advertise Lite 2 name and gamepad/HID service')
                await self.discovery(adapter, True)
                await asyncio.sleep(1)
                continue
            self.agent.target = path
            connected = device.get('Connected', False)
            paired = device.get('Paired', False)
            self.update(address=device['Address'], name=device.get('Name'), paired=paired,
                        connected=connected, battery_percent=objects[path].get('org.bluez.Battery1', {}).get('Percentage'))
            if not connected:
                await self.clear_input()
            if not paired or not connected:
                self.update(state='bonded; waiting to reconnect' if paired else 'controller found')
                if time.monotonic() >= next_attempt:
                    try:
                        if not paired:
                            await self.discovery(adapter, True)
                            self.update(state='pairing')
                            await self.call(path, DEVICE, 'Pair', timeout=35)
                        await self.set_property(path, DEVICE, 'Trusted', True)
                        address = device['Address'].upper()
                        self._save_address(address)
                        self.update(address=address, paired=True, state='connecting')
                        await self.discovery(adapter, False)
                        await self.call(path, DEVICE, 'Connect', timeout=15)
                        retry = 2
                        self.update(last_error='')
                    except (DBusError, asyncio.TimeoutError) as error:
                        self.update(last_error=str(error) or 'Bluetooth operation timed out')
                        if not paired:
                            with suppress(DBusError, asyncio.TimeoutError):
                                await self.call(path, DEVICE, 'CancelPairing')
                        next_attempt = time.monotonic() + retry
                        retry = min(30, retry * 2)
                await asyncio.sleep(1)
                continue
            if address is None:
                address = device['Address'].upper()
            self._save_address(address)
            if not device.get('Trusted'):
                await self.set_property(path, DEVICE, 'Trusted', True)
            await self.discovery(adapter, False)
            ready_before = self.snapshot()['input_ready']
            if not ready_before:
                self.update(state='connected; waiting for Linux input device')
            await self.attach_input(address)
            await asyncio.sleep(0)
            if not ready_before and self.snapshot()['input_ready']:
                self.update(connections=self.snapshot()['connections'] + 1)
            next_attempt, retry = 0, 2
            await asyncio.sleep(1)
        return address
