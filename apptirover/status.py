"""Extensible status rendering; no hardware commands or camera capture."""

import json
import os
from pathlib import Path

from rich import box
from rich.table import Table
from rich.text import Text


def host_status():
    temperature = None
    try:
        temperature = float(Path('/sys/class/thermal/thermal_zone0/temp').read_text()) / 1000
    except (OSError, ValueError):
        pass
    cameras = {}
    for node in Path('/sys/class/video4linux').glob('video*'):
        try:
            # USB video interfaces only: do not count Pi codec/ISP nodes as cameras.
            if (node / 'device/subsystem').resolve().name != 'usb':
                continue
            name = (node / 'name').read_text().strip()
            cameras.setdefault(name, []).append('/dev/' + node.name)
        except OSError:
            continue
    return {"cpu_temperature_c": temperature, "load_1m": os.getloadavg()[0], "usb_cameras": cameras}


def number(value, digits=2):
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return f"{value:.{digits}f}"
    return "unavailable" if value is None else str(value)


def packet_values(snapshot, kind):
    packet = snapshot['packets'].get(str(kind))
    return packet['data'] if packet else {}


def stale_suffix(snapshot, kind):
    packet = snapshot['packets'].get(str(kind))
    if packet and not packet['fresh']:
        return f"  [STALE: {packet['age_s']:.1f}s old]"
    return ""


def triple(data, keys):
    return ' / '.join(number(data[key]) if data.get(key) is not None else 'n/a' for key in keys)


def freshest(snapshot, field):
    candidates = [(int(kind), packet) for kind, packet in snapshot['packets'].items()
                  if field in packet['data']]
    if not candidates:
        return {}, ''
    kind, packet = min(candidates, key=lambda item: (not item[1]['fresh'], item[1]['age_s']))
    return packet['data'], stale_suffix(snapshot, kind)


def render(snapshot, host, uptime, controller=None):
    table = Table(
        title=f"apptirover | telemetry | {uptime:.0f}s", box=box.SIMPLE,
        show_header=False, expand=True, padding=(0, 1),
    )
    table.add_column("Item", style="cyan", no_wrap=True)
    table.add_column("Value", ratio=1)

    def row(label, value, style=None):
        # Treat firmware/error strings as text, never Rich markup or terminal escapes.
        cleaned = ''.join(char if char.isprintable() else ' ' for char in str(value))
        table.add_row(label, Text(cleaned, style=style))

    state = snapshot['state']
    row('UART', f"{snapshot['port']} @ {snapshot['baud']} | {state}",
        'green' if state == 'telemetry live' else 'yellow')
    ages = []
    for kind, label in ((1001, 'chassis'), (1002, 'IMU')):
        packet = snapshot['packets'].get(str(kind))
        ages.append(f"{label}: {packet['age_s']:.1f}s old" if packet else f"{label}: not received")
    row('Feedback', ' | '.join(ages))
    base, imu = packet_values(snapshot, 1001), packet_values(snapshot, 1002)
    base_stale, imu_stale = stale_suffix(snapshot, 1001), stale_suffix(snapshot, 1002)
    orientation, orientation_stale = freshest(snapshot, 'r')
    temperature, temperature_stale = freshest(snapshot, 'temp')
    voltage = number(base.get('v'))
    row('Battery', (f"{voltage} V" if 'v' in base else 'not received') + base_stale)
    row('Charge/current', 'not reported; no estimated percentage')
    row('Motor L / R', triple(base, ('L', 'R')) + ' (reported; no encoders)' + base_stale)
    row('Roll/pitch/yaw', triple(orientation, ('r', 'p', 'y')) + ' deg' + orientation_stale)
    row('Accel XYZ', triple(imu, ('ax', 'ay', 'az')) + ' (firmware units)' + imu_stale)
    row('Gyro XYZ', triple(imu, ('gx', 'gy', 'gz')) + ' (firmware units)' + imu_stale)
    row('Mag XYZ', triple(imu, ('mx', 'my', 'mz')) + ' (raw)' + imu_stale)
    row('ESP32 temp', number(temperature.get('temp')) + ' C' + temperature_stale)
    row('Pi', f"CPU {number(host['cpu_temperature_c'], 1)} C | load {host['load_1m']:.2f}")
    cameras = [f"{name}: {', '.join(nodes)}" for name, nodes in host['usb_cameras'].items()]
    row('USB camera', ('; '.join(cameras) + ' | capture not started') if cameras else 'not detected')
    pad = controller or {'state': 'disabled', 'input_ready': False}
    battery = pad.get('battery_percent')
    row('8BitDo', pad['state'] + (f' | battery {battery}%' if battery is not None else ''),
        'green' if pad['input_ready'] else 'yellow')
    inputs = pad.get('input') if pad['input_ready'] else None
    if inputs:
        sticks = inputs['sticks']
        row('Sticks X / Y', f"L {sticks['left_x']:+.2f} / {sticks['left_y']:+.2f}   R {sticks['right_x']:+.2f} / {sticks['right_y']:+.2f}")
        hats = [f"{key.removeprefix('ABS_')}={axis['value']}" for key, axis in inputs['axes'].items() if key.startswith('ABS_HAT')]
        row('Buttons / D-pad', ', '.join(inputs['buttons'] + hats) or 'none pressed')
        row('Input events', f"{inputs['events']} | {inputs['last_event'] or 'waiting for stick/button movement'}")
    elif pad.get('last_error'):
        row('Controller error', pad['last_error'], 'red')
    row('Movement', 'disabled; telemetry queries only')
    row('Traffic', f"queries {snapshot['tx_queries']} | replies {snapshot['telemetry_packets']} | echoes {snapshot['echoes']}")
    row('Serial health', f"invalid lines {snapshot['invalid_lines']} | reconnects {snapshot['reconnects']}")
    known = {1001: {'T', 'L', 'R', 'v', 'r', 'p', 'y', 'temp'},
             1002: {'T', 'r', 'p', 'y', 'temp', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz'}}
    for kind, packet in snapshot['packets'].items():
        extra = {key: value for key, value in packet['data'].items() if key not in known.get(int(kind), set())}
        if extra:
            row(f'Extra T={kind}', json.dumps(extra, ensure_ascii=True) + stale_suffix(snapshot, int(kind)))
    for kind, packet in snapshot['other_packets'].items():
        suffix = '' if packet['fresh'] else ' [STALE]'
        row(f'Reply T={kind}', json.dumps(packet['data'], ensure_ascii=True) + suffix)
    if snapshot['last_error']:
        row('Last error', snapshot['last_error'], 'red')
    row('Exit', 'Ctrl+C | unavailable UART retries automatically')
    return table
