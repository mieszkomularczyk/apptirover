"""Extensible status rendering; no hardware commands or camera capture."""

import json
import os
from pathlib import Path

from rich import box
from rich.table import Table
from rich.text import Text
from rich.console import Group


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


def render(snapshot, host, uptime, controller=None, width=80, camera=None, height=None):
    """Fixed-width cells keep signs, decimal places, and sections from shifting."""
    width = max(60, width)
    label_width = 15 if width >= 76 else 12
    value_width = 10 if width >= 76 else 8
    info_width = width - label_width - 3 * value_width - 10
    hint = ("Left Y: throttle   Right X: turn   Center: stop   Ctrl+C: exit"
            if snapshot.get('drive', {}).get('enabled') else "Monitoring only   |   Ctrl+C: exit")
    if camera:
        hint = ('Left Y: move  Right X: turn  Center: stop  D: detection  Ctrl+C: exit'
                if snapshot.get('drive', {}).get('enabled') else 'Monitoring only   |   D: detection   |   Ctrl+C: exit')
    table = Table(
        title=Text(f"apptirover  |  {uptime:8.1f}s", style="bold cyan"),
        caption=Text(hint, style="dim"),
        box=box.SIMPLE, show_header=False, width=width, padding=(0, 1),
    )
    table.add_column(width=label_width, no_wrap=True, overflow="ellipsis")
    for _ in range(3):
        table.add_column(width=value_width, justify="right", no_wrap=True, overflow="ellipsis")
    table.add_column(width=info_width, no_wrap=True, overflow="ellipsis")

    def clean(value, style=None):
        return Text(''.join(c if c.isprintable() else ' ' for c in str(value)), style=style)

    rows = []

    def row(label, first='', second='', third='', note='', style=None):
        rows.append((label, (clean(label, 'white'), clean(first, style), clean(second, style),
                            clean(third, style), clean(note, style))))

    def section(label, first='', second='', third='', note='', color='cyan'):
        rows.append((label, tuple(clean(value, f'bold {color} on grey11')
                                 for value in (label, first, second, third, note))))

    def fixed(value, suffix='', signed=False, digits=2):
        if type(value) not in (int, float):
            return 'n/a'
        text = f"{value:+.{digits}f}" if signed else f"{value:.{digits}f}"
        if len(text + suffix) > value_width:
            text = f"{value:.1e}"
        return text + suffix

    def freshness(kind):
        packet = snapshot['packets'].get(str(kind))
        if not packet:
            return 'not received', 'yellow'
        return ((f"{packet['age_s']:.1f}s old", 'green') if packet['fresh']
                else (f"STALE {packet['age_s']:.1f}s", 'yellow'))

    pad = controller or {'state': 'disabled', 'input_ready': False}
    drive = snapshot.get('drive', {'enabled': False, 'state': 'disabled'})
    base, imu = packet_values(snapshot, 1001), packet_values(snapshot, 1002)
    orientation, orientation_stale = freshest(snapshot, 'r')
    temperature, temperature_stale = freshest(snapshot, 'temp')
    base_age, base_color = freshness(1001)
    imu_age, imu_color = freshness(1002)
    section('BATTERY', 'Rover', 'Pad', '', 'charge / current', color='yellow')
    row('Supply', fixed(base.get('v'), ' V'), fixed(pad.get('battery_percent') if pad.get('connected') else None, '%', digits=0),
        '', 'not reported' if base_color == 'green' else base_age, base_color)

    section('DRIVE', 'Left', 'Right', 'Limit', 'motor power', color='green')
    row('Output', fixed(drive.get('left_power', 0) * 100, '%', True, 1),
        fixed(drive.get('right_power', 0) * 100, '%', True, 1),
        fixed(drive.get('max_power', 0) * 100, '%', digits=0), drive['state'],
        'green' if drive.get('armed') else 'yellow')
    row('Board L / R', fixed(base.get('L'), signed=True), fixed(base.get('R'), signed=True), '',
        'reported; no encoders' if base_color == 'green' else base_age, base_color)

    inputs = pad.get('input') if pad['input_ready'] else None
    sticks = inputs['sticks'] if inputs else {}
    pad_state = 'input ready' if pad['input_ready'] else pad['state']
    section('CONTROLLER', 'X', 'Y', 'Events', pad_state, color='magenta')
    row('Left stick', fixed(sticks.get('left_x'), signed=True), fixed(sticks.get('left_y'), signed=True),
        inputs['events'] if inputs else 'n/a', 'forward / reverse', 'magenta')
    row('Right stick', fixed(sticks.get('right_x'), signed=True), fixed(sticks.get('right_y'), signed=True),
        '', 'turn left / right', 'magenta')
    axes = inputs['axes'] if inputs else {}
    row('D-pad / keys', axes.get('ABS_HAT0X', {}).get('value', 'n/a'),
        axes.get('ABS_HAT0Y', {}).get('value', 'n/a'),
        len(inputs['buttons']) if inputs else 'n/a',
        ', '.join(inputs['buttons']) if inputs and inputs['buttons'] else 'none pressed', 'magenta')

    if camera:
        section('CAMERA', 'Video FPS', 'AI FPS', 'People', camera['state'], color='bright_green')
        row('Stream', fixed(camera.get('video_fps'), digits=1), fixed(camera.get('detection_fps'), digits=1),
            camera.get('people') if camera.get('people') is not None else 'n/a',
            camera['detection_state'], 'green' if camera.get('ready') else 'yellow')
        observation = camera.get('observation') or {}
        row('Vision age', fixed(camera.get('frame_age_s'), ' s'),
            fixed(observation.get('inference_ms'), ' ms', digits=1), fixed(camera.get('observation_age_s'), ' s'),
            'fresh' if camera.get('observation_valid') else 'no fresh detection', 'dim')

    section('IMU / GYRO', 'X / Roll', 'Y / Pitch', 'Z / Yaw', imu_age, color='cyan')
    row('Attitude', *(fixed(orientation.get(key), signed=True) for key in ('r', 'p', 'y')),
        'STALE / deg' if orientation_stale else 'degrees', 'yellow' if orientation_stale else 'cyan')
    row('Acceleration', *(fixed(imu.get(key), signed=True) for key in ('ax', 'ay', 'az')), 'firmware units', imu_color)
    row('Gyroscope', *(fixed(imu.get(key), signed=True) for key in ('gx', 'gy', 'gz')), 'firmware units', imu_color)
    row('Magnetometer', *(fixed(imu.get(key), signed=True, digits=0) for key in ('mx', 'my', 'mz')), 'raw', imu_color)

    section('SYSTEM', 'Pi CPU', 'ESP32', 'Load', 'camera / UART', color='blue')
    camera_presence = 'USB camera present' if host['usb_cameras'] else 'no USB camera'
    row('Health', fixed(host['cpu_temperature_c'], ' C', digits=1), 'STALE' if temperature_stale else fixed(temperature.get('temp'), ' C', digits=1),
        fixed(host['load_1m']), camera_presence, 'blue')
    row('UART', snapshot['port'].removeprefix('/dev/'), snapshot['baud'], base_age,
        snapshot['state'], 'green' if snapshot['state'] == 'telemetry live' else 'yellow')
    row('RX / TX / bad', snapshot['telemetry_packets'],
        snapshot['tx_queries'] + drive.get('commands', 0), snapshot['invalid_lines'],
        f"reconnects {snapshot['reconnects']}", 'dim')
    known = {1001: {'T', 'L', 'R', 'v', 'r', 'p', 'y', 'temp'},
             1002: {'T', 'r', 'p', 'y', 'temp', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz'}}
    for kind, packet in snapshot['packets'].items():
        extra = {key: value for key, value in packet['data'].items() if key not in known.get(int(kind), set())}
        if extra:
            row(f'Extra T={kind}', note=json.dumps(extra, ensure_ascii=True) + stale_suffix(snapshot, int(kind)))
    for kind, packet in snapshot['other_packets'].items():
        row(f'Reply T={kind}', note=json.dumps(packet['data'], ensure_ascii=True) + ('' if packet['fresh'] else ' STALE'))
    for label, error in (('UART error', snapshot['last_error']), ('Pad error', pad.get('last_error'))):
        if error:
            row(label, note=error, style='red')
    if camera:
        for label, error in (('Camera error', camera.get('last_error')), ('Detector error', camera.get('detection_error'))):
            if error:
                row(label, note=error, style='red')
    command = None
    if camera:
        command = Text('Remote video: ' + camera['ffplay_command'], style='bright_green', overflow='fold')
        # Keep the complete command at the top, including on a short terminal.
        # Omit secondary telemetry rows first; JSON/plain retain every field.
        if height:
            from rich.console import Console
            measure = Console(width=width, color_system=None)
            command_lines = len(measure.render_lines(command, measure.options, pad=False))
            budget = max(5, height - command_lines - 5)
            optional = ['RX / TX / bad', 'Board L / R', 'D-pad / keys', 'Magnetometer',
                        'Health', 'Vision age', 'Acceleration']
            for label in optional:
                if len(rows) <= budget:
                    break
                rows = [item for item in rows if item[0] != label]
    for _, cells in rows:
        table.add_row(*cells)
    return Group(command, table) if command else table
