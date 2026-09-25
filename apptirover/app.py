"""Controller driving and live rover status."""

import argparse
import json
import math
import signal
import threading
import time

from rich.console import Console
from rich.live import Live

from .serial_monitor import SerialMonitor
from .status import host_status, render
from .controller import ControllerMonitor, MAC
from .telemetry import Telemetry
from .camera import CameraMonitor, add_arguments, config_from_arguments
from .terminal import TerminalKeys


def arguments(argv=None):
    parser = argparse.ArgumentParser(description='WAVE ROVER: left stick forward/reverse, right stick steering.')
    parser.add_argument('--port', default='/dev/ttyAMA0', help='GPIO UART on this Pi 5')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--poll-interval', type=float, default=0.5, help='Seconds between queries of each telemetry type')
    parser.add_argument('--refresh', type=float, default=5.0, help='Screen refreshes per second')
    parser.add_argument('--no-drive', action='store_true', help='Read telemetry and controller; send no motor/configuration commands')
    parser.add_argument('--max-power', type=float, default=0.5, help='Maximum motor power fraction (0.05–1; default 0.5)')
    parser.add_argument('--duration', type=float, default=0, help='Exit after this many seconds; 0 runs until Ctrl+C')
    controllers = parser.add_mutually_exclusive_group()
    controllers.add_argument('--no-controller', action='store_true', help='Do not access Bluetooth or drive')
    controllers.add_argument('--controller-only', action='store_true', help='Pair and read inputs without opening the rover UART')
    controllers.add_argument('--camera-only', action='store_true', help='Camera/RTSP only; no UART or Bluetooth access')
    parser.add_argument('--controller-address', type=str.upper, help='Select a specific Lite 2 Bluetooth address')
    output = parser.add_mutually_exclusive_group()
    output.add_argument('--plain', action='store_true', help='Print snapshots without redrawing the screen')
    output.add_argument('--json', action='store_true', help='Print newline-delimited JSON status snapshots')
    add_arguments(parser)
    args = parser.parse_args(argv)
    if args.controller_address and not MAC.fullmatch(args.controller_address):
        parser.error('--controller-address must look like AA:BB:CC:DD:EE:FF')
    if args.controller_address and (args.no_controller or args.camera_only):
        parser.error('--controller-address requires controller monitoring')
    if args.camera_only and args.no_camera:
        parser.error('--camera-only cannot be combined with --no-camera')
    if not 9600 <= args.baud <= 921600:
        parser.error('--baud must be between 9600 and 921600')
    for key, low, high in (('poll_interval', 0.1, 10), ('refresh', 0.2, 10), ('duration', 0, 86400), ('max_power', 0.05, 1)):
        value = getattr(args, key)
        if not math.isfinite(value) or not low <= value <= high:
            parser.error(f'--{key.replace("_", "-")} must be between {low} and {high}')
    if not (args.no_drive or args.no_controller or args.controller_only or args.camera_only) and args.poll_interval > 0.5:
        parser.error('--poll-interval must be at most 0.5 seconds while driving')
    args.camera_config = config_from_arguments(args, parser)
    return args


def main(argv=None):
    args = arguments(argv)
    console = Console()
    controller = None if args.no_controller or args.camera_only else ControllerMonitor(args.controller_address)
    source = controller.snapshot if controller and not args.no_drive else None
    monitor = None if args.controller_only or args.camera_only else SerialMonitor(
        args.port, args.baud, args.poll_interval, controller_source=source, max_power=args.max_power)
    stopping = threading.Event()
    camera = CameraMonitor(args.camera_config)
    toggles = []
    previous_handlers = {}
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous_handlers[sig] = signal.signal(sig, lambda *_: stopping.set())
    previous_handlers[signal.SIGUSR1] = signal.signal(signal.SIGUSR1, lambda *_: toggles.append(True))
    started = time.monotonic()
    interactive = console.is_terminal and not console.is_dumb_terminal and not args.plain and not args.json
    interval = 1 / args.refresh if interactive or args.json else max(2, 1 / args.refresh)
    live = Live(console=console, auto_refresh=False, screen=True) if interactive else None
    result = 0
    keys = None
    try:
        if monitor:
            monitor.start()
        if controller:
            controller.start()
        camera.start()
        keys = TerminalKeys(interactive)
        if live:
            live.start()
        while not stopping.is_set():
            if 'd' in keys.read():
                toggles.append(True)
            if toggles:
                count = len(toggles)
                del toggles[:count]
                if count % 2:
                    camera.toggle_detection()
            now = time.monotonic()
            snapshot = monitor.snapshot() if monitor else Telemetry(args.port, args.baud).snapshot(now)
            if not monitor:
                snapshot['state'] = 'disabled'
            host = host_status()
            pad = controller.snapshot() if controller else {'state': 'disabled (--no-controller)', 'input_ready': False}
            vision = camera.snapshot()
            if args.json:
                print(json.dumps({"uptime_s": round(now - started, 3), "rover": snapshot, "host": host,
                                  "controller": pad, "camera": vision}), flush=True)
            else:
                screen = render(snapshot, host, now - started, pad, width=console.width,
                                camera=vision, height=console.height if interactive else None)
                if live:
                    live.update(screen, refresh=True)
                else:
                    console.print(screen)
            remaining = args.duration - (now - started) if args.duration else interval
            if args.duration and remaining <= 0:
                # Every enabled subsystem must be ready at the end of a bounded check.
                result = 0 if ((not monitor or snapshot['state'] == 'telemetry live')
                               and (not controller or pad['input_ready'])
                               and (not vision['enabled'] or vision['ready'])) else 2
                break
            if (monitor and not monitor.thread.is_alive()) or (controller and not controller.thread.is_alive()):
                result = 1
                break
            stopping.wait(min(interval, remaining))
    except BrokenPipeError:
        result = 0
    finally:
        # Stop motors before waiting for Bluetooth pairing/agent cleanup.
        if monitor:
            monitor.close()
        if controller:
            controller.close()
        camera.close()
        if keys:
            keys.close()
        if live:
            live.stop()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
    return result
