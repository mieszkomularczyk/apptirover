"""Live rover telemetry and controller inputs; motion remains disabled."""

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


def arguments(argv=None):
    parser = argparse.ArgumentParser(description='Live WAVE ROVER telemetry (no motion commands).')
    parser.add_argument('--port', default='/dev/ttyAMA0', help='GPIO UART on this Pi 5')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--poll-interval', type=float, default=0.5, help='Seconds between queries of each telemetry type')
    parser.add_argument('--refresh', type=float, default=2.0, help='Screen refreshes per second')
    parser.add_argument('--duration', type=float, default=0, help='Exit after this many seconds; 0 runs until Ctrl+C')
    controllers = parser.add_mutually_exclusive_group()
    controllers.add_argument('--no-controller', action='store_true', help='UART telemetry only; do not access Bluetooth')
    controllers.add_argument('--controller-only', action='store_true', help='Pair and read inputs without opening the rover UART')
    parser.add_argument('--controller-address', type=str.upper, help='Select a specific Lite 2 Bluetooth address')
    output = parser.add_mutually_exclusive_group()
    output.add_argument('--plain', action='store_true', help='Print snapshots without redrawing the screen')
    output.add_argument('--json', action='store_true', help='Print newline-delimited JSON status snapshots')
    args = parser.parse_args(argv)
    if args.controller_address and not MAC.fullmatch(args.controller_address):
        parser.error('--controller-address must look like AA:BB:CC:DD:EE:FF')
    if args.controller_address and args.no_controller:
        parser.error('--controller-address cannot be combined with --no-controller')
    if not 9600 <= args.baud <= 921600:
        parser.error('--baud must be between 9600 and 921600')
    for key, low, high in (('poll_interval', 0.1, 10), ('refresh', 0.2, 10), ('duration', 0, 86400)):
        value = getattr(args, key)
        if not math.isfinite(value) or not low <= value <= high:
            parser.error(f'--{key.replace("_", "-")} must be between {low} and {high}')
    return args


def main(argv=None):
    args = arguments(argv)
    console = Console()
    monitor = None if args.controller_only else SerialMonitor(args.port, args.baud, args.poll_interval)
    controller = None if args.no_controller else ControllerMonitor(args.controller_address)
    stopping = threading.Event()
    previous_handlers = {}
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous_handlers[sig] = signal.signal(sig, lambda *_: stopping.set())
    started = time.monotonic()
    interactive = console.is_terminal and not console.is_dumb_terminal and not args.plain and not args.json
    interval = 1 / args.refresh if interactive or args.json else max(2, 1 / args.refresh)
    live = Live(console=console, auto_refresh=False, screen=True) if interactive else None
    result = 0
    try:
        if monitor:
            monitor.start()
        if controller:
            controller.start()
        if live:
            live.start()
        while not stopping.is_set():
            now = time.monotonic()
            snapshot = monitor.snapshot() if monitor else Telemetry(args.port, args.baud).snapshot(now)
            if not monitor:
                snapshot['state'] = 'disabled (--controller-only)'
            host = host_status()
            pad = controller.snapshot() if controller else {'state': 'disabled (--no-controller)', 'input_ready': False}
            if args.json:
                print(json.dumps({"uptime_s": round(now - started, 3), "rover": snapshot, "host": host, "controller": pad}), flush=True)
            else:
                screen = render(snapshot, host, now - started, pad)
                if live:
                    live.update(screen, refresh=True)
                else:
                    console.print(screen)
            remaining = args.duration - (now - started) if args.duration else interval
            if args.duration and remaining <= 0:
                # Every enabled subsystem must be ready at the end of a bounded check.
                result = 0 if ((not monitor or snapshot['state'] == 'telemetry live')
                               and (not controller or pad['input_ready'])) else 2
                break
            if (monitor and not monitor.thread.is_alive()) or (controller and not controller.thread.is_alive()):
                result = 1
                break
            stopping.wait(min(interval, remaining))
    except BrokenPipeError:
        result = 0
    finally:
        if controller:
            controller.close()
        if monitor:
            monitor.close()
        if live:
            live.stop()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
    return result
