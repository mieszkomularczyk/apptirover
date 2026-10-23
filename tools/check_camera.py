"""Real video/AI processes and RTSP decoding alongside simulated rover control.

No real UART, Bluetooth or USB camera is opened. Requires camera dependencies,
model files and permission to create processes/listen on a loopback socket.
"""

import argparse
import json
import os
from pathlib import Path
import pty
import select
import signal
import statistics
import subprocess
import sys
import tempfile
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from apptirover.camera import CameraConfig, CameraMonitor
from apptirover.serial_monitor import SerialMonitor
from apptirover.following import FollowConfig


def wait_for(predicate, message, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', help='Optional known person image instead of test pattern')
    parser.add_argument('--follow-check', action='store_true', help='Exercise X following with an image containing one centered person; simulated motors only')
    parser.add_argument('--port', type=int, default=18554)
    args = parser.parse_args()
    if args.follow_check and not args.image:
        parser.error('--follow-check requires --image with one centered person')
    camera = CameraMonitor(CameraConfig(test_pattern=not bool(args.image), image=args.image,
                                       host='127.0.0.1', port=args.port, no_detection=True))
    master, slave = pty.openpty()
    stopping = threading.Event()
    commands = []
    gaps = []
    pad = dict(connected=True, input_ready=True, input_generation=1, worker_age_s=0, link_age_s=0,
               input=dict(sticks=dict(left_y=0, right_x=0), resyncing=False,
                          buttons=[], button_presses={'BTN_NORTH': 0}, dropped=0))

    def rover():
        pending = b''
        while not stopping.is_set():
            if not select.select([master], [], [], .02)[0]:
                continue
            pending += os.read(master, 4096)
            while b'\n' in pending:
                raw, pending = pending.split(b'\n', 1)
                command = json.loads(raw)
                commands.append(command)
                if command['T'] in (126, 130):
                    reply = dict(T=1001, v=12.3, L=0, R=0) if command['T'] == 130 else dict(T=1002, ax=0)
                    os.write(master, json.dumps(reply).encode() + b'\n')

    serial = SerialMonitor(os.ttyname(slave), poll_interval=.1, controller_source=lambda: pad,
                           vision_source=camera.snapshot, follow_config=FollowConfig(desired_width=.08))
    original_step = serial.drive.step
    previous = None

    def measured_step(pad, state, now, vision=None):
        nonlocal previous
        if previous is not None:
            gaps.append(now - previous)
        previous = now
        return original_step(pad, state, now, vision)

    serial.drive.step = measured_step
    fake = threading.Thread(target=rover)
    fake.start()
    serial.start()
    camera.start()
    client = None
    with tempfile.TemporaryFile() as video_log, tempfile.TemporaryFile() as errors:
        try:
            wait_for(lambda: camera.snapshot()['stream_ready'], 'Synthetic stream did not start')
            wait_for(lambda: serial.snapshot()['drive']['armed'], 'Simulated control did not arm')
            pad['input']['sticks']['left_y'] = -.5
            client = subprocess.Popen([
                'ffmpeg', '-hide_banner', '-loglevel', 'error', '-rtsp_transport', 'tcp',
                '-fflags', 'nobuffer', '-flags', 'low_delay', '-probesize', '32', '-analyzeduration', '0',
                '-threads', '1', '-i', camera.url, '-an', '-f', 'null', '-', '-progress', 'pipe:1',
                '-stats_period', '0.2'], stdout=video_log, stderr=errors)

            def decoded():
                lines = os.pread(video_log.fileno(), 1000000, 0).decode().splitlines()
                return max([int(line.split('=')[1]) for line in lines if line.startswith('frame=')] or [0])

            wait_for(lambda: decoded() >= 30, 'RTSP did not decode')
            camera.set_detection(True)
            wait_for(lambda: camera.snapshot()['observation_valid'], 'Detector did not produce observations', 50)
            if args.image:
                assert camera.snapshot()['people'] > 0, camera.snapshot()
            time.sleep(3)
            print('Detection:', json.dumps(camera.snapshot()), flush=True)
            if args.follow_check:
                pad['input']['sticks']['left_y'] = 0
                wait_for(lambda: serial.snapshot()['drive']['left_power'] == 0, 'Neutral did not stop')
                pad['input']['button_presses']['BTN_NORTH'] = 1
                wait_for(lambda: serial.snapshot()['drive']['following']['state'] == 'following',
                         'X did not acquire a person', 5)
                wait_for(lambda: serial.snapshot()['drive']['left_power'] < 0 and serial.snapshot()['drive']['right_power'] < 0,
                         'Following did not reverse for a close person')
                print('Following:', json.dumps(serial.snapshot()['drive']), flush=True)
                pad['input']['sticks']['right_x'] = .5
                wait_for(lambda: serial.snapshot()['drive']['following']['manual_override'], 'Manual input did not take priority')
                assert serial.snapshot()['drive']['following']['enabled']
                pad['input']['sticks']['right_x'] = 0
                wait_for(lambda: serial.snapshot()['drive']['following']['state'] == 'following', 'Neutral did not resume following')
                wait_for(lambda: serial.snapshot()['drive']['left_power'] < 0 and serial.snapshot()['drive']['right_power'] < 0,
                         'Following did not resume reverse after manual input')
                pad['input']['button_presses']['BTN_NORTH'] = 2
                wait_for(lambda: not serial.snapshot()['drive']['following']['enabled'], 'X did not disable following')
                wait_for(lambda: serial.snapshot()['drive']['left_power'] == 0, 'X did not stop motors')
                pad['input']['button_presses']['BTN_NORTH'] = 3
                wait_for(lambda: serial.snapshot()['drive']['following']['state'] == 'following', 'Explicit X did not re-enable', 5)
            first_pid = camera.snapshot()['detector_pid']
            before = decoded()
            camera.set_detection(False)
            assert not camera.snapshot()['observation_valid']
            if args.follow_check:
                wait_for(lambda: not serial.snapshot()['drive']['following']['motion_active'], 'Detection disable did not suspend motion')
                assert serial.snapshot()['drive']['following']['enabled']
                assert serial.snapshot()['drive']['left_power'] == 0
                pad['input']['sticks']['left_y'] = -.5
            wait_for(lambda: camera.snapshot()['detector_pid'] is None, 'Detector did not stop')
            wait_for(lambda: decoded() > before + 30, 'Video interrupted by detection disable')
            camera.set_detection(True)
            wait_for(lambda: camera.snapshot()['observation_valid'], 'Detection did not resume', 50)
            assert camera.snapshot()['detector_pid'] != first_pid
            if args.follow_check:
                assert serial.snapshot()['drive']['following']['enabled'], 'Recovery disabled the X mode'

            before = decoded()
            os.kill(camera.snapshot()['detector_pid'], signal.SIGKILL)
            wait_for(lambda: camera.snapshot()['detection_state'] == 'failed', 'Detector crash not reported')
            assert not camera.snapshot()['observation_valid']
            wait_for(lambda: decoded() > before + 30, 'Detector crash stopped video')
            assert serial.snapshot()['drive']['armed']

            # Kill an entire camera process group, then verify supervised recovery.
            generation = camera.snapshot()['generation']
            os.killpg(camera.process.pid, signal.SIGKILL)
            wait_for(lambda: not camera.snapshot()['stream_ready'], 'Camera crash not reported')
            assert serial.snapshot()['drive']['armed']
            wait_for(lambda: camera.snapshot()['generation'] > generation and camera.snapshot()['ready'],
                     'Camera did not recover', 55)
            assert serial.snapshot()['drive']['armed']
            # An alive but frozen camera must also be detected without blocking control.
            generation = camera.snapshot()['generation']
            os.kill(camera.process.pid, signal.SIGSTOP)
            wait_for(lambda: not camera.snapshot()['stream_ready'], 'Frozen camera remained fresh')
            assert not camera.snapshot()['observation_valid']
            wait_for(lambda: camera.snapshot()['generation'] > generation and camera.snapshot()['ready'],
                     'Frozen camera did not recover', 55)
            assert serial.snapshot()['drive']['armed']
            receiver = subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-rtsp_transport', 'tcp',
                                       '-i', camera.url, '-frames:v', '30', '-an', '-f', 'null', '-'],
                                      capture_output=True, timeout=12)
            assert receiver.returncode == 0, receiver.stderr.decode()
            pad['input']['sticks']['left_y'] = 0
            if args.follow_check:
                wait_for(lambda: serial.snapshot()['drive']['following']['state'] == 'following', 'Following did not resume after recovery', 5)
                wait_for(lambda: serial.snapshot()['drive']['left_power'] < 0 and serial.snapshot()['drive']['right_power'] < 0,
                         'Automatic following did not request reverse after recovery')
                pad['input']['button_presses']['BTN_NORTH'] = 4
                wait_for(lambda: not serial.snapshot()['drive']['following']['enabled'], 'Final X disable failed')
            wait_for(lambda: serial.snapshot()['drive']['left_power'] == 0, 'Neutral did not stop')
            result = serial.snapshot()
            assert result['telemetry_packets'] > 100 and result['invalid_lines'] == 0, result
            assert max(gaps) < .25, f'Control deadline exceeded: {max(gaps):.3f}s'
            print(f'PASS: {decoded()} decoded frames before camera restart; '
                  f'{result["telemetry_packets"]} telemetry replies; '
                  f'control gap median={statistics.median(gaps)*1000:.1f} ms, '
                  f'p99={sorted(gaps)[int(len(gaps)*.99)]*1000:.1f} ms, max={max(gaps)*1000:.1f} ms', flush=True)
        finally:
            serial.close()
            camera.close()
            stopping.set()
            fake.join(timeout=2)
            os.close(master)
            os.close(slave)
            if client:
                client.terminate()
                try:
                    client.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    client.kill()
                    client.wait()
            if sys.exc_info()[0]:
                print(camera.snapshot(), file=sys.stderr)
                print(os.pread(errors.fileno(), 65536, 0).decode(), file=sys.stderr)
    assert not camera.thread.is_alive(), 'Camera supervisor leaked'
    assert commands[-1] == dict(T=1, L=0., R=0.), 'Final stop missing'


if __name__ == '__main__':
    main()
