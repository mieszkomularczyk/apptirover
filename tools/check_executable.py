#!/usr/bin/env python3
"""Check a copied standalone binary with RTSP/NCNN and simulated camera input.

No rover UART, Bluetooth controller or USB camera is opened.
"""

import argparse
from collections import deque
import json
import os
from pathlib import Path
import pty
import select
import shutil
import signal
import subprocess
import tempfile
import threading
import time


def check_telemetry(binary, directory, environment):
    """Exercise the frozen serial adapter and rendered CLI on a pseudo-UART."""
    master, slave = pty.openpty()
    stopping = threading.Event()
    commands = []

    def board():
        pending = b''
        while not stopping.is_set():
            if not select.select([master], [], [], .02)[0]:
                continue
            pending += os.read(master, 4096)
            while b'\n' in pending:
                line, pending = pending.split(b'\n', 1)
                command = json.loads(line)
                commands.append(command)
                if command['T'] in (126, 130):
                    reply = dict(T=1001, v=12.3, L=0, R=0) if command['T'] == 130 else dict(T=1002, ax=0)
                    os.write(master, json.dumps(reply).encode() + b'\n')

    worker = threading.Thread(target=board)
    worker.start()
    try:
        result = subprocess.run([
            str(binary), '--no-controller', '--no-camera', '--port', os.ttyname(slave),
            '--poll-interval', '.1', '--duration', '2', '--plain',
        ], cwd=directory, env=environment, capture_output=True, text=True, timeout=15)
        assert result.returncode == 0, result.stdout + result.stderr
        assert 'ffplay-rtsp_transporttcp' in ''.join(result.stdout.split())
        assert len(commands) > 10 and all(command['T'] in (126, 130) for command in commands), commands
    finally:
        stopping.set()
        worker.join(timeout=1)
        os.close(master)
        os.close(slave)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('executable', nargs='?', default=str(Path(__file__).resolve().parents[1] / 'dist/apptirover'))
    parser.add_argument('--image', help='Optional known person image; require a detected person when supplied')
    parser.add_argument('--port', type=int, default=18555)
    args = parser.parse_args()
    executable = Path(args.executable).resolve()
    latest, output = {}, deque(maxlen=10)
    process = None

    with tempfile.TemporaryDirectory(prefix='apptirover-standalone-') as directory, tempfile.TemporaryFile() as errors:
        directory = Path(directory)
        binary = directory / 'apptirover'
        shutil.copy2(executable, binary)
        source = ['--camera-test-pattern']
        if args.image:
            image = directory / ('person' + Path(args.image).suffix)
            shutil.copy2(args.image, image)
            source = ['--camera-image', str(image)]
        environment = {key: value for key, value in os.environ.items()
                       if not key.startswith(('PYTHON', 'GST_', 'GI_', 'LD_LIBRARY_PATH'))}
        environment.update(PATH='/usr/bin:/bin', PYTHONNOUSERSITE='1')

        def read_output():
            for line in process.stdout:
                output.append(line.rstrip())
                try:
                    latest['state'] = json.loads(line)
                except json.JSONDecodeError:
                    pass

        def camera():
            return latest.get('state', {}).get('camera', {})

        def wait_for(predicate, description, timeout=30):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if predicate():
                    return
                if process.poll() is not None:
                    break
                time.sleep(.05)
            detail = os.pread(errors.fileno(), 20000, 0).decode(errors='replace')
            raise AssertionError(f'{description}\nCamera: {camera()}\nOutput: {list(output)}\n{detail}')

        try:
            help_result = subprocess.run([str(binary), '--help'], cwd=directory, env=environment,
                                         capture_output=True, text=True, timeout=30)
            assert help_result.returncode == 0, help_result.stderr
            assert '--follow-pulse-ms' in help_result.stdout
            check_telemetry(binary, directory, environment)
            process = subprocess.Popen([
                str(binary), '--camera-only', *source, '--rtsp-host', '127.0.0.1',
                '--stream-host', '127.0.0.1', '--rtsp-port', str(args.port),
                '--json', '--refresh', '10', '--duration', '90',
            ], cwd=directory, env=environment, stdout=subprocess.PIPE, stderr=errors,
                text=True, start_new_session=True)
            reader = threading.Thread(target=read_output, daemon=True)
            reader.start()
            wait_for(lambda: camera().get('observation_valid'), 'Bundled detector did not produce a fresh observation')
            if args.image:
                wait_for(lambda: (camera().get('people') or 0) > 0, 'Bundled detector did not detect the fixture person')
            assert latest['state']['rover']['state'] == 'disabled'
            assert not latest['state']['controller']['input_ready']
            first_detector = camera()['detector_pid']
            decoder = subprocess.run([
                '/usr/bin/ffmpeg', '-hide_banner', '-loglevel', 'error', '-rtsp_transport', 'tcp',
                '-i', camera()['rtsp_url'], '-frames:v', '30', '-an', '-f', 'null', '-',
            ], env=environment, capture_output=True, timeout=15)
            assert decoder.returncode == 0, decoder.stderr.decode(errors='replace')
            process.send_signal(signal.SIGUSR1)
            wait_for(lambda: not camera().get('detection_enabled') and camera().get('stream_ready')
                     and camera().get('detector_pid') is None, 'Detection toggle did not preserve streaming')
            process.send_signal(signal.SIGUSR1)
            wait_for(lambda: camera().get('observation_valid') and camera().get('detector_pid') != first_detector,
                     'Bundled detector did not restart after toggle')
            detector = camera()['detector_pid']
            os.kill(detector, signal.SIGKILL)
            wait_for(lambda: camera().get('detection_state') == 'failed', 'Detector crash was not reported')
            assert camera()['stream_ready']
            # Re-enable with a new revision after a failed detector.
            process.send_signal(signal.SIGUSR1)
            wait_for(lambda: not camera().get('detection_enabled'), 'Could not disable failed detector')
            process.send_signal(signal.SIGUSR1)
            wait_for(lambda: camera().get('observation_valid'), 'Bundled detector did not recover')
            final_detector = camera()['detector_pid']
            process.terminate()
            assert process.wait(timeout=10) == 0
            reader.join(timeout=2)
            assert not Path(f'/proc/{final_detector}').exists(), 'Detector survived normal shutdown'
            print('PASS: copied binary without project/venv, pseudo-UART telemetry and rendered CLI, '
                  'bundled model inference, 30 RTSP frames, '
                  'detection toggles, detector failure/recovery and clean shutdown; no hardware opened')
        finally:
            if process is not None:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)
                if process.stdout:
                    process.stdout.close()


if __name__ == '__main__':
    main()
