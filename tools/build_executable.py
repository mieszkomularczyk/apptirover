#!/usr/bin/env python3
"""Build dist/apptirover with Python, the model and native camera dependencies."""

import hashlib
from importlib.util import find_spec
import os
from pathlib import Path
import platform
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    if sys.platform != 'linux' or platform.machine() not in ('aarch64', 'arm64'):
        raise SystemExit('Build on the target ARM64 Linux/Pi OS platform.')
    if find_spec('PyInstaller') is None:
        raise SystemExit('Install requirements-build.txt into the project venv first.')
    model = root / 'models/yolo26n_ncnn_model'
    for filename in ('metadata.yaml', 'model.ncnn.param', 'model.ncnn.bin'):
        if not (model / filename).is_file():
            raise SystemExit(f'Missing {model / filename}; export the detection model first.')

    # Expose only the two Debian native bindings and their metadata to Analysis.
    # Adding all distro site-packages could shadow the venv's NumPy/OpenCV/NCNN.
    native = Path(sys.base_prefix) / 'lib/python3/dist-packages'
    bindings = root / 'build/native-bindings'
    bindings.mkdir(parents=True, exist_ok=True)
    for package in ('gi', 'cairo'):
        if not (native / package).is_dir():
            raise SystemExit(f'Missing {native / package}; install the GI/Cairo OS dependencies.')
    sources = [native / 'gi', native / 'cairo',
               *native.glob('PyGObject*.egg-info'), *native.glob('pycairo*.egg-info'),
               *native.glob('PyGObject*.dist-info'), *native.glob('pycairo*.dist-info')]
    for source in sources:
        target = bindings / source.name
        if not target.is_symlink():
            target.symlink_to(source, target_is_directory=source.is_dir())

    environment = os.environ.copy()
    environment['PYTHONPATH'] = os.pathsep.join((str(root), str(bindings)))
    environment['PYINSTALLER_CONFIG_DIR'] = str(root / '.cache/pyinstaller')
    subprocess.run([
        sys.executable, '-m', 'PyInstaller', '--clean', '--noconfirm',
        '--distpath', str(root / 'dist'), '--workpath', str(root / 'build/pyinstaller'),
        str(root / 'packaging/apptirover.spec'),
    ], cwd=root, env=environment, check=True)
    binary = root / 'dist/apptirover'
    with binary.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    print(f'Built {binary} ({binary.stat().st_size / 1024**2:.1f} MiB)\nSHA256 {digest}')


if __name__ == '__main__':
    main()
