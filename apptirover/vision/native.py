"""Load Debian's ABI-matched GI/Cairo without exposing all system Python packages.

The project venv uses /usr/bin/python3. GI and Cairo are supplied by apt, while
AI dependencies stay in the venv. No distro packages enter the control process.
"""

import importlib
from pathlib import Path
import sys


def load_bindings():
    native = str(Path(sys.base_prefix) / 'lib/python3/dist-packages')
    added = native not in sys.path
    if added:
        sys.path.append(native)
    try:
        gi = importlib.import_module('gi')
        importlib.import_module('cairo')
        return gi
    finally:
        if added:
            sys.path.remove(native)
