"""Fresh camera interpreter; it inherits only the small IPC socket, never UART."""

import json
import os
from pathlib import Path
import socket
import sys
from types import SimpleNamespace

from .lifecycle import die_with_parent


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    channel = socket.socket(fileno=int(argv[0]))
    channel.setblocking(False)
    try:
        die_with_parent(int(argv[2]))
        os.nice(5)
        config = json.loads(argv[1])
        config['model'] = Path(config['model'])
        config['image'] = Path(config['image']) if config['image'] else None
        from .stream import CameraServer
        return CameraServer(SimpleNamespace(**config), channel).run()
    except Exception as exc:
        try:
            channel.send(json.dumps(dict(type='status', state='failed', stream_ready=False,
                                        last_error=f'{type(exc).__name__}: {exc}')).encode())
        except OSError:
            pass
        return 1
    finally:
        channel.close()


if __name__ == '__main__':
    raise SystemExit(main())
