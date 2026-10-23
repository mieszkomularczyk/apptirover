"""One-file ARM64 build; run through tools/build_executable.py."""

from pathlib import Path

root = Path(SPECPATH).parent
model = root / 'models' / 'yolo26n_ncnn_model'

analysis = Analysis(
    [str(root / 'main.py')],
    pathex=[str(root), str(root / 'build' / 'native-bindings')],
    datas=[(str(model / filename), 'models/yolo26n_ncnn_model')
           for filename in ('metadata.yaml', 'model.ncnn.param', 'model.ncnn.bin')],
    hiddenimports=[
        'apptirover.vision.worker', 'apptirover.vision.stream',
        'gi', 'cairo', 'gi._gi_cairo', 'gi.repository.Gst',
        'gi.repository.GstApp', 'gi.repository.GstVideo', 'gi.repository.GstRtspServer',
    ],
    hooksconfig={
        'gi': {'languages': [], 'icons': [], 'themes': []},
        'gstreamer': {'include_plugins': [
            'app', 'cairo', 'coreelements', 'imagefreeze', 'jpeg', 'png',
            'playback', 'rtp', 'rtpmanager', 'rtsp', 'tcp', 'typefindfunctions',
            'udp', 'video4linux2', 'videoconvertscale', 'videoparsersbad',
            'videorate', 'videotestsrc', 'x264',
        ]},
    },
    excludes=['tkinter', 'matplotlib', 'torch', 'torchvision', 'ultralytics',
              'PyQt5', 'PyQt6', 'PySide2', 'PySide6'],
)
archive = PYZ(analysis.pure)
executable = EXE(
    archive, analysis.scripts, analysis.binaries, analysis.datas, [],
    name='apptirover', console=True, debug=False, strip=False, upx=False,
)
