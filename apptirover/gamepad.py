"""Linux gamepad readings, independent of rover motion or Bluetooth enrollment."""

from evdev import ecodes


def code_name(kind, code):
    name = ecodes.bytype.get(kind, {}).get(code, str(code))
    return name[0] if isinstance(name, (list, tuple)) else name


def normalize(value, minimum, maximum, deadzone=0.08):
    """Map the kernel axis range to [-1, 1], with a rescaled center deadzone."""
    if maximum <= minimum:
        return 0.0
    value = max(-1.0, min(1.0, 2 * (value - minimum) / (maximum - minimum) - 1))
    if abs(value) <= deadzone:
        return 0.0
    return round((abs(value) - deadzone) / (1 - deadzone) * (1 if value > 0 else -1), 4)


def stick_axes(capabilities):
    axes = set(capabilities.get(ecodes.EV_ABS, []))
    if not {ecodes.ABS_X, ecodes.ABS_Y} <= axes:
        return None
    for right in ((ecodes.ABS_RX, ecodes.ABS_RY), (ecodes.ABS_Z, ecodes.ABS_RZ)):
        if set(right) <= axes:
            return (ecodes.ABS_X, ecodes.ABS_Y, *right)
    return None


def matches_input(device, address):
    return (device.info.bustype == ecodes.BUS_BLUETOOTH
            and device.uniq.upper() == address.upper()
            and stick_axes(device.capabilities(absinfo=False)) is not None)


class InputState:
    def __init__(self, device):
        self.mapping = stick_axes(device.capabilities(absinfo=False))
        self.axes = {}
        self.buttons = set()
        self.events = 0
        self.dropped = 0
        self.resyncing = False
        self.last_event_at = None
        self.last_event = ''
        self.resync(device)

    def resync(self, device):
        for code in device.capabilities(absinfo=False).get(ecodes.EV_ABS, []):
            info = device.absinfo(code)
            old = self.axes.get(code, {})
            self.axes[code] = dict(value=info.value, min=info.min, max=info.max,
                                   flat=info.flat, fuzz=info.fuzz,
                                   observed_min=min(old.get('observed_min', info.value), info.value),
                                   observed_max=max(old.get('observed_max', info.value), info.value))
        self.buttons = set(device.active_keys())

    def accept(self, event, now, device):
        if event.type == ecodes.EV_SYN:
            if event.code == ecodes.SYN_DROPPED:
                self.dropped += 1
                self.resyncing = True
                return False
            if event.code == ecodes.SYN_REPORT:
                if self.resyncing:
                    self.resync(device)
                    self.resyncing = False
                return True
            return False
        if self.resyncing or event.type not in (ecodes.EV_ABS, ecodes.EV_KEY):
            return False
        self.events += 1
        self.last_event_at = now
        self.last_event = f'{code_name(event.type, event.code)}={event.value}'
        if event.type == ecodes.EV_ABS and event.code in self.axes:
            axis = self.axes[event.code]
            axis['value'] = event.value
            axis['observed_min'] = min(axis['observed_min'], event.value)
            axis['observed_max'] = max(axis['observed_max'], event.value)
        elif event.type == ecodes.EV_KEY:
            if event.value:
                self.buttons.add(event.code)
            else:
                self.buttons.discard(event.code)
        return False

    def snapshot(self):
        sticks = {name: normalize(self.axes[code]['value'], self.axes[code]['min'], self.axes[code]['max'])
                  for name, code in zip(('left_x', 'left_y', 'right_x', 'right_y'), self.mapping)}
        return dict(sticks=sticks, axes={code_name(ecodes.EV_ABS, code): dict(axis)
                                      for code, axis in self.axes.items()},
                    buttons=sorted(code_name(ecodes.EV_KEY, code) for code in self.buttons),
                    events=self.events, dropped=self.dropped,
                    last_event=self.last_event, last_event_at=self.last_event_at)
