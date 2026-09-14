"""Jetson power, throttling, temperature and camera USB state, read from sysfs.

For soak tests (issue #14's one-hour bench run). Every reading is optional: a
file that is missing or unreadable gives None rather than an error, so the
recorder keeps going on a board that exposes less.

VDD_IN is the module's input as the carrier board delivers it (about 5 V on
this Orin Nano), not the battery: it sags only when the battery nears the
carrier regulator's dropout. The soctherm over-current counters count
throttling events, which a weak supply triggers before VDD_IN visibly drops.
"""
import os
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

REALSENSE_D435I = ('8086', '0b3a')
TEMPERATURE_ZONES = ('cpu-thermal', 'gpu-thermal', 'tj-thermal')


@dataclass(frozen=True)
class Health:
    vdd_in_v: Optional[float] = None
    vdd_in_a: Optional[float] = None
    oc_events: Optional[int] = None         # sum of the soctherm over-current event counters
    current_alarm: Optional[bool] = None    # VDD_IN over its critical current
    temps_c: Dict[str, float] = field(default_factory=dict)
    load_1min: Optional[float] = None
    camera: Optional[str] = None            # '<bus-port> dev<N> <speed>M', None when not on the bus

    @property
    def vdd_in_w(self) -> Optional[float]:
        if self.vdd_in_v is None or self.vdd_in_a is None:
            return None
        return self.vdd_in_v * self.vdd_in_a


def _read(path: str) -> Optional[str]:
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def _number(path: str, scale: float = 1.0) -> Optional[float]:
    text = _read(path)
    try:
        return float(text) * scale
    except (TypeError, ValueError):
        return None


def _entries(directory: str):
    try:
        return sorted(os.listdir(directory))
    except OSError:
        return []


def read_health(sys_root: str = '/sys', proc_root: str = '/proc',
                camera_id: Tuple[str, str] = REALSENSE_D435I) -> Health:
    values = {'temps_c': {}}
    hwmon = os.path.join(sys_root, 'class', 'hwmon')
    for entry in _entries(hwmon):
        base = os.path.join(hwmon, entry)
        name = _read(os.path.join(base, 'name'))
        if name == 'ina3221' and _read(os.path.join(base, 'in1_label')) == 'VDD_IN':
            values['vdd_in_v'] = _number(os.path.join(base, 'in1_input'), 1e-3)
            values['vdd_in_a'] = _number(os.path.join(base, 'curr1_input'), 1e-3)
            alarm = _number(os.path.join(base, 'curr1_crit_alarm'))
            values['current_alarm'] = None if alarm is None else alarm != 0
        elif name == 'soctherm_oc':
            counts = [_number(os.path.join(base, f)) for f in _entries(base)
                      if f.startswith('oc') and f.endswith('_event_cnt')]
            counts = [c for c in counts if c is not None]
            values['oc_events'] = int(sum(counts)) if counts else None

    thermal = os.path.join(sys_root, 'class', 'thermal')
    for entry in _entries(thermal):
        zone = _read(os.path.join(thermal, entry, 'type'))
        if zone in TEMPERATURE_ZONES:
            temp = _number(os.path.join(thermal, entry, 'temp'), 1e-3)
            if temp is not None:
                values['temps_c'][zone] = temp

    loadavg = _read(os.path.join(proc_root, 'loadavg'))
    if loadavg:
        values['load_1min'] = float(loadavg.split()[0])

    usb = os.path.join(sys_root, 'bus', 'usb', 'devices')
    for entry in _entries(usb):
        base = os.path.join(usb, entry)
        if (_read(os.path.join(base, 'idVendor')), _read(os.path.join(base, 'idProduct'))) == camera_id:
            values['camera'] = (f"{entry} dev{_read(os.path.join(base, 'devnum'))} "
                                f"{_read(os.path.join(base, 'speed'))}M")
            break

    return Health(**values)
