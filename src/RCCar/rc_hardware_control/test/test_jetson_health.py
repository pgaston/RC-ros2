"""Jetson health readings from a fake sysfs and procfs tree laid out like the
Orin Nano's. Pure Python; runs on the host.
"""
import pytest

from rc_hardware_control.jetson_health import read_health


def write(root, relative, text):
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text + '\n')


@pytest.fixture
def board(tmp_path):
    sys_root, proc_root = tmp_path / 'sys', tmp_path / 'proc'
    write(sys_root, 'class/hwmon/hwmon0/name', 'pwmfan')
    write(sys_root, 'class/hwmon/hwmon1/name', 'ina3221')
    write(sys_root, 'class/hwmon/hwmon1/in1_label', 'VDD_IN')
    write(sys_root, 'class/hwmon/hwmon1/in1_input', '4984')
    write(sys_root, 'class/hwmon/hwmon1/curr1_input', '1856')
    write(sys_root, 'class/hwmon/hwmon1/curr1_crit_alarm', '0')
    write(sys_root, 'class/hwmon/hwmon3/name', 'soctherm_oc')
    write(sys_root, 'class/hwmon/hwmon3/oc1_event_cnt', '1')
    write(sys_root, 'class/hwmon/hwmon3/oc2_event_cnt', '0')
    write(sys_root, 'class/hwmon/hwmon3/oc3_event_cnt', '2')
    write(sys_root, 'class/hwmon/hwmon3/oc1_throt_en', '1')
    write(sys_root, 'class/thermal/thermal_zone0/type', 'cpu-thermal')
    write(sys_root, 'class/thermal/thermal_zone0/temp', '63625')
    write(sys_root, 'class/thermal/thermal_zone2/type', 'cv0-thermal')   # unreadable on the board
    write(sys_root, 'class/thermal/thermal_zone8/type', 'tj-thermal')
    write(sys_root, 'class/thermal/thermal_zone8/temp', '63781')
    write(sys_root, 'bus/usb/devices/2-1/idVendor', '0bda')
    write(sys_root, 'bus/usb/devices/2-1/idProduct', '0489')
    write(sys_root, 'bus/usb/devices/2-1.1/idVendor', '8086')
    write(sys_root, 'bus/usb/devices/2-1.1/idProduct', '0b3a')
    write(sys_root, 'bus/usb/devices/2-1.1/devnum', '3')
    write(sys_root, 'bus/usb/devices/2-1.1/speed', '5000')
    write(proc_root, 'loadavg', '7.29 5.77 4.38 3/1024 12345')
    return sys_root, proc_root


def test_readings_from_the_orin_nano_layout(board):
    h = read_health(str(board[0]), str(board[1]))
    assert h.vdd_in_v == pytest.approx(4.984)
    assert h.vdd_in_a == pytest.approx(1.856)
    assert h.vdd_in_w == pytest.approx(4.984 * 1.856)
    assert h.oc_events == 3
    assert h.current_alarm is False
    assert h.temps_c == pytest.approx({'cpu-thermal': 63.625, 'tj-thermal': 63.781})
    assert h.load_1min == pytest.approx(7.29)
    assert h.camera == '2-1.1 dev3 5000M'


def test_a_camera_off_the_bus_is_none(board):
    sys_root, proc_root = board
    (sys_root / 'bus/usb/devices/2-1.1/idVendor').write_text('dead\n')
    assert read_health(str(sys_root), str(proc_root)).camera is None


def test_a_board_that_exposes_nothing_reads_as_unknown(tmp_path):
    h = read_health(str(tmp_path / 'sys'), str(tmp_path / 'proc'))
    assert (h.vdd_in_v, h.vdd_in_a, h.vdd_in_w, h.oc_events, h.current_alarm, h.load_1min, h.camera) == \
        (None, None, None, None, None, None, None)
    assert h.temps_c == {}
