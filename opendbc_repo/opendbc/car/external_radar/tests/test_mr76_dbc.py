import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.dbc.generator.nanoradar.nanoradar_mr76 import SENSOR_IDS, address, dbc_name, motorola_msb


def test_motorola_start_bits_match_dbc_convention():
  # DistLong, DistLat, VrelLat and MaxDistance cross byte boundaries.
  assert motorola_msb(19, 13) == 15
  assert motorola_msb(24, 11) == 18
  assert motorola_msb(53, 9) == 45
  assert motorola_msb(22, 10) == 15
  assert motorola_msb(39, 3) == 25
  assert motorola_msb(0, 8) == 7


def test_vendor_example_object_frame():
  # Example frame from the MR76 protocol document, as quoted by the MR76_Radar driver.
  parser = CANParser(dbc_name(0), [("ObjGeneral", float("nan"))], 1)
  parser.update([(1, [(0x60B, bytes.fromhex("574ec40c7f601880"), 1)])])
  obj = parser.vl["ObjGeneral"]
  assert obj["ID"] == 87
  assert obj["DistLong"] == pytest.approx(4.0)
  assert obj["DistLat"] == pytest.approx(2.6)
  assert obj["VrelLong"] == pytest.approx(-0.75)


@pytest.mark.parametrize("values, expected", [
  # Quick-config frames listed in the MR76_Radar driver.
  ({"SensorID_valid": 1, "SensorID": 1, "StoreInNVM_valid": 1, "StoreInNVM": 1}, "8200000001800000"),
  ({"StoreInNVM_valid": 1, "StoreInNVM": 1, "RCS_Threshold_valid": 1, "RCS_Threshold": 1}, "8000000000800300"),
  ({"StoreInNVM_valid": 1, "StoreInNVM": 1, "Calibration_valid": 1, "Calibration": 1}, "800000000080000a"),
])
def test_config_frames_match_driver_quick_commands(values, expected):
  addr, dat, bus = CANPacker(dbc_name(0)).make_can_msg("RadarCfg", 1, values)
  assert (addr, bus) == (0x200, 1)
  assert dat.hex() == expected


@pytest.mark.parametrize("sensor_id", SENSOR_IDS)
def test_sensor_id_shifts_every_address(sensor_id):
  parser = CANParser(dbc_name(sensor_id), [(name, float("nan")) for name in ("RadarState", "ObjListStatus", "ObjGeneral", "SwVersion")], 1)
  shift = 0x10 * sensor_id
  assert parser.addresses == {0x201 + shift, 0x60A + shift, 0x60B + shift, 0x700 + shift}
  assert address(0x60B, sensor_id) == 0x60B + shift
