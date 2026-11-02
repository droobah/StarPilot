import importlib.util
from pathlib import Path

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.dbc.generator.nanoradar.nanoradar_mr76 import MESSAGES, dbc_name

spec = importlib.util.spec_from_file_location("mr76_config", Path(__file__).with_name("mr76_config.py"))
assert spec is not None and spec.loader is not None
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


@pytest.mark.parametrize("message, signals", (("RadarCfg", tool.RADAR_CFG_SIGNALS), ("RadarState", tool.RADAR_STATE_SIGNALS),
                                              ("SwVersion", tool.SW_VERSION_SIGNALS)))
def test_layouts_match_the_dbc_generator(message, signals):
  generator = {name: (lsb, size) for name, lsb, size, *_ in MESSAGES[message][2]}
  assert all(generator[name] == layout for name, layout in signals.items())


@pytest.mark.parametrize("sensor_id", tool.SENSOR_IDS)
@pytest.mark.parametrize("new_id", tool.SENSOR_IDS)
def test_set_id_frame_matches_the_dbc(sensor_id, new_id):
  values = {"SensorID_valid": 1, "SensorID": new_id, "StoreInNVM_valid": 1, "StoreInNVM": 1}
  assert tool.set_id_frame(sensor_id, new_id) == CANPacker(dbc_name(sensor_id)).make_can_msg("RadarCfg", 1, values)[:2]


def test_radar_state_decodes_like_the_dbc():
  values: dict[str, float] = {"SensorID": 5, "MaxDistance": 196, "OutputType": 1, "SortIndex": 2, "RCS_Threshold": 1, "NVMWriteStatus": 1,
            "RadarPower": 3, "CANBaudRate": 0, "NVMReadStatus": 1}
  addr, dat, _ = CANPacker(dbc_name(5)).make_can_msg("RadarState", 1, values)
  parser = CANParser(dbc_name(5), [("RadarState", float("nan"))], 1)
  parser.update([(1, [(addr, dat, 1)])])
  physical = {name: int(parser.vl["RadarState"][name]) for name in tool.RADAR_STATE_SIGNALS}
  assert tool.decode(tool.RADAR_STATE_SIGNALS, dat) == physical | {"MaxDistance": 98}  # the tool keeps raw units (2 m)


def test_driver_quick_config_frame():
  # "Set sensor ID 1 and save" from the MR76_Radar driver's quick commands.
  assert tool.set_id_frame(0, 1) == (0x200, bytes.fromhex("8200000001800000"))


class FakeRadar:
  """An MR76 on the bench: a 60 ms object cycle, RadarState once a second, and the config frame."""

  def __init__(self, sensor_id=0, saves=True):
    self.sensor_id = sensor_id
    self.saves = saves
    self.now = 0.0
    self.sent = []
    self.saved = False
    self.packer = CANPacker(dbc_name(sensor_id))

  def clock(self):
    return self.now

  def recv(self, timeout):
    if timeout <= 0:
      return None
    self.now += 0.06
    if int(self.now / 0.06) % 17 == 0:
      values = {"SensorID": self.sensor_id, "MaxDistance": 196, "OutputType": 1, "NVMWriteStatus": int(self.saved)}
      return self.packer.make_can_msg("RadarState", 0, values)[:2]
    return self.packer.make_can_msg("ObjListStatus", 0, {"NofObjects": 0})[:2]

  def send(self, addr, dat):
    self.sent.append((addr, dat))
    if addr == tool.address(tool.RADAR_CFG, self.sensor_id):
      cfg = tool.decode(tool.RADAR_CFG_SIGNALS, dat)
      if cfg["SensorID_valid"]:
        self.sensor_id = cfg["SensorID"]
        self.packer = CANPacker(dbc_name(self.sensor_id))
      self.saved = bool(self.saves and cfg["StoreInNVM_valid"] and cfg["StoreInNVM"])


def run_set_id(radar, new_id, answer="OK", current=None):
  lines = []
  ok = tool.set_id(radar, new_id, current, confirm=lambda _: answer, out=lines.append, clock=radar.clock)
  return ok, lines


def test_scan_finds_the_radar_and_its_settings():
  radar = FakeRadar(sensor_id=3)
  found = tool.scan(radar, clock=radar.clock)
  assert list(found) == [3] and found[3].state["MaxDistance"] == 98 and "max distance 196 m" in found[3].describe() and found[3].object_cycles > 30


def test_set_id_moves_the_radar_and_checks_it_saved():
  radar = FakeRadar(sensor_id=0)
  ok, lines = run_set_id(radar, 2)
  assert ok and radar.sensor_id == 2 and radar.sent == [(0x200, tool.set_id_frame(0, 2)[1])]
  assert "Done" in lines[-1]


def test_nothing_is_sent_without_ok():
  radar = FakeRadar()
  ok, lines = run_set_id(radar, 2, answer="no")
  assert not ok and radar.sent == [] and lines[-1] == "Nothing sent."


def test_an_unconfirmed_save_is_reported():
  ok, lines = run_set_id(FakeRadar(saves=False), 2)
  assert not ok and "didn't confirm the save" in lines[-1]


def test_no_radar_or_wrong_current_id_sends_nothing():
  class Silent:
    now = 0.0
    sent: list = []

    def clock(self):
      return self.now

    def recv(self, timeout):
      self.now += timeout
      return None

    def send(self, addr, dat):
      self.sent.append((addr, dat))

  silent = Silent()
  ok, lines = run_set_id(silent, 2)
  assert not ok and silent.sent == [] and "No radar heard" in lines[0]
  radar = FakeRadar(sensor_id=0)
  ok, _ = run_set_id(radar, 2, current=4)
  assert not ok and radar.sent == []


def test_already_at_the_new_id():
  radar = FakeRadar(sensor_id=2)
  ok, lines = run_set_id(radar, 2)
  assert ok and radar.sent == [] and "already" in lines[0]
