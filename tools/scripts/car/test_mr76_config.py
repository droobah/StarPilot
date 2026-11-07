import importlib.util
import io
from pathlib import Path

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.dbc.generator.nanoradar.nanoradar_mr76 import MESSAGES, dbc_name

spec = importlib.util.spec_from_file_location("mr76_config", Path(__file__).with_name("mr76_config.py"))
assert spec is not None and spec.loader is not None
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


@pytest.mark.parametrize("message, signals", (("RadarCfg", tool.RADAR_CFG_SIGNALS), ("RadarState", tool.RADAR_STATE_SIGNALS),
                                              ("SwVersion", tool.SW_VERSION_SIGNALS), ("ObjListStatus", tool.OBJ_LIST_SIGNALS),
                                              ("ObjGeneral", tool.OBJ_GENERAL_SIGNALS)))
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


def test_object_scaling_matches_the_dbc_generator():
  generator = {name: (factor, offset) for name, _, _, factor, offset, _ in MESSAGES["ObjGeneral"][2]}
  assert all(generator[name] == scale for name, scale in tool.OBJ_GENERAL_SCALE.items())


def test_real_frames_from_a_bench_radar():
  # Captured from an MR76 on firmware 2.0.5, face up on a table under a ceiling.
  assert tool.decode(tool.OBJ_LIST_SIGNALS, bytes.fromhex("0105060001000000")) == {"NofObjects": 1, "MeasCount": 1286}
  assert tool.decode(tool.SW_VERSION_SIGNALS, bytes.fromhex("0200050100220022")) == {"Major": 2, "Minor": 0, "Patch": 5}
  state = tool.decode(tool.RADAR_STATE_SIGNALS, bytes.fromhex("4014000010c40020"))
  assert (state["SensorID"], state["MaxDistance"] * 2, state["SortIndex"], state["OutputType"]) == (0, 160, 1, 1)


def test_vendor_example_object_frame():
  obj = tool.decode_object(bytes.fromhex("574ec40c7f601880"))
  assert (obj["ID"], obj["DistLong"], obj["DistLat"], obj["VrelLong"]) == (87, pytest.approx(4.0), pytest.approx(2.6), pytest.approx(-0.75))


def test_driver_quick_config_frame():
  # "Set sensor ID 1 and save" from the MR76_Radar driver's quick commands.
  assert tool.set_id_frame(0, 1) == (0x200, bytes.fromhex("8200000001800000"))


class FakeRadar:
  """An MR76 on the bench: a 60 ms object cycle, RadarState once a second, and the config frame."""

  def __init__(self, sensor_id=0, saves=True, period=0.06, counting=True):
    self.sensor_id = sensor_id
    self.saves = saves
    self.period = period
    self.counting = counting
    self.count = 0
    self.now = 0.0
    self.sent = []
    self.saved = False
    self.packer = CANPacker(dbc_name(sensor_id))

  def clock(self):
    return self.now

  def recv(self, timeout):
    if timeout <= 0:
      return None
    self.now += self.period
    self.count += self.counting
    if self.count % 17 == 16:
      values = {"SensorID": self.sensor_id, "MaxDistance": 196, "OutputType": 1, "NVMWriteStatus": int(self.saved)}
      return self.packer.make_can_msg("RadarState", 0, values)[:2]
    return self.packer.make_can_msg("ObjListStatus", 0, {"NofObjects": 0, "MeasCount": self.count})[:2]

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


def test_a_healthy_radar_has_no_warning():
  radar = FakeRadar()
  assert "NOT RUNNING" not in tool.scan(radar, clock=radar.clock)[0].describe()


def test_starved_radar_is_flagged():
  # What an MR76 did on 5 V: one all-zero header every 468 ms, counter stuck, no RadarState.
  radar = FakeRadar(period=0.468, counting=False)
  described = tool.scan(radar, clock=radar.clock)[0].describe()
  assert "NOT RUNNING NORMALLY" in described and "12 V" in described


def test_stuck_counter_is_flagged_even_at_full_rate():
  radar = FakeRadar(counting=False)
  assert "isn't moving" in tool.scan(radar, clock=radar.clock)[0].describe()


def test_watch_shows_objects_moving_first():
  packer = CANPacker(dbc_name(0))

  def obj(i, d, lat, dyn):
    return packer.make_can_msg("ObjGeneral", 0, {"ID": i, "DistLong": d, "DistLat": lat, "DynProp": dyn, "RCS": 5.0})[:2]

  frames = iter([packer.make_can_msg("ObjListStatus", 0, {"NofObjects": 3})[:2], obj(1, 2.4, 0.0, 1), obj(2, 4.0, 1.2, 0), obj(3, 90.0, 0.0, 1)])

  class Bus:
    now = 0.0

    def clock(self):
      return self.now

    def recv(self, timeout):
      self.now += 0.01
      return next(frames, None)

  bus, lines = Bus(), []
  assert tool.watch(bus, 0, seconds=1.0, out=lines.append, clock=bus.clock) == 1
  rows = lines[0].splitlines()
  assert rows[0].startswith("3 objects, 2 within 30 m")
  assert "+1.2" in rows[2] and "moving" in rows[2] and "stationary" in rows[3] and len(rows) == 4


def test_frame_log_saves_every_id_in_candump_format():
  class Bus:
    frames = [(0x60A, bytes.fromhex("0105060001000000")), None, (0x123, b"\x01\x02")]

    def recv(self, timeout):
      return self.frames.pop(0)

  out = io.StringIO()
  log = tool.FrameLog(Bus(), out, clock=lambda: 12.5)
  assert [log.recv(0) for _ in range(3)][2] == (0x123, b"\x01\x02")
  assert out.getvalue() == "(12.500000) can0 60A#0105060001000000\n(12.500000) can0 123#0102\n" and log.frames == 2


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
