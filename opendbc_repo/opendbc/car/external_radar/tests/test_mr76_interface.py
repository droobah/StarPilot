import math

import pytest

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.external_radar.mr76 import (CYCLE_TIMEOUT_NS, MAX_EPISODES, RECOVERY_NS, FaultReason, MR76Config,
                                             MR76RadarInterface, RadarStatus)
from opendbc.dbc.generator.nanoradar.nanoradar_mr76 import dbc_name

FRAME_NS = 10_000_000  # card runs at 100 Hz
CYCLE_FRAMES = 6  # MR76 cycle time is 60 ms
STATE_FRAMES = 100

OBJ = {"ID": 5, "DistLong": 30.0, "DistLat": 2.0, "VrelLong": -1.5, "VrelLat": 0.0, "DynProp": 0, "Class": 1, "RCS": 10.0}


class Sim:
  """Drives the interface at card's 100 Hz with a 60 ms radar cycle and 1 Hz RadarState."""

  def __init__(self, config=None, radar_sensor_id=None, bus=None):
    self.config = config or MR76Config()
    self.ri = MR76RadarInterface(structs.CarParams(), self.config)
    radar_id = self.config.sensor_id if radar_sensor_id is None else radar_sensor_id
    self.packer = CANPacker(dbc_name(radar_id))
    self.radar_id = radar_id
    self.bus = self.config.bus if bus is None else bus
    self.tick = 0
    self.objects = [OBJ]
    self.radar_on = True
    self.state_on = True
    self.state_due = True

  def cycle_frames(self, objects=None):
    objects = self.objects if objects is None else objects
    frames = [self.packer.make_can_msg("ObjListStatus", self.bus, {"NofObjects": len(objects), "MeasCount": self.tick})]
    frames += [self.packer.make_can_msg("ObjGeneral", self.bus, obj) for obj in objects]
    return frames

  def step(self, extra=None):
    self.tick += 1
    frames = []
    if self.radar_on and self.state_on and (self.state_due or self.tick % STATE_FRAMES == 1):
      self.state_due = False
      frames.append(self.packer.make_can_msg("RadarState", self.bus, {"SensorID": self.radar_id}))
    if self.radar_on and self.tick % CYCLE_FRAMES == 0:
      frames += self.cycle_frames()
    frames += extra or []
    return self.ri.update([(self.tick * FRAME_NS, frames)])

  def run(self, n):
    published = [rd for rd in (self.step() for _ in range(n)) if rd is not None]
    return published[-1] if published else None


def assert_no_errors(rd):
  assert not any(rd.errors.to_dict().values())


def test_publishes_at_base_cadence_without_errors():
  sim = Sim()
  results = [sim.step() for _ in range(50)]
  assert [i for i, rd in enumerate(results, 1) if rd is not None] == list(range(5, 51, 5))
  for rd in results:
    if rd is not None:
      assert_no_errors(rd)


def test_searching_until_cycle_and_state_then_active():
  sim = Sim()
  sim.state_on = False
  rd = sim.run(30)
  assert sim.ri.status == RadarStatus.searching and len(rd.points) == 0
  sim.state_on = True
  sim.state_due = True
  rd = sim.run(10)
  assert sim.ri.status == RadarStatus.active and len(rd.points) == 1


def test_point_geometry_offsets_and_sign():
  sim = Sim(MR76Config(long_offset=-1.0, lat_offset=0.25))
  pt = sim.run(20).points[0]
  # MR76 DistLat is positive to the right; openpilot yRel is positive to the left.
  assert pt.dRel == pytest.approx(29.0)
  assert pt.yRel == pytest.approx(-2.0 + 0.25)
  assert pt.vRel == pytest.approx(-1.5)


def test_upside_down_mount_flips_lateral():
  pt = Sim(MR76Config(upside_down=True)).run(20).points[0]
  assert pt.yRel == pytest.approx(2.0)


def test_yaw_rotates_position():
  sim = Sim(MR76Config(yaw_deg=10.0))
  sim.objects = [{**OBJ, "DistLong": 20.0, "DistLat": 0.0, "VrelLong": 0.0}]
  pt = sim.run(20).points[0]
  assert pt.dRel == pytest.approx(20.0 * math.cos(math.radians(10)), abs=1e-3)
  assert pt.yRel == pytest.approx(20.0 * math.sin(math.radians(10)), abs=1e-3)


def test_cycle_split_across_card_frames():
  sim = Sim()
  objects = [{**OBJ, "ID": i, "DistLong": 10.0 + i} for i in range(4)]
  sim.radar_on = False
  sim.step([sim.packer.make_can_msg("RadarState", sim.bus, {"SensorID": sim.radar_id})])
  frames = sim.cycle_frames(objects)
  sim.step(frames[:3])
  assert sim.ri.cycle == []
  sim.step(frames[3:])
  assert [o.object_id for o in sim.ri.cycle] == [0, 1, 2, 3]


def test_incomplete_cycle_is_not_published():
  sim = Sim()
  sim.run(20)
  good = list(sim.ri.cycle)
  partial = sim.cycle_frames([{**OBJ, "ID": 1}, {**OBJ, "ID": 2}])[:-1]
  sim.radar_on = False
  sim.step(partial)
  sim.step(sim.cycle_frames([{**OBJ, "ID": 9}])[:1])
  assert sim.ri.cycle == good
  assert sim.ri.incomplete_cycles == 1


def test_filters():
  sim = Sim(MR76Config(max_lat=3.0, rcs_min=0.0, rcs_max=30.0, max_objects=4))
  sim.objects = [
    {**OBJ, "ID": 1},
    {**OBJ, "ID": 2, "DynProp": 2},  # oncoming
    {**OBJ, "ID": 3, "RCS": -5.0},
    {**OBJ, "ID": 4, "DistLat": 4.0},
    {**OBJ, "ID": 6},  # beyond max_objects
  ]
  assert len(sim.run(20).points) == 1


def test_frames_on_other_buses_are_ignored():
  sim = Sim(MR76Config(bus=1), bus=0)
  assert len(sim.run(40).points) == 0
  assert sim.ri.status == RadarStatus.searching


def test_track_ids_are_never_reused():
  sim = Sim()
  first = sim.run(20).points[0].trackId
  assert sim.run(20).points[0].trackId == first
  sim.objects = []
  sim.run(20)
  sim.objects = [OBJ]
  second = sim.run(20).points[0].trackId
  assert second != first
  sim.objects = [{**OBJ, "DistLong": 60.0}]  # same MR76 ID, different object
  assert sim.run(20).points[0].trackId not in (first, second)


def test_lost_radar_falls_back_to_vision_then_recovers():
  sim = Sim()
  sim.run(20)
  sim.radar_on = False
  rd = sim.run(CYCLE_TIMEOUT_NS // FRAME_NS + 10)
  assert sim.ri.status == RadarStatus.fallback and sim.ri.fault == FaultReason.cycleTimeout
  assert sim.ri.episodes == 1 and len(rd.points) == 0
  assert_no_errors(rd)

  sim.radar_on = True
  sim.state_due = True
  assert len(sim.run(RECOVERY_NS // FRAME_NS - 20).points) == 0
  assert len(sim.run(40).points) == 1
  assert sim.ri.status == RadarStatus.active


def test_missing_radar_state_falls_back():
  sim = Sim()
  sim.run(20)
  sim.state_on = False
  sim.run(320)
  assert sim.ri.status == RadarStatus.fallback and sim.ri.fault == FaultReason.stateTimeout


def test_repeated_faults_disable_for_the_drive():
  sim = Sim()
  for _ in range(MAX_EPISODES):
    sim.state_due = True
    sim.radar_on = True
    sim.run(RECOVERY_NS // FRAME_NS + 40)
    sim.radar_on = False
    sim.run(CYCLE_TIMEOUT_NS // FRAME_NS + 10)
  assert sim.ri.status == RadarStatus.disabled and sim.ri.episodes == MAX_EPISODES
  sim.state_due = True
  sim.radar_on = True
  assert len(sim.run(RECOVERY_NS // FRAME_NS + 100).points) == 0
  assert sim.ri.status == RadarStatus.disabled


def test_radar_at_another_sensor_id_disables():
  sim = Sim(MR76Config(sensor_id=2), radar_sensor_id=0)
  rd = sim.run(200)
  assert sim.ri.status == RadarStatus.disabled and sim.ri.fault == FaultReason.sensorIdMismatch
  assert len(rd.points) == 0
  assert_no_errors(rd)


def test_passive_mode_tracks_health_without_points():
  sim = Sim(MR76Config(publish_points=False))
  rd = sim.run(20)
  assert sim.ri.status == RadarStatus.active and len(sim.ri.cycle) == 1
  assert len(rd.points) == 0


@pytest.mark.parametrize("changes", ({"sensor_id": 8}, {"bus": 128}, {"max_objects": 0}, {"max_objects": 65},
                                     {"long_offset": math.nan}, {"max_lat": 0.0}, {"rcs_min": 5.0, "rcs_max": 1.0}))
def test_invalid_configuration_is_rejected(changes):
  with pytest.raises(ValueError):
    MR76Config(**changes)
