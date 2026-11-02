import json
from pathlib import Path

import pytest

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.external_radar.mr76 import FaultReason, MR76Config, MR76RadarInterface, RadarStatus
from opendbc.car.external_radar.startup import ineligible_reason, mr76_collisions, occupied_addresses, radar_sensor_ids
from opendbc.dbc.generator.nanoradar.nanoradar_mr76 import SENSOR_IDS, dbc_name

FIXTURE = Path(__file__).parent / "fixtures" / "chevrolet_bolt_2020_pedal_can.json"


@pytest.fixture(scope="module")
def bolt():
  doc = json.loads(FIXTURE.read_text())
  return {int(bus): {int(addr): size for addr, size in addrs.items()} for bus, addrs in doc["fingerprint"].items()}


def car_params(**changes):
  cp = structs.CarParams()
  cp.openpilotLongitudinalControl = True
  cp.radarUnavailable = True
  for key, value in changes.items():
    setattr(cp, key, value)
  return cp


def test_eligible_car():
  assert ineligible_reason(car_params()) is None


@pytest.mark.parametrize("changes", ({"openpilotLongitudinalControl": False}, {"radarUnavailable": False},
                                     {"passive": True}, {"dashcamOnly": True}, {"notCar": True}))
def test_ineligible_cars(changes):
  assert ineligible_reason(car_params(**changes)) is not None


def test_bolt_spare_bus_is_free_for_every_sensor_id(bolt):
  assert 1 not in bolt
  assert all(mr76_collisions(bolt, 1, sensor_id) == () for sensor_id in SENSOR_IDS)


def test_bolt_powertrain_bus_only_leaves_sensor_id_2(bolt):
  # Includes driving-only traffic (0x241, 0x250, 0x451) that a parked capture misses.
  clean = [sensor_id for sensor_id in SENSOR_IDS if not mr76_collisions(bolt, 0, sensor_id)]
  assert clean == [2]
  assert (0x201, "RadarState") in mr76_collisions(bolt, 0, 0)  # comma pedal GAS_SENSOR
  assert (0x241, "RadarState") in mr76_collisions(bolt, 0, 4)
  assert {0x250, 0x451} <= {addr for addr, _ in mr76_collisions(bolt, 0, 5)}


def test_car_traffic_at_another_sensor_id_is_not_taken_for_a_radar():
  # A car module sending 0x60A on the radar's bus must not trigger the sensor-ID mismatch.
  packer = CANPacker(dbc_name(0))
  fingerprints = {1: {0x60A: 8}}
  for occupied, expected in ((occupied_addresses(fingerprints, 1), RadarStatus.searching), (frozenset(), RadarStatus.disabled)):
    ri = MR76RadarInterface(structs.CarParams(), MR76Config(sensor_id=2), occupied)
    for tick in range(1, 40):
      ri.update([(tick * 10_000_000, [packer.make_can_msg("ObjListStatus", 1, {"NofObjects": 0})])])
    assert ri.status == expected


def radar_cycle(sensor_id, extra=None):
  """The fingerprint when an MR76 at sensor_id powered up with the car."""
  shift = 0x10 * sensor_id
  return {1: {0x201 + shift: 8, 0x60A + shift: 8, 0x60B + shift: 8, **(extra or {})}}


def test_the_radar_itself_is_not_a_collision():
  assert radar_sensor_ids(radar_cycle(2), 1) == {2}
  assert mr76_collisions(radar_cycle(2), 1, 2) == ()
  assert occupied_addresses(radar_cycle(2, {0x123: 8}), 1) == {0x123}


def test_a_car_module_on_a_radar_input_still_collides_with_the_radar_present():
  # 0x220 would reconfigure the radar, and the radar never sends it.
  assert mr76_collisions(radar_cycle(2, {0x220: 8}), 1, 2) == ((0x220, "RadarCfg (radar input)"),)


def test_one_matching_address_is_car_traffic_not_a_radar():
  assert radar_sensor_ids({1: {0x62A: 8}}, 1) == frozenset()
  assert radar_sensor_ids({1: {0x62A: 8, 0x62B: 5}}, 1) == frozenset()
  assert mr76_collisions({1: {0x62A: 8}}, 1, 2) == ((0x62A, "ObjListStatus"),)


def test_a_radar_seen_at_another_sensor_id_at_startup_is_still_a_mismatch():
  fingerprints = radar_cycle(0)
  assert mr76_collisions(fingerprints, 1, 2) == ()
  packer = CANPacker(dbc_name(0))
  ri = MR76RadarInterface(structs.CarParams(), MR76Config(sensor_id=2), occupied_addresses(fingerprints, 1))
  for tick in range(1, 40):
    ri.update([(tick * 10_000_000, [packer.make_can_msg("ObjListStatus", 1, {"NofObjects": 0})])])
  assert (ri.status, ri.fault) == (RadarStatus.disabled, FaultReason.sensorIdMismatch)
