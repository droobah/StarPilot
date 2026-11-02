import json
from pathlib import Path
from unittest import mock

import pytest

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.external_radar import tests as opendbc_tests
from opendbc.car.external_radar.mr76 import MR76Config, MR76RadarInterface, RadarStatus
from opendbc.dbc.generator.nanoradar.nanoradar_mr76 import dbc_name
from openpilot.cereal import messaging
from openpilot.starpilot.car.external_radar import runtime
from openpilot.starpilot.car.external_radar.settings import DOCUMENT_KEY, encode

FIXTURE = Path(opendbc_tests.__file__).parent / "fixtures" / "chevrolet_bolt_2020_pedal_can.json"


class Saved:
  def __init__(self, path):
    self.path = path

  def get_param_path(self, name):
    return str(self.path / name)


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


def install(tmp_path, fingerprints, raw=None, cp=None, enabled=True):
  if raw is not None:
    (tmp_path / DOCUMENT_KEY).write_bytes(raw)
  stock = object()
  ri, external = runtime.install(stock, cp or car_params(), Saved(tmp_path), fingerprints, enabled=enabled)
  return stock, ri, external


def status(external):
  msg = messaging.new_message("starpilotCarState")
  external.write(msg.starpilotCarState.externalRadar)
  return msg.as_reader().starpilotCarState.externalRadar


def test_no_document_keeps_the_car_radar_interface(tmp_path, bolt):
  stock, ri, external = install(tmp_path, bolt)
  assert ri is stock and external is None


@pytest.mark.parametrize("mode", ("off", "passive", "active"))
def test_openpilot_disabled_never_installs(tmp_path, bolt, mode):
  stock, ri, external = install(tmp_path, bolt, encode(mode, MR76Config()), enabled=False)
  assert ri is stock and external is None


def test_off_mode_keeps_the_car_radar_interface(tmp_path, bolt):
  stock, ri, external = install(tmp_path, bolt, encode("off", MR76Config()))
  assert ri is stock and external is None


@pytest.mark.parametrize("mode, passive", (("passive", True), ("active", False)))
def test_bolt_spare_bus_installs_the_mr76(tmp_path, bolt, mode, passive):
  _, ri, external = install(tmp_path, bolt, encode(mode, MR76Config(bus=1, sensor_id=2)))
  assert type(ri) is MR76RadarInterface and external.ri is ri
  assert ri.config.publish_points is not passive
  reading = status(external)
  assert str(reading.status) == "searching" and reading.passive is passive and reading.blockedReason == ""


def test_invalid_document_is_blocked(tmp_path, bolt):
  stock, ri, external = install(tmp_path, bolt, b"not json")
  assert ri is stock
  reading = status(external)
  assert str(reading.status) == "blocked" and "invalid" in reading.blockedReason


@pytest.mark.parametrize("changes", ({"openpilotLongitudinalControl": False}, {"radarUnavailable": False}, {"dashcamOnly": True}))
def test_ineligible_car_is_blocked(tmp_path, bolt, changes):
  stock, ri, external = install(tmp_path, bolt, encode("active", MR76Config()), cp=car_params(**changes))
  assert ri is stock and str(status(external).status) == "blocked"


def test_address_collision_is_blocked(tmp_path, bolt):
  # Sensor ID 0 on the Bolt's powertrain bus collides with the comma pedal's 0x201.
  stock, ri, external = install(tmp_path, bolt, encode("active", MR76Config(bus=0, sensor_id=0)))
  assert ri is stock
  reading = status(external)
  assert str(reading.status) == "blocked" and "0x201" in reading.blockedReason and "bus 0" in reading.blockedReason


def test_startup_failure_keeps_the_car_radar_interface(tmp_path, bolt):
  with mock.patch.object(runtime, "MR76RadarInterface", side_effect=RuntimeError("no DBC")):
    stock, ri, external = install(tmp_path, bolt, encode("active", MR76Config()))
  assert ri is stock and str(status(external).status) == "blocked"


def test_status_follows_the_radar():
  ri = MR76RadarInterface(car_params(), MR76Config(sensor_id=2))
  external = runtime.ExternalRadar(ri)
  packer = CANPacker(dbc_name(2))

  def cycle(tick, objects):
    frames = [packer.make_can_msg("ObjListStatus", 1, {"NofObjects": len(objects)})]
    frames += [packer.make_can_msg("ObjGeneral", 1, {"ID": i, "DistLong": d, "RCS": 10.0}) for i, d in enumerate(objects)]
    if tick == 6:
      frames.append(packer.make_can_msg("RadarState", 1, {"SensorID": 2}))
    ri.update([(tick * 10_000_000, frames if tick % 6 == 0 else [])])

  for tick in range(1, 20):
    cycle(tick, [20.0, 40.0])
  reading = status(external)
  assert str(reading.status) == "active" and reading.objectCount == 2 and str(reading.faultReason) == "none"

  for tick in range(20, 60):
    ri.update([(tick * 10_000_000, [])])
  assert ri.status == RadarStatus.fallback
  reading = status(external)
  assert str(reading.status) == "fallback" and str(reading.faultReason) == "cycleTimeout" and reading.faultCount == 1
