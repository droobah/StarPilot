import json

import pytest

from opendbc.car.external_radar.mr76 import MR76Config
from openpilot.starpilot.car.external_radar.settings import DOCUMENT_KEY, MAX_BYTES, encode, read_external_radar_settings


class Saved:
  def __init__(self, path):
    self.path = path

  def get_param_path(self, name):
    return str(self.path / name)


def saved(tmp_path, raw=None):
  if raw is not None:
    (tmp_path / DOCUMENT_KEY).write_bytes(raw)
  return read_external_radar_settings(Saved(tmp_path))


def document(**changes):
  value = json.loads(encode("active", MR76Config()))
  value.update(changes)
  return json.dumps(value).encode()


def test_absent_document_is_valid_and_off(tmp_path):
  settings = saved(tmp_path)
  assert settings.valid and settings.mode == "off" and not settings.enabled


@pytest.mark.parametrize("mode, publish", (("passive", False), ("active", True)))
def test_round_trip(tmp_path, mode, publish):
  config = MR76Config(bus=1, sensor_id=2, long_offset=-1.2, lat_offset=0.1, yaw_deg=-1.5, upside_down=True,
                      max_lat=4.0, rcs_min=-5.0, rcs_max=25.0, drop_oncoming=False, max_objects=32)
  settings = saved(tmp_path, encode(mode, config))
  assert settings.valid and settings.enabled and settings.mode == mode
  assert settings.config == MR76Config(**{**config.__dict__, "publish_points": publish})


def test_off_mode_keeps_config_but_is_not_enabled(tmp_path):
  settings = saved(tmp_path, encode("off", MR76Config()))
  assert settings.valid and not settings.enabled


def test_integer_values_for_float_fields_are_accepted(tmp_path):
  assert saved(tmp_path, document(longOffset=-1, yawDeg=0)).valid


@pytest.mark.parametrize("raw", (
  b"not json",
  b"[]",
  document(version=2),
  document(mode="on"),
  document(model="ars408"),
  document(bus=3),
  document(bus=True),
  document(sensorId=8),
  document(sensorId=2.0),
  document(maxObjects=0),
  document(longOffset=1.5),
  document(yawDeg=-6.0),
  document(maxLat=1.0),
  document(rcsMin=10.0, rcsMax=5.0),
  document(upsideDown=1),
  document(latOffset=True),
  document(extra=1),
  b'{"version":1,"version":1}',
))
def test_invalid_documents_are_rejected(tmp_path, raw):
  settings = saved(tmp_path, raw)
  assert not settings.valid and not settings.enabled


def test_missing_key_is_rejected(tmp_path):
  value = json.loads(document())
  del value["yawDeg"]
  assert not saved(tmp_path, json.dumps(value).encode()).valid


def test_non_finite_values_are_rejected(tmp_path):
  for value in (float("nan"), float("inf")):
    raw = document(longOffset=value)
    assert b"NaN" in raw or b"Infinity" in raw
    assert not saved(tmp_path, raw).valid


def test_oversized_document_is_unreadable(tmp_path):
  assert not saved(tmp_path, b" " * (MAX_BYTES + 1)).valid


def test_encode_rejects_out_of_range_values():
  with pytest.raises(ValueError):
    encode("active", MR76Config(yaw_deg=10.0))
  with pytest.raises(ValueError):
    encode("on", MR76Config())
