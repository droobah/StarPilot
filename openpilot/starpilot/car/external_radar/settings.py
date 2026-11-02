"""Saved add-on radar configuration, read once at startup and frozen for the drive."""
from dataclasses import dataclass
import json
import math

from opendbc.car.external_radar.mr76 import MR76Config
from openpilot.starpilot.saved_source import read_saved

DOCUMENT_KEY = "ExternalRadarConfig"
MAX_BYTES = 1024
VERSION = 1
MODES = ("off", "passive", "active")
MODELS = ("mr76",)
BUSES = (0, 1, 2)

# document key: (MR76Config field, type, minimum, maximum)
NUMBERS = {
  "sensorId": ("sensor_id", int, 0, 7),
  "longOffset": ("long_offset", float, -3.0, 1.0),
  "latOffset": ("lat_offset", float, -1.0, 1.0),
  "yawDeg": ("yaw_deg", float, -5.0, 5.0),
  "maxLat": ("max_lat", float, 2.0, 20.0),
  "rcsMin": ("rcs_min", float, -64.0, 63.5),
  "rcsMax": ("rcs_max", float, -64.0, 63.5),
  "maxObjects": ("max_objects", int, 1, 64),
}
FLAGS = {"upsideDown": "upside_down", "dropOncoming": "drop_oncoming"}
KEYS = frozenset(("version", "mode", "model", "bus", *NUMBERS, *FLAGS))


@dataclass(frozen=True)
class ExternalRadarSettings:
  mode: str
  config: MR76Config | None
  valid: bool
  sources: tuple = ()

  @property
  def enabled(self) -> bool:
    return self.valid and self.mode != "off" and self.config is not None


def _number(value, kind, low, high):
  if isinstance(value, bool) or not isinstance(value, int | float):
    raise ValueError("Invalid external radar number")
  if kind is int:
    if not isinstance(value, int):
      raise ValueError("Invalid external radar integer")
  elif not math.isfinite(value):
    raise ValueError("Invalid external radar number")
  if not low <= value <= high:
    raise ValueError("External radar setting out of range")
  return kind(value)


def parse(document: bytes) -> tuple[str, MR76Config]:
  def unique_pairs(pairs):
    result = {}
    for key, value in pairs:
      if key in result:
        raise ValueError("Duplicate external radar key")
      result[key] = value
    return result

  value = json.loads(document, object_pairs_hook=unique_pairs)
  if not isinstance(value, dict) or set(value) != KEYS:
    raise ValueError("Invalid external radar configuration")
  if type(value["version"]) is not int or value["version"] != VERSION or value["mode"] not in MODES or value["model"] not in MODELS:
    raise ValueError("Invalid external radar configuration")
  if type(value["bus"]) is not int or value["bus"] not in BUSES or any(type(value[key]) is not bool for key in FLAGS):
    raise ValueError("Invalid external radar configuration")
  fields = {field: _number(value[key], kind, low, high) for key, (field, kind, low, high) in NUMBERS.items()}
  fields |= {field: value[key] for key, field in FLAGS.items()}
  config = MR76Config(bus=value["bus"], publish_points=value["mode"] == "active", **fields)
  return value["mode"], config


def encode(mode: str, config: MR76Config) -> bytes:
  document = {"version": VERSION, "mode": mode, "model": "mr76", "bus": config.bus}
  document |= {key: getattr(config, field) for key, (field, *_) in NUMBERS.items()}
  document |= {key: getattr(config, field) for key, field in FLAGS.items()}
  raw = json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
  parse(raw)
  return raw


def read_external_radar_settings(params) -> ExternalRadarSettings:
  sources = ()
  try:
    document, readable = read_saved(params, DOCUMENT_KEY, MAX_BYTES)
    sources = ((DOCUMENT_KEY, document),)
    if not readable:
      return ExternalRadarSettings("off", None, False, sources)
    if document is None:
      return ExternalRadarSettings("off", None, True, sources)
    mode, config = parse(document)
    return ExternalRadarSettings(mode, config, True, sources)
  except (OSError, TypeError, ValueError, OverflowError):
    return ExternalRadarSettings("off", None, False, sources)
