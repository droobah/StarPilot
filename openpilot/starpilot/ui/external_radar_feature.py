"""Galaxy setup for an add-on radar: one entry on Vehicle settings and its own page.

Hidden until the driver opts in on the Developer page, so cars that could take an add-on
radar don't all grow an experimental entry. Once a setup is saved it stays visible and the
opt-in stays on, so the radar's settings can't be hidden while they exist.
"""
from collections.abc import Callable
from dataclasses import replace
import math

from opendbc.car.external_radar.mr76 import MR76Config
from opendbc.car.external_radar.startup import ineligible_reason
from openpilot.starpilot.car.external_radar.settings import BUSES, DOCUMENT_KEY, MAX_BYTES, NUMBERS, encode, read_external_radar_settings
from openpilot.starpilot.longitudinal.voacc import KEY as VOACC_KEY
from openpilot.starpilot.saved_document import commit_exact
from openpilot.starpilot.saved_source import read_saved
from openpilot.starpilot.ui.feature_settings_state import FeaturePage, FeatureRow

SETUP_KEY = "ExternalRadarSetup"
SETUP_DESCRIPTION = ("Experimental. Show External Radar under Vehicle settings to set up an add-on Nanoradar MR76. " +
                     "It appears only for cars with openpilot speed control and no factory radar in use.")
PREFIX = "external_radar:"
MODE = PREFIX + "mode"
BUS = PREFIX + "bus"
SENSOR_ID = PREFIX + "sensor_id"
MODES = {"Off": "off", "Passive": "passive", "Active": "active"}
SWITCHES = {PREFIX + "upside_down": ("upside_down", "Mounted Upside Down"),
            PREFIX + "drop_oncoming": ("drop_oncoming", "Ignore Oncoming Traffic")}
BOUNDS = {field: (low, high) for field, _, low, high in NUMBERS.values()}
# key: (MR76Config field, label, step, unit)
NUMBER_ROWS = {
  PREFIX + "long_offset": ("long_offset", "Forward Offset", 0.05, "m"),
  PREFIX + "lat_offset": ("lat_offset", "Lateral Offset", 0.01, "m"),
  PREFIX + "yaw_deg": ("yaw_deg", "Yaw", 0.1, "deg"),
  PREFIX + "max_lat": ("max_lat", "Maximum Lateral Distance", 0.5, "m"),
  PREFIX + "rcs_min": ("rcs_min", "Minimum Reflection Strength", 0.5, "dBsm"),
  PREFIX + "rcs_max": ("rcs_max", "Maximum Reflection Strength", 0.5, "dBsm"),
  PREFIX + "max_objects": ("max_objects", "Maximum Objects", 1.0, ""),
}
REMOVE = PREFIX + "remove"
KEYS = frozenset((MODE, BUS, SENSOR_ID, *SWITCHES, *NUMBER_ROWS, REMOVE))
REASON = "Parked hardware setup. Applies after restart."
SETUP_LOCKED_REASON = ("Stays on while External Radar settings are saved; turning it off requires " +
                       "Remove Setup at the bottom of Vehicle > External Radar.")


def setup_saved(params) -> bool:
  """Any saved setup, even an unreadable one, keeps External Radar visible and its opt-in on."""
  document, readable = read_saved(params, DOCUMENT_KEY, MAX_BYTES)
  return not readable or document is not None


def _number_text(value) -> str:
  return str(value) if isinstance(value, int) else f"{value:g}"


class ExternalRadarFeature:
  def __init__(self, owner, *, galaxy: Callable[[], bool]):
    self.owner = owner
    self.galaxy = galaxy

  def capability(self):
    if not self.galaxy():
      return None
    cp = self.owner.vehicle_params()
    if cp is None or ineligible_reason(cp) is not None:
      return None
    if read_saved(self.owner.params, SETUP_KEY, 8) != (b"1", True) and not setup_saved(self.owner.params):
      return None
    return (str(cp.carFingerprint), bool(cp.openpilotLongitudinalControl), bool(cp.radarUnavailable))

  def entry(self):
    """The Vehicle settings row that opens the setup page."""
    if self.capability() is None:
      return ()
    saved = read_external_radar_settings(self.owner.params)
    mode = next((label for label, value in MODES.items() if value == saved.mode), "Off") if saved.valid else "Invalid"
    return (FeatureRow("", "External Radar", f"Experimental · {mode}", page=FeaturePage.EXTERNAL_RADAR, available=True),)

  def rows(self):
    capable = self.capability()
    if capable is None:
      return []
    owner = self.owner
    saved = read_external_radar_settings(owner.params)
    source = dict(saved.sources).get(DOCUMENT_KEY)
    allowed = owner.authority("parked_preferences")
    fingerprint = owner.vehicle_fingerprint()

    def row(key, label, value, *, reason=REASON, choices=(), step=0.0, minimum=0.0, maximum=0.0, unit="", repair_value="",
            default_value=None):
      return FeatureRow(key, label, value, source, choices, step, minimum, maximum, unit, available=allowed, reason=reason,
                        vehicle_fingerprint=fingerprint, capability=capable, dependencies=saved.sources,
                        repair_value=repair_value, default_value=default_value)

    remove = row(REMOVE, "Remove Setup", "", reason="")
    if not saved.valid:
      # Only the mode row can repair it, back to the defaults with the radar off.
      return [row(MODE, "Mode", "Invalid saved configuration", reason="Choose Off to reset the saved setup. " + REASON,
                  repair_value="Off"), remove]
    config = saved.config or MR76Config()
    mode_reason = REASON
    if read_saved(owner.params, VOACC_KEY, 8) == (b"1", True):
      mode_reason = "Active requires Use VOACC off in Developer settings; it ignores all radar points. " + REASON
    mode = next(label for label, value in MODES.items() if value == saved.mode)
    rows = [row(MODE, "Mode", mode, reason=mode_reason, choices=tuple(MODES), default_value="Off"),
            row(BUS, "CAN Bus", str(config.bus), choices=tuple(str(bus) for bus in BUSES), default_value=str(MR76Config.bus)),
            row(SENSOR_ID, "Sensor ID", str(config.sensor_id), choices=tuple(str(i) for i in range(8)),
                default_value=str(MR76Config.sensor_id))]
    for key, (field, label, step, unit) in NUMBER_ROWS.items():
      low, high = BOUNDS[field]
      rows.append(row(key, label, _number_text(getattr(config, field)), step=step, minimum=low, maximum=high, unit=unit,
                      default_value=_number_text(getattr(MR76Config, field))))
    for key, (field, label) in SWITCHES.items():
      rows.append(row(key, label, "On" if getattr(config, field) else "Off", choices=("Off", "On"),
                      default_value="On" if getattr(MR76Config, field) else "Off"))
    # Wiring, then mounting, then filters.
    order = (MODE, BUS, SENSOR_ID, PREFIX + "long_offset", PREFIX + "lat_offset", PREFIX + "yaw_deg", PREFIX + "upside_down",
             PREFIX + "max_lat", PREFIX + "rcs_min", PREFIX + "rcs_max", PREFIX + "drop_oncoming", PREFIX + "max_objects")
    rows = sorted(rows, key=lambda item: order.index(item.key))
    # Nothing saved yet means nothing to remove; the opt-in alone turns it off.
    return [*rows, remove] if source is not None else rows

  def _remove(self, request) -> bool:
    if request.value != "confirm":
      return False
    params = self.owner.params

    def authorized():
      fresh = next((row for row in self.rows() if row.key == REMOVE), None)
      return bool(fresh is not None and fresh.available and fresh.source == request.expected and
                  fresh.capability == request.capability and self.owner.vehicle_fingerprint() == request.vehicle_fingerprint)

    if not authorized():
      return False
    try:
      params.remove(DOCUMENT_KEY)
      params.put_bool(SETUP_KEY, False, block=True)
    except (OSError, RuntimeError, TypeError, ValueError):
      return False
    return read_saved(params, DOCUMENT_KEY, MAX_BYTES) == (None, True)

  def _requested(self, saved, key: str, value: str) -> bytes | None:
    if not saved.valid:
      return encode("off", MR76Config()) if key == MODE and value == "Off" else None
    mode, config = saved.mode, saved.config or MR76Config()
    if key == MODE:
      if value not in MODES:
        return None
      mode = MODES[value]
    elif key == BUS:
      if value not in tuple(str(bus) for bus in BUSES):
        return None
      config = replace(config, bus=int(value))
    elif key == SENSOR_ID:
      if value not in tuple(str(i) for i in range(8)):
        return None
      config = replace(config, sensor_id=int(value))
    elif key in SWITCHES:
      if value not in ("Off", "On"):
        return None
      config = replace(config, **{SWITCHES[key][0]: value == "On"})
    else:
      field = NUMBER_ROWS[key][0]
      number = float(value)
      if not math.isfinite(number):
        return None
      if isinstance(getattr(MR76Config, field), int):
        if not number.is_integer():
          return None
        number = int(number)
      config = replace(config, **{field: number})
    return encode(mode, config)

  def apply(self, request):
    if request.key not in KEYS or request.related_source is not None or request.direction or request.display_unit:
      return False
    row = next((row for row in self.rows() if row.key == request.key), None)
    if (row is None or not row.available or request.capability != row.capability or request.dependencies != row.dependencies or
        request.expected != row.source):
      return False
    if request.key == REMOVE:
      return self._remove(request)
    try:
      raw = self._requested(read_external_radar_settings(self.owner.params), request.key, request.value)
    except (ValueError, TypeError, OverflowError):
      return False
    if raw is None:
      return False

    def authorized():
      fresh = next((row for row in self.rows() if row.key == request.key), None)
      return bool(fresh is not None and fresh.available and fresh.capability == request.capability and
                  fresh.dependencies == request.dependencies and self.owner.vehicle_fingerprint() == request.vehicle_fingerprint)

    result = commit_exact(self.owner.params, key=DOCUMENT_KEY, max_bytes=MAX_BYTES, raw=raw, expected=request.expected,
                          authorized=authorized, temp_prefix=".external-radar-")
    return result.committed and result.verified
