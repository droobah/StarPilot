"""card's single hook for an add-on radar: choose the radar interface at startup, then report its status.

The add-on only replaces the radar interface of a car that has none in use. It never
changes CarParams and never sets radar errors, so losing it falls back to vision
instead of disengaging.
"""
from openpilot.common.swaglog import cloudlog
from opendbc.car.external_radar.mr76 import MR76RadarInterface
from opendbc.car.external_radar.startup import ineligible_reason, mr76_collisions, occupied_addresses
from openpilot.starpilot.car.external_radar.settings import read_external_radar_settings


class ExternalRadar:
  def __init__(self, ri: MR76RadarInterface | None, blocked_reason: str = ""):
    self.ri = ri
    self.blocked_reason = blocked_reason

  def write(self, status) -> None:
    """Fill a StarPilotCarState.ExternalRadar builder."""
    if self.ri is None:
      status.status = "blocked"
      status.blockedReason = self.blocked_reason
      return
    status.status = str(self.ri.status)
    status.faultReason = str(self.ri.fault)
    status.faultCount = min(self.ri.episodes, 255)
    status.passive = not self.ri.config.publish_points
    status.objectCount = min(len(self.ri.cycle), 255)


def _blocked(ri, reason: str):
  cloudlog.event("external_radar_blocked", reason=reason)
  return ri, ExternalRadar(None, reason)


def install(ri, cp, params, fingerprints, *, enabled: bool):
  """Returns (radar interface, ExternalRadar or None). None means no add-on radar is configured."""
  if not enabled:
    return ri, None
  try:
    settings = read_external_radar_settings(params)
    if not settings.valid:
      return _blocked(ri, "saved radar settings are invalid")
    config = settings.config
    if not settings.enabled or config is None:
      return ri, None
    reason = ineligible_reason(cp)
    if reason is not None:
      return _blocked(ri, reason)
    collisions = mr76_collisions(fingerprints or {}, config.bus, config.sensor_id)
    if collisions:
      used = ", ".join(f"0x{addr:X}" for addr, _ in collisions)
      return _blocked(ri, f"car already uses {used} on bus {config.bus}; choose another sensor ID or bus")
    external = MR76RadarInterface(cp, config, occupied_addresses(fingerprints or {}, config.bus))
  except Exception:
    # A broken add-on must never stop the car from starting.
    cloudlog.exception("external_radar_install_failed")
    return ri, ExternalRadar(None, "radar failed to start")
  cloudlog.event("external_radar_installed", mode=settings.mode, bus=config.bus, sensor_id=config.sensor_id)
  return external, ExternalRadar(external)
