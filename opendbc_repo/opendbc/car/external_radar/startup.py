"""Startup checks for an add-on radar: which cars qualify, and whether its addresses are free.

Both checks are brand-agnostic. The address check uses the startup CAN fingerprint
({bus: {address: length}}), so it only sees what the car sent while fingerprinting;
some modules only transmit while driving, which a full-drive capture should confirm.
"""
from opendbc.dbc.generator.nanoradar.nanoradar_mr76 import address

# Every MR76 base address (MR76_Radar driver message list, plus the object quality and
# extended IDs that the config frame can enable). Inputs would reconfigure the radar if a
# car module sent them; outputs could confuse a car module listening on that address.
MR76_ADDRESSES = {
  0x200: "RadarCfg (radar input)",
  0x201: "RadarState",
  0x400: "CollDetCfg (radar input)",
  0x401: "CollDetRegionCfg (radar input)",
  0x402: "CollDetRegionState",
  0x408: "CollDetState",
  0x60A: "ObjListStatus",
  0x60B: "ObjGeneral",
  0x60C: "ObjQuality",
  0x60D: "ObjExtended",
  0x60E: "ObjCollWarning",
  0x700: "SwVersion",
}


def ineligible_reason(cp) -> str | None:
  """None when the car can use an add-on radar, otherwise why not."""
  if cp.notCar or cp.dashcamOnly or cp.passive:
    return "openpilot is not controlling this car"
  if not cp.openpilotLongitudinalControl:
    return "requires openpilot longitudinal control"
  if not cp.radarUnavailable:
    return "car already uses a factory radar"
  return None


def occupied_addresses(fingerprints: dict[int, dict[int, int]], bus: int) -> frozenset[int]:
  return frozenset(fingerprints.get(bus, {}))


def mr76_collisions(fingerprints: dict[int, dict[int, int]], bus: int, sensor_id: int) -> tuple[tuple[int, str], ...]:
  """MR76 addresses for this sensor ID that the car already uses on this bus."""
  occupied = occupied_addresses(fingerprints, bus)
  return tuple((address(base, sensor_id), name) for base, name in MR76_ADDRESSES.items() if address(base, sensor_id) in occupied)
