"""Nanoradar MR76 as an add-on, receive-only radar for any car without a factory radar.

The radar never reports radarErrors. On openpilot-longitudinal cars every error flag
soft-disables, so a lost add-on radar instead publishes no points and the car keeps its
own vision-only behavior. The state machine below decides when points are trusted.
"""
import math
from dataclasses import dataclass
from enum import StrEnum

from opendbc.can.dbc import DBC
from opendbc.car import structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.dbc.generator.nanoradar.nanoradar_mr76 import (OBJ_GENERAL, OBJ_LIST_STATUS, RADAR_STATE, SENSOR_IDS, SW_VERSION,
                                                           address, dbc_name)

CYCLE_TIMEOUT_NS = 300_000_000  # about 5 radar cycles of 60 ms
STATE_TIMEOUT_NS = 3_000_000_000  # RadarState is a 1 Hz heartbeat
RECOVERY_NS = 2_000_000_000
MAX_EPISODES = 3
FOREIGN_HEADERS_FOR_MISMATCH = 20  # about 1 s of another sensor ID's cycle headers
PUBLISH_EVERY = 5  # 20 Hz at card's 100 Hz, the RadarInterfaceBase cadence
TRACK_JUMP_M = 5.0  # an MR76 object ID reused for a different object

# The MR76 reports DistLat positive to the right. openpilot's yRel is positive to the left.
# Confirm on the bench before active use.
LATERAL_SIGN = -1.0
DYN_PROP_ONCOMING = 2


class RadarStatus(StrEnum):
  searching = "searching"
  active = "active"
  fallback = "fallback"
  disabled = "disabled"


class FaultReason(StrEnum):
  none = "none"
  cycleTimeout = "cycleTimeout"
  stateTimeout = "stateTimeout"
  sensorIdMismatch = "sensorIdMismatch"


@dataclass(frozen=True)
class MR76Config:
  bus: int = 1
  sensor_id: int = 2
  long_offset: float = 0.0  # m from the front bumper to the radar; negative means behind the bumper
  lat_offset: float = 0.0  # m, positive when the radar is left of the car's centerline
  yaw_deg: float = 0.0  # positive when the radar points left of straight ahead
  upside_down: bool = False
  max_lat: float = 6.0
  rcs_min: float = 0.0
  rcs_max: float = 30.0
  drop_oncoming: bool = True
  max_objects: int = 64
  publish_points: bool = True  # False keeps decoding and health for a passive drive without influencing radard

  def __post_init__(self):
    if self.sensor_id not in SENSOR_IDS or not 0 <= self.bus < 128 or not 1 <= self.max_objects <= 64:
      raise ValueError("Invalid MR76 configuration")
    values = (self.long_offset, self.lat_offset, self.yaw_deg, self.max_lat, self.rcs_min, self.rcs_max)
    if not all(math.isfinite(v) for v in values) or self.max_lat <= 0 or self.rcs_min > self.rcs_max:
      raise ValueError("Invalid MR76 configuration")


@dataclass
class MR76Object:
  object_id: int
  dist_long: float
  dist_lat: float
  vrel_long: float
  vrel_lat: float
  dyn_prop: int
  rcs: float


class _Decoder:
  def __init__(self, sensor_id: int):
    dbc = DBC(dbc_name(sensor_id))
    self.messages = {addr: list(dbc.addr_to_msg[addr].sigs.values())
                     for addr in (address(base, sensor_id) for base in (RADAR_STATE, OBJ_LIST_STATUS, OBJ_GENERAL, SW_VERSION))}

  def decode(self, addr: int, dat: bytes) -> dict[str, float]:
    values = {}
    for sig in self.messages[addr]:
      raw = sig.get_raw_value(dat)
      values[sig.name] = raw * sig.factor + sig.offset
    return values


class MR76RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, config: MR76Config, occupied: frozenset[int] = frozenset()):
    """occupied: addresses the car already sends on config.bus, never taken as another sensor ID's radar."""
    super().__init__(CP)
    self.config = config
    self.decoder = _Decoder(config.sensor_id)
    self.state_addr = address(RADAR_STATE, config.sensor_id)
    self.header_addr = address(OBJ_LIST_STATUS, config.sensor_id)
    self.object_addr = address(OBJ_GENERAL, config.sensor_id)
    self.version_addr = address(SW_VERSION, config.sensor_id)
    self.foreign_header_addrs = {address(OBJ_LIST_STATUS, i) for i in SENSOR_IDS if i != config.sensor_id} - occupied

    yaw = math.radians(config.yaw_deg)
    self.cos_yaw, self.sin_yaw = math.cos(yaw), math.sin(yaw)
    self.lat_sign = -LATERAL_SIGN if config.upside_down else LATERAL_SIGN

    self.now_ns = 0
    self.status = RadarStatus.searching
    self.fault = FaultReason.none
    self.episodes = 0
    self.healthy_since_ns: int | None = None
    self.last_cycle_ns = 0
    self.last_state_ns = 0
    self.foreign_headers = 0
    self.reported_sensor_id: int | None = None
    self.sw_version: tuple[int, int, int] | None = None
    self.incomplete_cycles = 0

    self._pending: list[MR76Object] | None = None
    self._expected = 0
    self.cycle: list[MR76Object] = []
    self._tracks: dict[int, tuple[int, float]] = {}  # MR76 object ID -> (trackId, last dRel)
    self._next_track_id = 0

  def update(self, can_packets):
    self.frame += 1
    for nanos, frames in can_packets or ():
      self.now_ns = max(self.now_ns, nanos)
      for addr, dat, src in frames:
        if src == self.config.bus:
          self._receive(nanos, addr, dat)
    self._update_status()

    if self.frame % PUBLISH_EVERY:
      return None
    ret = structs.RadarData()
    if self.status == RadarStatus.active and self.config.publish_points:
      ret.points = self._points()
    return ret

  def _receive(self, nanos: int, addr: int, dat: bytes) -> None:
    if addr == self.object_addr:
      if self._pending is not None:
        v = self.decoder.decode(addr, dat)
        self._pending.append(MR76Object(int(v["ID"]), v["DistLong"], v["DistLat"], v["VrelLong"], v["VrelLat"],
                                        int(v["DynProp"]), v["RCS"]))
        self._finish_cycle(nanos)
    elif addr == self.header_addr:
      if self._pending is not None:
        self.incomplete_cycles += 1
      self._pending = []
      self._expected = int(self.decoder.decode(addr, dat)["NofObjects"])
      self._finish_cycle(nanos)
    elif addr == self.state_addr:
      self.reported_sensor_id = int(self.decoder.decode(addr, dat)["SensorID"])
      self.last_state_ns = nanos
    elif addr == self.version_addr:
      v = self.decoder.decode(addr, dat)
      self.sw_version = (int(v["Major"]), int(v["Minor"]), int(v["Patch"]))
    elif addr in self.foreign_header_addrs:
      self.foreign_headers += 1

  def _finish_cycle(self, nanos: int) -> None:
    if self._pending is not None and len(self._pending) >= self._expected:
      self.cycle = self._pending
      self._pending = None
      self.last_cycle_ns = nanos

  def _healthy(self) -> FaultReason:
    if self.last_cycle_ns == 0 or self.now_ns - self.last_cycle_ns > CYCLE_TIMEOUT_NS:
      return FaultReason.cycleTimeout
    if self.last_state_ns == 0 or self.now_ns - self.last_state_ns > STATE_TIMEOUT_NS:
      return FaultReason.stateTimeout
    return FaultReason.none

  def _update_status(self) -> None:
    if self.status == RadarStatus.disabled:
      return
    if (self.reported_sensor_id not in (None, self.config.sensor_id) or
        (self.foreign_headers >= FOREIGN_HEADERS_FOR_MISMATCH and self.last_cycle_ns == 0)):
      self._disable(FaultReason.sensorIdMismatch)
      return

    fault = self._healthy()
    if self.status == RadarStatus.active:
      if fault != FaultReason.none:
        self.episodes += 1
        self.fault = fault
        self.healthy_since_ns = None
        self._tracks.clear()
        if self.episodes >= MAX_EPISODES:
          self._disable(fault)
        else:
          self.status = RadarStatus.fallback
      return

    if fault != FaultReason.none:
      self.healthy_since_ns = None
      return
    if self.status == RadarStatus.searching:
      self.status = RadarStatus.active
      return
    if self.healthy_since_ns is None:
      self.healthy_since_ns = self.now_ns
    elif self.now_ns - self.healthy_since_ns >= RECOVERY_NS:
      self.status = RadarStatus.active
      self.healthy_since_ns = None

  def _disable(self, reason: FaultReason) -> None:
    self.status = RadarStatus.disabled
    self.fault = reason
    self.cycle = []
    self._tracks.clear()

  def _points(self) -> list:
    c = self.config
    points = []
    seen: dict[int, tuple[int, float]] = {}
    for obj in self.cycle[:c.max_objects]:
      if c.drop_oncoming and obj.dyn_prop == DYN_PROP_ONCOMING:
        continue
      if not c.rcs_min <= obj.rcs <= c.rcs_max:
        continue
      x, y = obj.dist_long, self.lat_sign * obj.dist_lat
      vx, vy = obj.vrel_long, self.lat_sign * obj.vrel_lat
      d_rel = x * self.cos_yaw - y * self.sin_yaw + c.long_offset
      y_rel = x * self.sin_yaw + y * self.cos_yaw + c.lat_offset
      v_rel = vx * self.cos_yaw - vy * self.sin_yaw
      if d_rel < 0.0 or abs(y_rel) > c.max_lat:
        continue

      previous = self._tracks.get(obj.object_id)
      if previous is None or abs(previous[1] - d_rel) > TRACK_JUMP_M:
        track_id = self._next_track_id
        self._next_track_id += 1
      else:
        track_id = previous[0]
      seen[obj.object_id] = (track_id, d_rel)

      pt = structs.RadarData.RadarPoint()
      pt.trackId = track_id
      pt.dRel = d_rel
      pt.yRel = y_rel
      pt.vRel = v_rel
      points.append(pt)
    self._tracks = seen
    return points
