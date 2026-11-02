"""Drive-local notifications from card's StarPilotCarState.externalRadar status.

The add-on radar never disengages anything: radard just stops getting its points.
These alerts only tell the driver that following now relies on vision.
"""
from dataclasses import dataclass

EVENT_TYPE = "externalRadarNotification"
MAX_STATUS_AGE_NS = 500_000_000
NOT_DETECTED_NS = 10_000_000_000
PASSIVE_TEXT = "Passive mode"
MAX_TEXT_2 = 20  # wider second lines are cut off on the comma four


@dataclass(frozen=True)
class Notice:
  key: str
  text_1: str
  text_2: str
  urgent: bool  # outranks routine notices and plays a prompt
  duration_s: float


LOST = Notice("lost", "External radar lost", "Using vision only", True, 4.)
DISABLED = Notice("disabled", "External radar off until restart", "Using vision only", True, 6.)
RECOVERED = Notice("recovered", "External radar available again", "", False, 3.)
NOT_DETECTED = Notice("notDetected", "External radar not detected", "Check wiring and ID", False, 6.)
BLOCKED = Notice("blocked", "External radar not started", "Check radar settings", False, 6.)


class ExternalRadarMonitor:
  def __init__(self):
    self.reset()

  def reset(self):
    self.drive = 0
    self.previous = None
    self.searching_since_ns = None
    self.shown: set[str] = set()

  def update(self, *, drive: int, now_ns: int, stamp_ns: int, status: str) -> Notice | None:
    if drive <= 0:
      self.reset()
      return None
    if drive != self.drive:
      self.reset()
      self.drive = drive
    if not stamp_ns <= now_ns <= stamp_ns + MAX_STATUS_AGE_NS:
      # A stale status says nothing about the radar; card's own health alerts cover a stalled card.
      return None
    previous, self.previous = self.previous, status

    if status == "searching":
      if self.searching_since_ns is None:
        self.searching_since_ns = now_ns
      if now_ns - self.searching_since_ns >= NOT_DETECTED_NS:
        return self._once(NOT_DETECTED)
      return None
    self.searching_since_ns = None

    if status == "blocked":
      return self._once(BLOCKED)
    if status == "disabled" and previous not in (None, "disabled"):
      return self._once(DISABLED)
    if status == "fallback" and previous == "active":
      return LOST
    if status == "active" and previous == "fallback":
      return RECOVERED
    return None

  def _once(self, notice: Notice) -> Notice | None:
    if notice.key in self.shown:
      return None
    self.shown.add(notice.key)
    return notice


class ExternalRadarNotification:
  def __init__(self):
    self.monitor = ExternalRadarMonitor()
    self.alert = None
    self.sound_presented = False

  def update(self, sm, *, now_ns: int):
    notice, passive = None, False
    try:
      drive = int(sm["deviceState"].startedMonoTime) if sm["deviceState"].started and sm.all_checks(["deviceState"]) else 0
      if not sm.updated["starpilotCarState"]:
        if drive <= 0:
          self.monitor.reset()
        return None
      radar = sm["starpilotCarState"].externalRadar
      passive = bool(radar.passive)
      notice = self.monitor.update(drive=drive, now_ns=now_ns, stamp_ns=int(sm.logMonoTime["starpilotCarState"]),
                                   status=str(radar.status))
    except (AttributeError, KeyError, TypeError, ValueError):
      self.monitor.reset()
    if notice is None:
      return None
    from openpilot.selfdrive.selfdrived.events import Alert, AlertSize, AlertStatus, AudibleAlert, Priority, VisualAlert
    text_2 = PASSIVE_TEXT if passive and notice.text_2 == LOST.text_2 else notice.text_2
    self.alert = Alert(notice.text_1, text_2, AlertStatus.normal, AlertSize.mid if text_2 else AlertSize.small,
                       Priority.LOW if notice.urgent else Priority.LOWEST, VisualAlert.none,
                       AudibleAlert.prompt if notice.urgent else AudibleAlert.none, notice.duration_s)
    self.alert.alert_type, self.alert.event_type = f"externalRadar/{notice.key}", EVENT_TYPE
    self.sound_presented = False
    return self.alert

  def selected(self, current):
    """Play the prompt once, even if a higher-priority alert interrupts this one and it comes back."""
    if self.alert is None:
      return
    if current.alert_type == self.alert.alert_type:
      self.sound_presented = True
    elif self.sound_presented:
      from openpilot.selfdrive.selfdrived.events import AudibleAlert
      self.alert.audible_alert = AudibleAlert.none
