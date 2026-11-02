from types import SimpleNamespace

from opendbc.car import structs
from openpilot.cereal import messaging
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.events import Alert, AlertSize, AlertStatus, AudibleAlert, Priority, VisualAlert
from openpilot.starpilot.car.external_radar.notifications import (
  BLOCKED, DISABLED, LOST, MAX_TEXT_2, NOT_DETECTED, PASSIVE_TEXT, RECOVERED, ExternalRadarMonitor, ExternalRadarNotification,
)

STEP_NS = 100_000_000


class Feed:
  def __init__(self):
    self.monitor = ExternalRadarMonitor()
    self.now = 10_000_000_000
    self.drive = 1

  def sample(self, status, **overrides):
    self.now += STEP_NS
    args = {"drive": self.drive, "now_ns": self.now, "stamp_ns": self.now, "status": status}
    args.update(overrides)
    return self.monitor.update(**args)

  def run(self, status, count):
    return [notice for notice in (self.sample(status) for _ in range(count)) if notice is not None]


def test_second_lines_fit_the_comma_four():
  assert all(len(text) <= MAX_TEXT_2 for text in (*(n.text_2 for n in (LOST, DISABLED, RECOVERED, NOT_DETECTED, BLOCKED)),
                                                 PASSIVE_TEXT))


def test_healthy_radar_is_silent():
  feed = Feed()
  assert feed.run("searching", 5) + feed.run("active", 200) == []


def test_loss_and_recovery_each_notify_once_per_episode():
  feed = Feed()
  feed.run("active", 3)
  assert feed.run("fallback", 10) == [LOST]
  assert feed.run("active", 10) == [RECOVERED]
  assert feed.run("fallback", 10) == [LOST]


def test_latching_off_notifies_once():
  feed = Feed()
  feed.run("active", 3)
  assert feed.run("disabled", 50) == [DISABLED]
  feed = Feed()
  feed.run("active", 3)
  feed.run("fallback", 3)
  assert feed.run("disabled", 50) == [DISABLED]


def test_not_detected_after_ten_seconds_of_searching():
  feed = Feed()
  assert feed.run("searching", 99) == []
  assert feed.run("searching", 300) == [NOT_DETECTED]


def test_radar_found_before_ten_seconds_is_silent():
  feed = Feed()
  feed.run("searching", 90)
  assert feed.run("active", 5) == []
  assert feed.run("disabled", 5) == [DISABLED]  # a sensor ID mismatch disables straight from searching


def test_blocked_notifies_once_per_drive():
  feed = Feed()
  assert feed.run("blocked", 50) == [BLOCKED]
  feed.drive = 2
  assert feed.run("blocked", 50) == [BLOCKED]


def test_no_radar_configured_is_silent():
  assert Feed().run("off", 200) == []


def test_stale_or_offroad_status_is_ignored():
  feed = Feed()
  feed.run("active", 3)
  assert feed.sample("fallback", stamp_ns=feed.now - 600_000_000) is None
  assert feed.sample("fallback", stamp_ns=feed.now + 2 * STEP_NS) is None  # from the future
  assert feed.sample("fallback", drive=0) is None
  # The offroad reset drops the history: fallback with no earlier active is not a loss.
  assert feed.run("fallback", 5) == []


def test_new_drive_resets_history():
  feed = Feed()
  feed.run("active", 3)
  feed.drive = 2
  assert feed.run("fallback", 5) == []


class Messages:
  def __init__(self):
    self.now = 10_000_000_000
    self.status, self.passive, self.fresh = "active", False, True
    self.updated = {"starpilotCarState": True}
    self.logMonoTime = {"starpilotCarState": self.now}

  def __getitem__(self, name):
    if name == "deviceState":
      return SimpleNamespace(started=True, startedMonoTime=1)
    return SimpleNamespace(externalRadar=SimpleNamespace(status=self.status, passive=self.passive))

  def all_checks(self, names):
    return True

  def step(self):
    self.now += STEP_NS
    self.logMonoTime["starpilotCarState"] = self.now


def test_alert_presentation_and_prompt_plays_once():
  sm, notification, manager = Messages(), ExternalRadarNotification(), AlertManager()
  for _ in range(3):
    sm.step()
    assert notification.update(sm, now_ns=sm.now) is None
  sm.step()
  sm.status = "fallback"
  alert = notification.update(sm, now_ns=sm.now)
  assert alert is not None and alert.alert_type == "externalRadar/lost"
  assert (alert.alert_text_1, alert.alert_text_2) == ("External radar lost", "Using vision only")
  assert alert.alert_size == AlertSize.mid and alert.priority == Priority.LOW
  assert alert.audible_alert == AudibleAlert.prompt and alert.duration == 400

  manager.add_many(100, [alert])
  urgent = Alert("Take control", "", AlertStatus.critical, AlertSize.full, Priority.HIGHEST,
                 VisualAlert.steerRequired, AudibleAlert.warningImmediate, 0.)
  urgent.alert_type, urgent.event_type = "urgent", "urgent"
  manager.process_alerts(100, set())
  notification.selected(manager.current_alert)
  manager.add_many(101, [urgent])
  manager.process_alerts(101, set())
  notification.selected(manager.current_alert)
  manager.process_alerts(103, set())
  notification.selected(manager.current_alert)
  assert manager.current_alert == alert and alert.audible_alert == AudibleAlert.none


def test_passive_mode_says_so():
  sm, notification = Messages(), ExternalRadarNotification()
  sm.passive = True
  sm.step()
  notification.update(sm, now_ns=sm.now)
  sm.step()
  sm.status = "fallback"
  alert = notification.update(sm, now_ns=sm.now)
  assert alert is not None and alert.alert_text_2 == PASSIVE_TEXT


def test_routine_notices_are_quiet_and_low_priority():
  sm, notification = Messages(), ExternalRadarNotification()
  sm.status = "blocked"
  sm.step()
  alert = notification.update(sm, now_ns=sm.now)
  assert alert is not None and alert.alert_type == "externalRadar/blocked"
  assert alert.priority == Priority.LOWEST and alert.audible_alert == AudibleAlert.none


def test_no_new_status_message_does_not_advance():
  sm, notification = Messages(), ExternalRadarNotification()
  sm.step()
  notification.update(sm, now_ns=sm.now)
  sm.status, sm.updated["starpilotCarState"] = "fallback", False
  sm.step()
  assert notification.update(sm, now_ns=sm.now) is None


def test_actual_selfdrived_alert_path(monkeypatch):
  from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
  from openpilot.selfdrive.selfdrived.events import Events
  from openpilot.selfdrive.selfdrived.state import StateMachine
  from openpilot.starpilot.controllers.mode_actions import SwitchbackCooldown
  from openpilot.starpilot.longitudinal.force_stop_alert import HoldAlertState
  daemon = SelfdriveD.__new__(SelfdriveD)
  daemon.CP, daemon.AM = structs.CarParams(), AlertManager()
  daemon.sm = messaging.SubMaster(["deviceState", "starpilotCarState"])
  daemon.events, daemon.enabled, daemon.personality, daemon.is_metric = Events(), False, 1, True
  daemon.state_machine = StateMachine()
  daemon.switchback_capable = False
  daemon.switchback_cooldown = SwitchbackCooldown()
  daemon.force_stop_hold_alert = HoldAlertState()
  daemon.radar_recovery_enabled = False
  now = [10_000_000_000]
  daemon.switchback_setting_ns = now[0] + 100_000_000_000
  daemon.switchback_cooldown_ns = 0
  monkeypatch.setattr("openpilot.selfdrive.selfdrived.selfdrived.time.monotonic_ns", lambda: now[0])

  def publish(status, *, started=True, update_device=False):
    companion = messaging.new_message("starpilotCarState", valid=True, logMonoTime=now[0])
    companion.starpilotCarState.externalRadar.status = status
    messages = [companion.as_reader()]
    if update_device:
      device = messaging.new_message("deviceState", valid=True, logMonoTime=now[0])
      device.deviceState.started, device.deviceState.startedMonoTime = started, 1
      messages.append(device.as_reader())
    daemon.sm.update_msgs(now[0] / 1e9, messages)

  for tick in range(40):
    now[0] += 50_000_000
    publish("active" if tick < 20 else "fallback", update_device=tick % 10 == 0)
    daemon.sm.frame = tick * 5
    daemon.update_alerts(SimpleNamespace())
  assert not daemon.enabled and daemon.AM.current_alert.alert_type == "externalRadar/lost"
