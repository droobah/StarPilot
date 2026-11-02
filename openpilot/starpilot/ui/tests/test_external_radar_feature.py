from dataclasses import replace
import json

import pytest

from opendbc.car.car_helpers import interfaces
from opendbc.car.external_radar.mr76 import MR76Config
from opendbc.car.gm.values import CAR as GM
from openpilot.common.params import Params
from openpilot.starpilot.car.external_radar.settings import DOCUMENT_KEY, encode, read_external_radar_settings
from openpilot.starpilot.galaxy.developer_settings import DeveloperSettingsOwner
from openpilot.starpilot.ui.external_radar_feature import KEYS, MODE, PREFIX, REMOVE, SETUP_KEY
from openpilot.starpilot.ui.feature_settings_owner import FeatureSettingsOwner
from openpilot.starpilot.ui.feature_settings_state import FeaturePage, FeatureSettingsRequest

ORDER = ["mode", "bus", "sensor_id", "long_offset", "lat_offset", "yaw_deg", "upside_down",
         "max_lat", "rcs_min", "rcs_max", "drop_oncoming", "max_objects"]


def bolt(**changes):
  cp = interfaces[GM.CHEVROLET_BOLT_EUV].get_non_essential_params(GM.CHEVROLET_BOLT_EUV)
  cp.openpilotLongitudinalControl = True
  cp.radarUnavailable = True
  for key, value in changes.items():
    setattr(cp, key, value)
  return cp


@pytest.fixture
def params(tmp_path):
  return Params(str(tmp_path))


def owner(params, cp=None, *, galaxy=True, parked=True):
  cp = bolt() if cp is None else cp
  return FeatureSettingsOwner(params, lambda group: parked, vehicle_fingerprint=lambda: cp.carFingerprint,
                              vehicle_params=lambda: cp, galaxy=lambda: galaxy)


def page(settings_owner, name):
  return settings_owner.snapshot(name, parked=True, system_long=True, lateral_context=True, metric=False)


def radar_rows(settings_owner):
  return {row.key: row for row in page(settings_owner, FeaturePage.EXTERNAL_RADAR).rows}


def entry(settings_owner):
  return next((row for row in page(settings_owner, FeaturePage.VEHICLE).rows if row.page == FeaturePage.EXTERNAL_RADAR), None)


def request(row, value):
  return FeatureSettingsRequest(row.key, row.source, value, vehicle_fingerprint=row.vehicle_fingerprint,
                                capability=row.capability, dependencies=row.dependencies)


def opt_in(params, on=True):
  params.put_bool(SETUP_KEY, on, block=True)


def test_hidden_until_opted_in(params):
  settings = owner(params)
  assert entry(settings) is None and radar_rows(settings) == {}
  opt_in(params)
  row = entry(settings)
  assert row is not None and row.value == "Experimental · Off"
  assert list(radar_rows(settings)) == [PREFIX + name for name in ORDER]


@pytest.mark.parametrize("changes", ({"openpilotLongitudinalControl": False}, {"radarUnavailable": False}, {"dashcamOnly": True}))
def test_hidden_for_cars_that_cannot_use_it(params, changes):
  opt_in(params)
  settings = owner(params, bolt(**changes))
  assert entry(settings) is None and radar_rows(settings) == {}


def test_hidden_on_the_device(params):
  opt_in(params)
  assert entry(owner(params, galaxy=False)) is None


def test_saved_setup_stays_visible_after_opting_out(params):
  opt_in(params)
  settings = owner(params)
  assert settings.apply(request(radar_rows(settings)[MODE], "Passive"))
  opt_in(params, False)
  assert entry(settings).value == "Experimental · Passive"


def test_defaults_shown_without_a_saved_setup(params):
  opt_in(params)
  rows = radar_rows(owner(params))
  values = {key.removeprefix(PREFIX): row.value for key, row in rows.items()}
  assert values == {"mode": "Off", "bus": "1", "sensor_id": "2", "long_offset": "0", "lat_offset": "0", "yaw_deg": "0",
                    "upside_down": "Off", "max_lat": "6", "rcs_min": "0", "rcs_max": "30", "drop_oncoming": "On", "max_objects": "64"}
  assert all(row.available and row.default_value == row.value for row in rows.values())
  assert rows[PREFIX + "long_offset"].minimum == -3.0 and rows[PREFIX + "long_offset"].maximum == 1.0


@pytest.mark.parametrize("name, value, field, expected", (
  ("bus", "0", "bus", 0),
  ("sensor_id", "5", "sensor_id", 5),
  ("long_offset", "-1.35", "long_offset", -1.35),
  ("lat_offset", "0.12", "lat_offset", 0.12),
  ("yaw_deg", "-1.5", "yaw_deg", -1.5),
  ("upside_down", "On", "upside_down", True),
  ("max_lat", "4.5", "max_lat", 4.5),
  ("rcs_min", "-5", "rcs_min", -5.0),
  ("drop_oncoming", "Off", "drop_oncoming", False),
  ("max_objects", "32", "max_objects", 32),
))
def test_each_row_saves_one_field(params, name, value, field, expected):
  opt_in(params)
  settings = owner(params)
  assert settings.apply(request(radar_rows(settings)[PREFIX + name], value))
  saved = read_external_radar_settings(params)
  assert saved.valid and saved.mode == "off"
  assert saved.config == replace(MR76Config(publish_points=False), **{field: expected})
  assert radar_rows(settings)[PREFIX + name].value == value


@pytest.mark.parametrize("mode, stored", (("Passive", "passive"), ("Active", "active"), ("Off", "off")))
def test_mode(params, mode, stored):
  opt_in(params)
  settings = owner(params)
  # Galaxy marks every confirmed change.
  assert settings.apply(replace(request(radar_rows(settings)[MODE], mode), confirmation=True))
  assert read_external_radar_settings(params).mode == stored


@pytest.mark.parametrize("name, value", (
  ("mode", "On"), ("bus", "3"), ("sensor_id", "8"), ("long_offset", "1.5"), ("yaw_deg", "nan"), ("yaw_deg", "inf"),
  ("max_objects", "10.5"), ("max_objects", "0"), ("rcs_min", "40"), ("upside_down", "Yes"), ("lat_offset", "left"),
))
def test_invalid_values_are_rejected(params, name, value):
  opt_in(params)
  settings = owner(params)
  assert not settings.apply(request(radar_rows(settings)[PREFIX + name], value))
  assert params.get(DOCUMENT_KEY) is None


def test_stale_or_unparked_requests_are_rejected(params):
  opt_in(params)
  settings = owner(params)
  row = radar_rows(settings)[MODE]
  assert settings.apply(request(row, "Passive"))
  assert not settings.apply(request(row, "Active"))  # expected the document before the first save
  assert not settings.apply(replace(request(radar_rows(settings)[MODE], "Active"), capability=("other",)))
  assert not owner(params, parked=False).apply(request(radar_rows(settings)[MODE], "Active"))
  assert read_external_radar_settings(params).mode == "passive"


def test_rows_are_read_only_while_driving(params):
  opt_in(params)
  assert not any(row.available for row in radar_rows(owner(params, parked=False)).values())


def test_invalid_saved_setup_can_only_be_reset(params):
  opt_in(params)
  params.put(DOCUMENT_KEY, {"version": 1}, block=True)
  settings = owner(params)
  rows = radar_rows(settings)
  assert list(rows) == [MODE, REMOVE] and rows[MODE].value == "Invalid saved configuration"
  assert entry(settings).value == "Experimental · Invalid"
  assert not settings.apply(request(rows[MODE], "Active"))
  assert settings.apply(request(rows[MODE], "Off"))
  saved = read_external_radar_settings(params)
  assert saved.valid and saved.mode == "off" and saved.config == MR76Config(publish_points=False)


def test_vision_only_acc_warning(params):
  opt_in(params)
  params.put_bool("UseVOACC", True, block=True)
  assert "Use VOACC" in radar_rows(owner(params))[MODE].reason


def test_vehicle_settings_entry_on_the_hub(params):
  # A car with no other vehicle settings still gets the hub entry once opted in.
  opt_in(params)
  hub = page(owner(params), FeaturePage.HUB)
  assert any(row.page == FeaturePage.VEHICLE for row in hub.rows)


def test_all_keys_are_covered():
  assert KEYS == {PREFIX + name for name in ORDER} | {REMOVE}


def test_developer_opt_in(params):
  developer = DeveloperSettingsOwner(params, lambda: True)
  row = next(row for row in developer.snapshot().rows if row.key == SETUP_KEY)
  assert row.value == "Off" and row.available and row.label == "External Radar Setup"
  assert developer.apply(FeatureSettingsRequest(SETUP_KEY, row.source, "On"))
  assert params.get_bool(SETUP_KEY)
  row = next(row for row in developer.snapshot().rows if row.key == SETUP_KEY)
  assert row.value == "On"
  assert not DeveloperSettingsOwner(params, lambda: False).apply(FeatureSettingsRequest(SETUP_KEY, row.source, "Off"))


def test_remove_appears_only_once_something_is_saved(params):
  opt_in(params)
  settings = owner(params)
  assert REMOVE not in radar_rows(settings)
  assert settings.apply(request(radar_rows(settings)[MODE], "Passive"))
  assert list(radar_rows(settings))[-1] == REMOVE


def test_remove_deletes_the_setup_and_turns_the_opt_in_off(params):
  opt_in(params)
  settings = owner(params)
  assert settings.apply(request(radar_rows(settings)[PREFIX + "long_offset"], "-1.2"))
  row = radar_rows(settings)[REMOVE]
  assert not settings.apply(request(row, "yes"))  # only the confirmed action
  assert not owner(params, parked=False).apply(request(row, "confirm"))
  assert settings.apply(replace(request(row, "confirm"), confirmation=True))
  assert params.get(DOCUMENT_KEY) is None and not params.get_bool(SETUP_KEY)
  assert entry(settings) is None and radar_rows(settings) == {}
  assert developer_row(params).value == "Off" and developer_row(params).available


def test_remove_refuses_a_setup_that_changed_since_the_page_loaded(params):
  opt_in(params)
  settings = owner(params)
  assert settings.apply(request(radar_rows(settings)[MODE], "Passive"))
  stale = radar_rows(settings)[REMOVE]
  assert settings.apply(request(radar_rows(settings)[MODE], "Active"))
  assert not settings.apply(request(stale, "confirm"))
  assert read_external_radar_settings(params).mode == "active"


def test_remove_repairs_an_invalid_setup(params):
  opt_in(params)
  params.put(DOCUMENT_KEY, {"version": 1}, block=True)
  settings = owner(params)
  rows = radar_rows(settings)
  assert list(rows) == [MODE, REMOVE]
  assert settings.apply(request(rows[REMOVE], "confirm"))
  assert params.get(DOCUMENT_KEY) is None


def developer_row(params):
  return next(row for row in DeveloperSettingsOwner(params, lambda: True).snapshot().rows if row.key == SETUP_KEY)


def test_developer_opt_in_locks_on_once_a_setup_is_saved(params):
  opt_in(params)
  settings = owner(params)
  assert settings.apply(request(radar_rows(settings)[MODE], "Passive"))
  row = developer_row(params)
  assert row.value == "On" and not row.available and row.default_value is None and "requires" in row.reason
  assert not DeveloperSettingsOwner(params, lambda: True).apply(FeatureSettingsRequest(SETUP_KEY, row.source, "Off"))
  assert params.get_bool(SETUP_KEY)


@pytest.mark.parametrize("document", ({"version": 1}, None))
def test_any_saved_setup_locks_the_opt_in_even_if_it_was_never_turned_on(params, document):
  # An invalid setup counts too: it still needs the page to repair it.
  params.put(DOCUMENT_KEY, document or json.loads(encode("off", MR76Config())), block=True)
  row = developer_row(params)
  assert row.value == "On" and not row.available
  assert entry(owner(params)) is not None


def test_saved_document_round_trips_through_the_settings_reader(params):
  opt_in(params)
  params.put(DOCUMENT_KEY, json.loads(encode("active", MR76Config(bus=0, sensor_id=2))), block=True)
  rows = radar_rows(owner(params))
  assert rows[MODE].value == "Active" and rows[PREFIX + "bus"].value == "0"
