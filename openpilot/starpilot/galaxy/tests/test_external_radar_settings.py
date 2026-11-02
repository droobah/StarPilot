from dataclasses import replace
import unittest

from opendbc.car.car_helpers import interfaces
from opendbc.car.gm.values import CAR as GM
from openpilot.starpilot.car.external_radar.settings import DOCUMENT_KEY, read_external_radar_settings
from openpilot.starpilot.galaxy.tests import test_settings as fixtures
from openpilot.starpilot.ui.external_radar_feature import SETUP_KEY


class TestExternalRadarGateway(unittest.TestCase):
  setUp = fixtures.SettingsGatewayTest.setUp
  page = fixtures.SettingsGatewayTest.page

  def bolt(self):
    cp = interfaces[GM.CHEVROLET_BOLT_EUV].get_non_essential_params(GM.CHEVROLET_BOLT_EUV)
    cp.openpilotLongitudinalControl = True
    cp.radarUnavailable = True
    self.context.value = replace(self.context.value, cp=cp)

  def save(self, label, value):
    page = self.page("vehicle/external_radar")
    index = next(i for i, row in enumerate(page["rows"]) if row["label"] == label)
    intent = self.gateway.preview(page["view"], index, 0, self.session, self.generation, value=value)
    return self.gateway.confirm(intent["intent"], self.session, self.generation)

  def test_entry_needs_the_developer_opt_in(self):
    self.bolt()
    self.assertNotIn("External Radar", [row["label"] for row in self.page("vehicle")["rows"]])
    self.params.put_bool(SETUP_KEY, True, block=True)
    entry = next(row for row in self.page("vehicle")["rows"] if row["label"] == "External Radar")
    self.assertEqual((entry["page"], entry["value"]), ("vehicle/external_radar", "Experimental · Off"))

  def test_choice_and_number_save_through_preview_and_confirm(self):
    self.bolt()
    self.params.put_bool(SETUP_KEY, True, block=True)
    self.assertTrue(self.save("Mode", "Passive"))
    self.assertTrue(self.save("Forward Offset", -1.35))
    self.assertTrue(self.save("Sensor ID", "3"))
    saved = read_external_radar_settings(self.params)
    self.assertEqual((saved.mode, saved.config.long_offset, saved.config.sensor_id), ("passive", -1.35, 3))

  def test_remove_asks_first_then_clears_everything(self):
    self.bolt()
    self.params.put_bool(SETUP_KEY, True, block=True)
    self.assertTrue(self.save("Mode", "Active"))
    page = self.page("vehicle/external_radar")
    index = next(i for i, row in enumerate(page["rows"]) if row["label"] == "Remove Setup")
    self.assertTrue(page["rows"][index]["confirm"])
    intent = self.gateway.preview(page["view"], index, 0, self.session, self.generation)
    self.assertIn("Delete the saved External Radar setup", intent["question"])
    self.assertIsNotNone(self.params.get(DOCUMENT_KEY))  # nothing is deleted before confirming
    self.assertTrue(self.gateway.confirm(intent["intent"], self.session, self.generation))
    self.assertIsNone(self.params.get(DOCUMENT_KEY))
    self.assertFalse(self.params.get_bool(SETUP_KEY))
    self.assertNotIn("External Radar", [row["label"] for row in self.page("vehicle")["rows"]])
