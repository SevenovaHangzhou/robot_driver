#!/usr/bin/env python3

import unittest

from tools import diff_legacy


class DiffLegacyApprovedOverlayTest(unittest.TestCase):
    def legacy_profile(self):
        return diff_legacy.compose_yaml(
            """auto_state_transitions: true
sdo:
  - {index: 0x6060, sub_index: 0, type: int8, value: 8}
rpdo:
  - index: 0x1600
    channels:
      - {index: 0x6040, sub_index: 0, type: uint16}
tpdo:
  - index: 0x1a00
    channels:
      - {index: 0x6041, sub_index: 0, type: uint16}
""",
            "unit-test legacy profile",
        )

    def test_zeroerr_overlay_prepends_uint16_sync_tolerance(self):
        expected = diff_legacy.apply_frozen_overlay(self.legacy_profile(), ti5=False)
        self.assertEqual(expected["sdo[0].index"].value, 0x10F1)
        self.assertEqual(expected["sdo[0].sub_index"].value, 2)
        self.assertEqual(expected["sdo[0].type"].value, "uint16")
        self.assertEqual(expected["sdo[0].value"].value, 100)
        self.assertEqual(expected["sdo[1].index"].value, 0x6060)

    def test_ti5_overlay_prepends_uint32_sync_tolerance(self):
        expected = diff_legacy.apply_frozen_overlay(self.legacy_profile(), ti5=True)
        self.assertEqual(expected["sdo[0].index"].value, 0x10F1)
        self.assertEqual(expected["sdo[0].type"].value, "uint32")
        self.assertEqual(expected["sdo[0].value"].value, 100)
        self.assertEqual(expected["sdo[1].index"].value, 0x6060)
        self.assertEqual(expected["sdo[2].index"].value, 0x60C2)

    def test_unapproved_sync_tolerance_change_still_fails(self):
        expected = diff_legacy.apply_frozen_overlay(self.legacy_profile(), ti5=False)
        actual = dict(expected)
        actual["sdo[0].value"] = diff_legacy.Scalar("int", 101, "101")
        differences = diff_legacy.compare_maps("joint", expected, actual)
        self.assertEqual(len(differences), 1)
        self.assertEqual(differences[0].key, "sdo[0].value")

    def test_xmc_sw511_overlay_rejects_stale_xml_tail_entries(self):
        expected = diff_legacy.compose_yaml(
            diff_legacy.XMC_UPDOWN_SW511_PROFILE, "unit-test XMC profile"
        )
        actual = dict(expected)
        actual["rpdo[0].channels[6].index"] = diff_legacy.Scalar(
            "int", 0x607F, "0x607f"
        )
        differences = diff_legacy.compare_maps("updown", expected, actual)
        self.assertEqual(len(differences), 1)
        self.assertEqual(differences[0].key, "rpdo[0].channels[6].index")


if __name__ == "__main__":
    unittest.main()
