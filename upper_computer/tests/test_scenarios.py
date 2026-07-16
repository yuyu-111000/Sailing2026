from __future__ import annotations

import unittest

from sailing.scenarios import run_c1_scenario, run_c2_scenario


class ScenarioTests(unittest.TestCase):
    def test_c1_host_scenario(self) -> None:
        result = run_c1_scenario()
        self.assertEqual(result["global_state"], "COMPLETE")
        self.assertEqual(result["gates_passed"], 10)
        self.assertFalse(result["hardware_used"])

    def test_c2_host_scenario(self) -> None:
        result = run_c2_scenario()
        self.assertEqual(result["global_state"], "COMPLETE")
        self.assertEqual(result["shot_id"], 1)
        self.assertEqual(result["fire_source"], "AUTO")
        self.assertFalse(result["hardware_used"])


if __name__ == "__main__":
    unittest.main()
