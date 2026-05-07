import unittest

import gymnasium as gym

from gymkhana import drift_config  # importing gymkhana also registers the gym id


class TestDriftConfig(unittest.TestCase):
    def test_defaults(self):
        cfg = drift_config()
        self.assertEqual(cfg["model"], "std")
        self.assertEqual(cfg["control_input"], ["accl", "steering_angle"])
        self.assertEqual(cfg["observation_config"], {"type": "drift"})
        self.assertTrue(cfg["normalize_obs"])
        self.assertTrue(cfg["normalize_act"])
        self.assertIn("params", cfg)

    def test_overrides_take_precedence(self):
        cfg = drift_config(map="Drift", num_agents=2, normalize_obs=False)
        self.assertEqual(cfg["map"], "Drift")
        self.assertEqual(cfg["num_agents"], 2)
        self.assertFalse(cfg["normalize_obs"])
        self.assertEqual(cfg["model"], "std")  # untouched default

    def test_constructs_env(self):
        env = gym.make("gymkhana:gymkhana-v0", config=drift_config(map="Drift"))
        env.reset()
        env.close()


if __name__ == "__main__":
    unittest.main()
