# MIT License

# Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Main Gymnasium environment for Gym-Khana autonomous racing simulation.

Authors: Hongrui Zheng, Teodor Ilie
"""

# gym imports
import warnings

import gymnasium as gym

# others
import numpy as np

from .action import CarAction, from_single_to_multi_action_space

# base classes
from .base_classes import DynamicModel, Simulator
from .integrator import IntegratorType
from .observation import observation_factory
from .params import load_params
from .rendering import make_renderer
from .reset import make_reset_fn
from .track import Track
from .utils import deep_update


def print_obs_min_max_stats(tracker, step_count, features, bounds, normalize_obs):
    """Print a table of recorded observation min/max values.

    Pure function — reused by ``GKEnv._print_obs_min_max_stats`` (standalone)
    and by ``train.train_utils.aggregate_and_print_obs_min_max`` (parallelized).
    Theoretical-bound columns render ``N/A`` for features missing from ``bounds``.
    """
    TABLE_WIDTH = 120
    FEATURE_WIDTH = 25
    OBS_WIDTH = 12
    NA = "N/A"

    # Print header
    print("\n" + "=" * TABLE_WIDTH)
    print("Observation Min/Max Statistics")
    print(f"Tracked over {step_count:,} timesteps")
    if normalize_obs:
        print("Normalization: ENABLED   (theoretical bounds are the active scaling targets)")
    else:
        print("Normalization: DISABLED  (theoretical bounds not computed)")
    print("=" * TABLE_WIDTH)
    print(
        f"{'Feature':<{FEATURE_WIDTH}} | {'Rec. Min':>{OBS_WIDTH}} | {'Rec. Max':>{OBS_WIDTH}} | {'Theor. Min':>{OBS_WIDTH}} | {'Theor. Max':>{OBS_WIDTH}} | {'Coverage':>{OBS_WIDTH}}"
    )
    print("-" * TABLE_WIDTH)

    bounds_violations = []

    for feature_name in features:
        # Get recorded min/max values
        rec_min = tracker[feature_name]["min"]
        rec_max = tracker[feature_name]["max"]

        # Handle case where no valid values were recorded
        if rec_min == float("inf") or rec_max == float("-inf"):
            print(
                f"{feature_name:<{FEATURE_WIDTH}} | {NA:>{OBS_WIDTH}} | {NA:>{OBS_WIDTH}} | {NA:>{OBS_WIDTH}} | {NA:>{OBS_WIDTH}} | {NA:>{OBS_WIDTH}}"
            )
            continue

        theor_bounds = bounds.get(feature_name)
        if theor_bounds is not None:
            theor_min, theor_max = theor_bounds

            # Calculate coverage
            theor_range = theor_max - theor_min
            rec_range = rec_max - rec_min
            coverage_str = f"{(rec_range / theor_range) * 100.0:6.1f}%" if theor_range > 0 else NA

            # Check for violations
            exceeds = ""
            eps = 1e-4
            if rec_min < theor_min - eps or rec_max > theor_max + eps:
                bounds_violations.append(feature_name)
                exceeds = " ⚠️"

            print(
                f"{feature_name:<{FEATURE_WIDTH}} | {rec_min:12.4f} | {rec_max:12.4f} | {theor_min:12.4f} | {theor_max:12.4f} | {coverage_str:>{OBS_WIDTH}}{exceeds}"
            )
        else:
            # Theoretical bounds not available for this feature
            print(
                f"{feature_name:<{FEATURE_WIDTH}} | {rec_min:12.4f} | {rec_max:12.4f} | {NA:>{OBS_WIDTH}} | {NA:>{OBS_WIDTH}} | {NA:>{OBS_WIDTH}}"
            )

    print("=" * TABLE_WIDTH)

    if bounds_violations:
        print("⚠️  WARNING: The following features exceeded theoretical bounds:")
        for feature_name in bounds_violations:
            print(f"    - {feature_name}")
        print("   Consider updating normalization bounds in calculate_norm_bounds()")
        print("=" * TABLE_WIDTH)

    print()


class GKEnv(gym.Env):
    """Gymnasium environment for Gym-Khana autonomous racing.

    Implements the standard Gymnasium ``Env`` interface. Create via::

        env = gym.make('gymkhana:gymkhana-v0', config={...})

    Configuration is passed as a dict to ``__init__`` and merged with defaults
    from :meth:`default_config`. See that method for all available keys and
    their defaults. Vehicle physics parameters are documented in
    :mod:`gymkhana.envs.dynamic_models`; preset parameter dicts are available
    via :meth:`f1tenth_vehicle_params`, :meth:`f1tenth_std_vehicle_params`, etc.

    Args:
        config: Configuration dict merged with :meth:`default_config`.
            Key configuration options:

            - ``map`` (str): Track name (default ``"Spielberg"``).
            - ``params`` (dict): Vehicle parameters (default F1TENTH ST params).
            - ``model`` (str): Dynamics model — ``"ks"``, ``"st"``, ``"mb"``, ``"std"`` (default ``"st"``).
            - ``num_agents`` (int): Number of agents (default ``2``).
            - ``timestep`` (float): Physics timestep in seconds (default ``0.01``).
            - ``ego_idx`` (int): Index of the ego agent (default ``0``).
            - ``control_input`` (list): Action types, e.g. ``["speed", "steering_angle"]``.
            - ``observation_config`` (dict): Observation type config, e.g. ``{"type": "drift"}``.
            - ``normalize_obs`` (bool | None): Observation normalisation; ``None`` = auto.
            - ``normalize_act`` (bool): Action normalisation (default ``True``).
            - ``training_mode`` (str): ``"race"`` or ``"recover"`` (default ``"race"``).
            - ``predictive_collision`` (bool): TTC-based (True) or Frenet-based (False) collision.
        render_mode: Gymnasium render mode (``"human"``, ``"human_fast"``, ``"rgb_array"``).
    """

    # NOTE: change matadata with default rendering-modes, add definition of render_fps
    metadata = {"render_modes": ["human", "human_fast", "rgb_array"], "render_fps": 100}

    def __init__(self, config: dict = None, render_mode=None, **kwargs):
        super().__init__()

        # Configuration
        self.config = self.default_config()
        self.configure(config)

        self.seed = self.config["seed"]
        self.params = self.config["params"]
        self.num_agents = self.config["num_agents"]
        self.timestep = self.config["timestep"]
        self.ego_idx = self.config["ego_idx"]
        self.integrator = IntegratorType.from_string(self.config["integrator"])
        self.model = DynamicModel.from_string(self.config["model"])
        self.observation_config = self.config["observation_config"]
        self.normalize_act = self.config["normalize_act"]
        self.action_type = CarAction(self.config["control_input"], params=self.params, normalize=self.normalize_act)
        self.num_beams = self.config["num_beams"]

        # training mode
        self.training_mode = self.config["training_mode"]

        # conditionally set parameters based on training_mode
        match self.training_mode:
            case "race":
                self.map = self.config["map"]
                self.track_direction_config = self.config["track_direction"]
                self.max_episode_steps = self.config["max_episode_steps"]

            case "recover":
                # Recovery mode only supports single agent
                if self.num_agents != 1:
                    raise ValueError(
                        "Recovery mode only supports single agent training. "
                        f"Got num_agents={self.num_agents}, but recovery mode requires num_agents=1."
                    )

                # Recovery mode requires Frenet-based boundary checking
                if self.config["predictive_collision"]:
                    raise ValueError(
                        "Recovery mode requires Frenet-based boundary checking (predictive_collision=False)."
                    )

                # set recovery map and episode length
                self.map = self.config["recovery_map"]
                self.track_direction_config = self.config["recovery_track_direction"]
                self.max_episode_steps = self.config["recovery_max_episode_steps"]

                # set initial and final arc-lengths
                self.recovery_s_init = self.config["recovery_s_init"]
                self.recovery_s_max = self.config["recovery_s_max"]

                # set ranges for initial state perturbation
                self.recovery_v_range = self.config["recovery_v_range"]
                self.recovery_beta_range = self.config["recovery_beta_range"]
                self.recovery_yaw_range = self.config["recovery_yaw_range"]
                self.recovery_r_range = self.config["recovery_r_range"]

                # set reward parameters
                self.recovery_timestep_penalty = self.config["recovery_timestep_penalty"]
                self.recovery_success_reward = self.config["recovery_success_reward"]
                self.recovery_collision_penalty = self.config["recovery_collision_penalty"]

                # set recovery condition threshold values
                self.recovery_delta_thresh = self.config["recovery_delta_thresh"]
                self.recovery_beta_thresh = self.config["recovery_beta_thresh"]
                self.recovery_r_thresh = self.config["recovery_r_thresh"]
                self.recovery_d_beta_thresh = self.config["recovery_d_beta_thresh"]
                self.recovery_d_r_thresh = self.config["recovery_d_r_thresh"]
                self.recovery_frenet_u_thresh = self.config["recovery_frenet_u_thresh"]

                # initialize values for beta, r derivative tracking
                self.prev_beta = 0.0
                self.prev_r = 0.0

                # initialize recovery success flag for reward computation
                self.recovery_succeeded = False

            case _:
                raise ValueError(f"Invalid training_mode: '{self.training_mode}'")

        # rendering and debug configuration
        self.render_track_lines = self.config["render_track_lines"]
        self.render_arc_length_annotations = self.config["render_arc_length_annotations"]
        self.arc_length_annotation_interval = self.config["arc_length_annotation_interval"]
        self.render_lookahead_curvatures = self.config["render_lookahead_curvatures"]
        self.lookahead_n_points = self.config["lookahead_n_points"]
        self.lookahead_ds = self.config["lookahead_ds"]
        self.sparse_width_obs = self.config["sparse_width_obs"]
        self.debug_frenet_projection = self.config["debug_frenet_projection"]
        self.record_obs_min_max = self.config["record_obs_min_max"]

        # integration-instability detection: when enabled, RaceCar runs sanity
        # checks on the post-RK4 state, reverts on blow-up, and the env truncates
        self.prevent_instability = self.config["prevent_instability"]
        self.instability_bounds = {
            "yaw_rate": self.config["instability_yaw_rate_bound"],
            "slip": self.config["instability_slip_bound"],
        }

        # cumulative count of instability-truncation events; aggregated to wandb
        # by InstabilityCountCallback and printed at end-of-run
        self._instability_count = 0

        # reward params
        self.out_of_bounds_penalty = self.config["out_of_bounds_penalty"]
        self.progress_gain = self.config["progress_gain"]
        self.negative_vel_penalty = self.config["negative_vel_penalty"]
        self.current_step = 0

        # collision detection strategy
        self.predictive_collision = self.config["predictive_collision"]

        # wall deflection behavior
        self.wall_deflection = self.config["wall_deflection"]

        # Validate track direction config
        if self.track_direction_config not in ["normal", "reverse", "random"]:
            raise ValueError(
                f"Invalid track_direction: '{self.track_direction_config}'. "
                f"Must be one of: 'normal', 'reverse', 'random'"
            )

        # Set initial direction
        self._resolve_direction()

        if self.progress_gain < 1.0:
            raise ValueError("Progress gain must be >= 1.")

        if self.lookahead_n_points < 2:
            raise ValueError("Minimum of 2 lookahead track observation points required")

        # radius to consider done
        self.start_thresh = 0.5  # 10cm

        # env states
        self.poses_x = []
        self.poses_y = []
        self.poses_theta = []
        self.collisions = np.zeros((self.num_agents,))
        self.boundary_exceeded = np.zeros((self.num_agents,), dtype=bool)

        # loop completion
        self.near_start = True
        self.num_toggles = 0

        # race info
        self.lap_times = np.zeros((self.num_agents,))
        self.lap_counts = np.zeros((self.num_agents,))
        self.current_time = 0.0

        # finish line info
        self.num_toggles = 0
        self.near_start = True
        self.near_starts = np.array([True] * self.num_agents)
        self.toggle_list = np.zeros((self.num_agents,))
        self.start_xs = np.zeros((self.num_agents,))
        self.start_ys = np.zeros((self.num_agents,))
        self.start_thetas = np.zeros((self.num_agents,))
        self.start_rot = np.eye(2)

        # initiate stuff
        self.sim = Simulator(
            self.params,
            self.num_agents,
            self.seed,
            wall_deflection=self.wall_deflection,
            time_step=self.timestep,
            integrator=self.integrator,
            model=self.model,
            action_type=self.action_type,
            num_beams=self.num_beams,
            prevent_instability=self.prevent_instability,
            instability_bounds=self.instability_bounds,
        )
        self.sim.set_map(self.map, self.config["scale"])

        if isinstance(self.map, Track):
            self.track = self.map
        else:
            self.track = Track.from_track_name(
                self.map,
                track_scale=self.config["scale"],
            )  # load track in gym env for convenience

        # Set initial track direction
        self.track.set_direction(self.direction_reversed)

        # observations
        self.agent_ids = [f"agent_{i}" for i in range(self.num_agents)]

        assert "type" in self.observation_config, "observation_config must contain 'type' key"

        # steering min and steering velocity min should be symmetrical to max values
        assert self.params["s_min"] == -self.params["s_max"], "s_min must be equal to -s_max"
        assert self.params["sv_min"] == -self.params["sv_max"], "sv_min must be equal to -sv_max"

        # Store observation type for validation
        obs_type = self.observation_config["type"]

        # Validate drift observation requires STD model
        if obs_type == "drift" and self.model != DynamicModel.STD:
            raise ValueError(
                "The 'drift' observation type requires the single_track_drift (STD) model. "
                f"Current model: {self.model}. "
                "Please set model='std' (or model=DynamicModel.STD) when creating the environment."
            )

        # Validate drift_st observation requires ST or STP model
        if obs_type == "drift_st" and self.model not in [DynamicModel.ST, DynamicModel.STP]:
            raise ValueError(
                "The 'drift_st' observation type requires the single_track (ST) or single_track_pacejka (STP) model. "
                f"Current model: {self.model}. "
                "Please set model='st' or 'stp' (or model=DynamicModel.ST, model=DynamicModel.STP) when creating the environment."
            )

        # Handle normalization configuration
        normalize_obs = self.config["normalize_obs"]

        # Identify whether the chosen observation type is supported for normalization
        supported_obs_types = ["drift", "race", "frenet", "drift_st"]
        obs_norm_supported = obs_type in supported_obs_types

        if normalize_obs is None:
            # User did not set normalize - auto-set based on observation type
            # Default to True for observation types that support normalization (drift, race, etc.)
            self.normalize_obs = obs_norm_supported
        else:
            # User explicitly set normalize
            if normalize_obs and not obs_norm_supported:
                # If user wants normalization, but obs_type is incompatible, warn and overwrite normalize to False to prevent failures
                warnings.warn(
                    f"Observation normalization is only supported for {supported_obs_types} observation types, not '{obs_type}'. "
                    "Setting normalize_obs=False.",
                    UserWarning,
                )
                self.normalize_obs = False
            elif not normalize_obs and obs_norm_supported:
                # If user chose supported obs_type but set normalize to False, allow but warn
                warnings.warn(
                    f"Observation normalization is recommended for {obs_type} observation type but was disabled. "
                    "Verify this is intentional.",
                    UserWarning,
                )
                self.normalize_obs = False
            else:
                # In all other cases, accept user input
                self.normalize_obs = normalize_obs

        self.observation_type = observation_factory(env=self, **self.observation_config)
        self.observation_space = self.observation_type.space()

        # Initialize observation min/max tracking if requested.
        if self.record_obs_min_max:
            self.obs_min_max_tracker = {
                feature: {"min": float("inf"), "max": float("-inf")} for feature in self.observation_type.features
            }
            self.obs_tracker_step_count = 0

        # action space
        self.action_space = from_single_to_multi_action_space(self.action_type.space, self.num_agents)

        # reset modes
        self.reset_fn = make_reset_fn(**self.config["reset_config"], track=self.track, num_agents=self.num_agents)

        # stateful observations for rendering
        # add choice of colors (same, random, ...)
        self.render_obs = None
        self.render_mode = render_mode

        # match render_fps to integration timestep
        self.metadata["render_fps"] = int(1.0 / self.timestep)
        if self.render_mode == "human_fast":
            self.metadata["render_fps"] *= 10  # boost fps by 10x
        self.renderer, self.render_spec = make_renderer(
            params=self.params,
            track=self.track,
            agent_ids=self.agent_ids,
            render_mode=render_mode,
            render_fps=self.metadata["render_fps"],
            render_config=self.config.get("render_config"),
        )

        # automatically add track line rendering callback if configured
        if self.render_track_lines:
            self.add_render_callback(self.track.render_both_lines)

        # automatically add arc length annotation rendering callback if configured
        if self.render_arc_length_annotations:
            self.add_render_callback(
                lambda e: self.track.render_arc_length_annotations(e, interval=self.arc_length_annotation_interval)
            )

        # automatically add lookahead curvature rendering callback if configured
        if self.render_lookahead_curvatures:
            # Initialize cache for rendering callback's own s tracking
            self.add_render_callback(self._render_lookahead_curvatures_callback)

        # automatically add frenet projection debug visualization if configured
        if self.debug_frenet_projection:
            self.add_render_callback(self.track.render_frenet_projection)

    def _render_lookahead_curvatures_callback(self, e) -> None:
        """Render callback for lookahead curvature visualisation.

        Called during :meth:`render` to display the lookahead curvature
        sampling points ahead of the ego vehicle on the centerline.
        Uses ``lookahead_n_points`` and ``lookahead_ds`` from config.
        Silently skips on errors to avoid breaking rendering.

        Args:
            e: Environment renderer object.
        """
        try:
            # Get ego agent state
            agent = self.sim.agents[self.ego_idx]
            x, y, theta = agent.state[0], agent.state[1], agent.state[4]

            # Convert to Frenet coordinates to get arc length s
            s, _, _ = self.track.cartesian_to_frenet(x, y, theta, use_raceline=False)

            # Render the lookahead curvature points
            self.track.render_lookahead_curvatures(
                e=e, vehicle_s=s, n_points=self.lookahead_n_points, ds=self.lookahead_ds
            )
        except Exception:
            # Silently skip on errors to avoid breaking rendering
            pass

    @classmethod
    def fullscale_vehicle_params(cls) -> dict:
        """Return full-scale vehicle parameters from CommonRoad.

        Copied as-is from ``commonroad-vehicle-models/PYTHON/vehiclemodels/parameters/parameters_vehicle1.yaml``.

        Returns:
            Complete parameter dictionary for a full-scale vehicle (ST/MB/STD models).
        """
        return load_params("fullscale")

    @classmethod
    def f1fifth_vehicle_params(cls) -> dict:
        """Return default parameters for the 1/5th scale F1FIFTH car (ST model).

        Returns:
            Parameter dictionary for ST/KS models.
        """
        return load_params("f1fifth")

    @classmethod
    def f1tenth_vehicle_params(cls) -> dict:
        """Return default parameters for the 1/10th scale F1TENTH car (ST model).

        Returns:
            Parameter dictionary for ST/KS models.
        """
        return load_params("f1tenth_st")

    @classmethod
    def f1tenth_std_drift_bias_params(cls) -> dict:
        """Return STD model parameters tuned for increased drift tendency.

        Extends :meth:`f1tenth_std_vehicle_params` with adjusted CoG, reduced
        ``a_max``, softer lateral tyre stiffness, and rear-wheel-drive torque split.

        Returns:
            Parameter dictionary for the STD model with drift bias.
        """
        return load_params("f1tenth_std_drift_bias")

    @classmethod
    def f1tenth_std_vehicle_params(cls) -> dict:
        """Return default parameters for the 1/10th scale F1TENTH car (STD model).

        Extends the standard F1TENTH parameters with wheel dynamics (``R_w``,
        ``I_y_w``) and the full PAC2002 (Pacejka Magic Formula) tyre coefficient
        set, adapted from full-scale values. See ``gymkhana/envs/params/f1tenth_std.yaml``
        for values and derivation notes.

        Returns:
            Complete parameter dictionary for the STD model.
        """
        return load_params("f1tenth_std")

    @classmethod
    def f1tenth_stp_vehicle_params(cls) -> dict:
        """Return default parameters for the 1/10th scale F1TENTH car (STP model).

        Single Track Pacejka: dynamic single-track chassis with a lateral-only
        Pacejka Magic Formula tyre model (8 coefficients, ``B_f, C_f, D_f, E_f,
        B_r, C_r, D_r, E_r``). Coefficients are seeded from the on-track sysid
        pipeline output (``SIM_pacejka.txt``).

        Returns:
            Complete parameter dictionary for the STP model.
        """
        return load_params("f1tenth_stp")

    @classmethod
    def default_config(cls) -> dict:
        """Return the default environment configuration dict.

        All keys can be overridden by passing a partial ``config`` dict to
        ``__init__`` or by calling :meth:`configure`.

        Returns:
            Complete default configuration dict.
        """
        return {
            "seed": 12345,
            "map": "Spielberg",
            "scale": 1.0,
            "params": cls.f1tenth_vehicle_params(),
            "num_agents": 2,
            "timestep": 0.01,
            "ego_idx": 0,
            "integrator": "rk4",
            "model": "st",
            "control_input": ["speed", "steering_angle"],
            "observation_config": {"type": None},
            "reset_config": {"type": None},
            "scale": 1.0,
            "num_beams": 1080,
            "render_config": None,  # dict of overrides for rendering.yaml fields, or None to use packaged defaults
            "render_track_lines": False,
            "render_arc_length_annotations": False,
            "arc_length_annotation_interval": 2.0,
            "render_lookahead_curvatures": False,
            "lookahead_n_points": 10,
            "lookahead_ds": 0.3,
            "sparse_width_obs": False,
            "debug_frenet_projection": False,
            "normalize_obs": None,  # None = auto-set based on observation type
            "normalize_act": True,
            "predictive_collision": False,  # default Frenet-based boundary check
            "record_obs_min_max": False,
            "prevent_instability": False,  # when True, sanity-check post-integration state and truncate on blow-up
            "instability_yaw_rate_bound": 4 * np.pi,  # rad/s; standardized-state |yaw_rate| limit
            "instability_slip_bound": np.pi / 2,  # rad; standardized-state |slip| limit
            "wall_deflection": False,  # default to no wall deflections
            "track_direction": "normal",  # "normal", "reverse", or "random"
            "training_mode": "race",  # "race" or "recover"
            "out_of_bounds_penalty": -50,
            "progress_gain": 5.0,
            "negative_vel_penalty": -1,
            "max_episode_steps": 4096,
            # Recovery mode parameters (safe defaults, no effect when training_mode="race")
            "recovery_map": "IMS",
            "recovery_s_init": 96,
            "recovery_s_max": 140,
            "recovery_track_direction": "normal",  # separate from race track_direction, in case it is modified
            "recovery_v_range": [2, 12],
            "recovery_beta_range": [-0.349, 0.349],
            "recovery_r_range": [-0.785, 0.785],
            "recovery_yaw_range": [-0.785, 0.785],
            "recovery_timestep_penalty": 1.0,
            "recovery_success_reward": 100,
            "recovery_collision_penalty": -50,
            "recovery_delta_thresh": 0.175,  # approx 10 degrees steering
            "recovery_beta_thresh": 0.07,  # approx  4 degrees sideslip
            "recovery_r_thresh": 0.175,  # approx 10 deg/s yaw rate
            "recovery_d_beta_thresh": 7.0,  # 1/2 of the range [-0.07, 0.07] traversed in timestep 0.01: 0.07/0.01 = 7 radians/s
            "recovery_d_r_thresh": 17.5,  # 1/2 of the range [-0.175, 0.175] traversed in timestep 0.01: 0.175/0.01 = 17.5 radians/s^2
            "recovery_frenet_u_thresh": 0.087,  # approx 5 degrees heading error
            "recovery_max_episode_steps": 2048,
        }

    def configure(self, config: dict) -> None:
        """Merge a partial config dict into the current configuration.

        Also updates the simulator and action space if they are already
        initialised (i.e. when called after ``__init__``).

        Args:
            config: Partial configuration dict; keys are merged via deep update.
        """
        if config:
            self.config = deep_update(self.config, config)
            self.params = self.config["params"]

            if hasattr(self, "sim"):
                self.sim.update_params(self.config["params"])

            if hasattr(self, "action_space"):
                # if some parameters changed, recompute action space
                self.normalize_act = self.config["normalize_act"]
                self.action_type = CarAction(
                    self.config["control_input"], params=self.params, normalize=self.normalize_act
                )
                self.action_space = from_single_to_multi_action_space(self.action_type.space, self.num_agents)

    def _resolve_direction(self) -> None:
        """Set ``self.direction_reversed`` from ``self.track_direction_config``.

        ``"normal"`` → False, ``"reverse"`` → True, ``"random"`` → 50/50 coin flip.
        """
        if self.track_direction_config == "normal":
            self.direction_reversed = False
        elif self.track_direction_config == "reverse":
            self.direction_reversed = True
        else:  # "random"
            self.direction_reversed = np.random.random() < 0.5

    def _check_done(self):
        """Check whether the current episode should end.

        Distinguishes natural termination (collision/boundary) from truncation
        (time limit). Behaviour depends on ``training_mode``:

        - **Recovery** (single-agent):
            - Terminated: boundary exceeded OR recovery success.
            - Truncated: max steps exceeded OR arc-length > ``recovery_s_max``.
        - **Race** (1+ agents):
            - Terminated (TTC mode): ego TTC collision OR all agents complete 2 laps.
            - Terminated (Frenet mode): ego agent exceeds track boundaries.
            - Truncated: max episode steps exceeded.

        Returns:
            Tuple of ``(terminated, truncated, toggle_list)`` where
            ``toggle_list`` tracks each agent's finish-zone crossing count.
        """

        if self.training_mode == "recover":
            # Terminated: crash (boundary exceeded) or successful recovery
            self.recovery_succeeded = self._check_recovery_success()
            terminated = self.boundary_exceeded[0] or self.recovery_succeeded

            # Truncated: arc-length exceeded OR timestep limit
            current_s, _ = self.track.centerline.spline.calc_arclength_inaccurate(self.poses_x[0], self.poses_y[0])
            truncated = current_s > self.recovery_s_max or self.current_step > self.max_episode_steps
            return bool(terminated), bool(truncated), False

        # Race mode: lap-based termination logic
        else:
            # this is assuming 2 agents
            # TODO: switch to maybe s-based
            left_t = 2
            right_t = 2

            poses_x = np.array(self.poses_x) - self.start_xs
            poses_y = np.array(self.poses_y) - self.start_ys
            delta_pt = np.dot(self.start_rot, np.stack((poses_x, poses_y), axis=0))
            temp_y = delta_pt[1, :]
            idx1 = temp_y > left_t
            idx2 = temp_y < -right_t
            temp_y[idx1] -= left_t
            temp_y[idx2] = -right_t - temp_y[idx2]
            temp_y[np.invert(np.logical_or(idx1, idx2))] = 0

            dist2 = delta_pt[0, :] ** 2 + temp_y**2
            closes = dist2 <= 0.1
            for i in range(self.num_agents):
                if closes[i] and not self.near_starts[i]:
                    self.near_starts[i] = True
                    self.toggle_list[i] += 1
                elif not closes[i] and self.near_starts[i]:
                    self.near_starts[i] = False
                    self.toggle_list[i] += 1
                self.lap_counts[i] = self.toggle_list[i] // 2
                if self.toggle_list[i] < 4:
                    self.lap_times[i] = self.current_time

            # Determine natural termination based on collision detection mode
            if self.predictive_collision:
                terminated = (self.collisions[self.ego_idx]) or np.all(self.toggle_list >= 4)
            else:
                terminated = self.boundary_exceeded[self.ego_idx]

            # Truncate based on timestep limit
            truncated = self.current_step > self.max_episode_steps

            return bool(terminated), bool(truncated), self.toggle_list >= 4

    def _update_state(self):
        """Update env state from the simulator after a step.

        Note: when ``predictive_collision=False`` (Frenet mode), ``self.collisions``
        reflects the simulator TTC/GJK state but is unused for reward/reset.
        ``self.boundary_exceeded`` tracks Frenet-based boundary violations instead.
        """
        self.poses_x = self.sim.agent_poses[:, 0]
        self.poses_y = self.sim.agent_poses[:, 1]
        self.poses_theta = self.sim.agent_poses[:, 2]
        self.collisions = self.sim.collisions

        # Check boundaries once per step in Frenet mode
        if not self.predictive_collision:
            for i in range(self.num_agents):
                self.boundary_exceeded[i] = self._check_boundary_frenet(i)

    def _update_obs_min_max(self):
        """Update observation min/max tracker with the current step's values."""

        raw_features = getattr(self.observation_type, "_last_raw_features", None)
        if raw_features is None:
            return

        # FeaturesObservation stores nested {agent_id: {feature: value}}.
        # Collapse to {feature: [values across agents]} so the array branch
        # below produces a single per-feature min/max row across all agents.
        if raw_features and isinstance(next(iter(raw_features.values())), dict):
            raw_features = {
                f: [agent_obs[f] for agent_obs in raw_features.values()] for f in self.observation_type.features
            }

        self.obs_tracker_step_count += 1

        for feature_name, feature_value in raw_features.items():
            # np.min/np.max accept scalars, lists, and ndarrays uniformly.
            curr_min = float(np.min(feature_value))
            curr_max = float(np.max(feature_value))

            if not (np.isfinite(curr_min) and np.isfinite(curr_max)):
                continue

            entry = self.obs_min_max_tracker[feature_name]
            entry["min"] = min(entry["min"], curr_min)
            entry["max"] = max(entry["max"], curr_max)

    def _print_obs_min_max_stats(self):
        """Print this env's tracked obs min/max table. Called from :meth:`close`."""
        print_obs_min_max_stats(
            tracker=self.obs_min_max_tracker,
            step_count=self.obs_tracker_step_count,
            features=self.observation_type.features,
            bounds=getattr(self.observation_type, "bounds", {}) or {},
            normalize_obs=self.normalize_obs,
        )

    def _check_boundary_frenet(self, agent_idx: int) -> bool:
        """Check whether an agent has exceeded track boundaries using Frenet coordinates.

        Used when ``predictive_collision=False`` (drift mode) to detect exact
        boundary crossings based on lateral deviation rather than predictive TTC.

        Args:
            agent_idx: Index of the agent to check.

        Returns:
            True if the agent has exceeded track boundaries.

        Raises:
            RuntimeError: If Frenet coordinate conversion fails.
            ValueError: If track boundary data (``w_lefts``, ``w_rights``, ``ss``) is missing.
        """
        # Get agent position
        x = self.poses_x[agent_idx]
        y = self.poses_y[agent_idx]
        theta = self.poses_theta[agent_idx]

        # Convert to Frenet coordinates
        try:
            s, ey, _ = self.track.cartesian_to_frenet(x, y, theta, use_raceline=False)
        except Exception as e:
            raise RuntimeError(
                f"Frenet coordinate conversion failed for agent {agent_idx} at position ({x:.2f}, {y:.2f}). "
                f"This is required for boundary checking with predictive_collision=False. Error: {e}"
            ) from e

        centerline = self.track.centerline

        # Check if boundary data is available
        if centerline.w_lefts is None or centerline.w_rights is None:
            raise ValueError(
                "Track boundary data (w_lefts, w_rights) is not available. "
                "Frenet-based boundary checking requires track width information. "
                "Ensure track is loaded with boundary data"
            )

        if not hasattr(centerline, "ss") or centerline.ss is None:
            raise ValueError(
                "Centerline arc length data (ss) is not available. "
                "Frenet-based boundary checking requires arc length information for interpolation. "
                "Ensure track is properly initialized"
            )

        # Find nearest waypoint index - np.argmin is efficient for this use case without numba
        idx = np.argmin(np.abs(centerline.ss - s))

        # Get track width at this position
        w_left = centerline.w_lefts[idx]
        w_right = centerline.w_rights[idx]
        half_width = (w_left + w_right) / 2

        # Check boundary violation
        if abs(ey) > half_width:
            return True  # Boundary exceeded

        return False  # Within boundaries

    def _check_recovery_success(self) -> bool:
        """Check whether the vehicle is in a recovered (stable) state.

        Recovery is defined as all six quantities — ``delta``, ``beta``, ``r``,
        ``d_beta``, ``d_r``, and ``frenet_u`` — being within their respective
        threshold distances from zero. Thresholds are set in config. Definition
        derived from phase-plane analysis of the drift dynamics.

        Returns:
            True if the vehicle is considered recovered.
        """
        agent = self.sim.agents[0]
        std_state = agent.standard_state

        # Get current state values
        delta = std_state["delta"]
        beta = std_state["slip"]
        r = std_state["yaw_rate"]

        # Calculate beta, r derivatives using prev values
        d_beta = (beta - self.prev_beta) / self.timestep
        d_r = (r - self.prev_r) / self.timestep

        # Calculate heading error
        x, y, theta = std_state["x"], std_state["y"], std_state["yaw"]
        _, _, frenet_u = self.track.cartesian_to_frenet(x, y, theta, use_raceline=False)

        # Check recovery condition
        return (
            abs(delta) < self.recovery_delta_thresh
            and abs(beta) < self.recovery_beta_thresh
            and abs(r) < self.recovery_r_thresh
            and abs(d_beta) < self.recovery_d_beta_thresh
            and abs(d_r) < self.recovery_d_r_thresh
            and abs(frenet_u) < self.recovery_frenet_u_thresh
        )

    def _get_recovery_reward(self) -> float:
        """Compute the reward for the current recovery-mode step.

        Combines a boundary collision penalty, a recovery success bonus, and a
        constant timestep penalty.

        Returns:
            Total reward for this step.
        """
        # collision penalty
        r_col = self.recovery_collision_penalty if self.boundary_exceeded[0] else 0.0

        # recovery success reward
        r_rec = self.recovery_success_reward if self.recovery_succeeded else 0.0

        # constant timestep penalty
        r_const = self.recovery_timestep_penalty * self.timestep

        return r_col + r_rec - r_const

    def set_recovery_ranges(self, v_range, beta_range, r_range, yaw_range):
        """Set recovery initial state sampling ranges (used by curriculum learning)."""
        self.recovery_v_range = v_range
        self.recovery_beta_range = beta_range
        self.recovery_r_range = r_range
        self.recovery_yaw_range = yaw_range

    def _get_reward(self):
        """Compute the reward for the current step.

        Reward structure depends on ``training_mode``:

        - **Recovery**: delegates to :meth:`_get_recovery_reward`.
        - **Race (TTC mode)**: additive — ``sum(progress) - sum(collision_penalties)``.
        - **Race (Frenet mode)**: exclusive — ``out_of_bounds_penalty`` if boundary
          exceeded, else ``progress * progress_gain``.

        Returns:
            Scalar reward for the current step.
        """

        if self.training_mode == "recover":
            return self._get_recovery_reward()

        else:
            reward = 0.0
            track_length = self.track.centerline.spline.s[-1]

            for i in range(self.num_agents):
                # current_s calculated as distance along track centerline from start, in meters
                current_s, _ = self.track.centerline.spline.calc_arclength_inaccurate(self.poses_x[i], self.poses_y[i])

                # progress is current - previous arc length
                prog = current_s - self.last_s[i]

                # correct forward/backward track wraparound
                prog = self._correct_wraparound_prog(prog=prog, track_length=track_length)

                # reward forward progress, multiplied by progress_gain
                prog_r = prog * self.progress_gain

                # penalize negative longitudinal velocity v_x, as agent should not drive backward
                agent = self.sim.agents[i]
                vel = agent.standard_state["v_x"]
                if vel < 0:
                    reward += self.negative_vel_penalty

                # Apply reward based on collision detection strategy
                if self.predictive_collision:
                    # Predictive TTC mode: additive reward structure
                    # Reward = sum(progress) - sum(collision_penalties)
                    reward += prog_r
                    if self.collisions[i]:
                        reward += self.out_of_bounds_penalty
                else:
                    # Frenet boundary mode (drift): exclusive reward structure
                    # Reward = -1 if boundary exceeded, else progress
                    if self.boundary_exceeded[i]:
                        reward += self.out_of_bounds_penalty  # Exclusive penalty for boundary violation
                    else:
                        reward += prog_r  # Only get progress if within boundaries

                self.last_s[i] = current_s

            return reward

    def step(self, action, skip_integration=False):
        """Step the environment by one timestep.

        Args:
            action: Control inputs for all agents, shape ``(num_agents, 2)``.
                Each row is ``[steer, longitudinal]``.
            skip_integration: If True, skip dynamics integration (used during
                reset to generate an initial observation).

        Returns:
            Tuple ``(obs, reward, terminated, truncated, info)``.
        """

        # call simulation step
        self.sim.step(action, skip_integration=skip_integration)

        # detect numerical instability flagged by RaceCar.update_pose
        # (no-op when prevent_instability=False — agents never set .unstable)
        if self.prevent_instability:
            unstable_agents = [i for i, a in enumerate(self.sim.agents) if a.unstable]
            instability_truncation = len(unstable_agents) > 0
        else:
            unstable_agents = []
            instability_truncation = False

        # observation
        obs = self.observation_type.observe()

        # Track observation min/max if enabled — skip on unstable steps so a
        # single blow-up doesn't poison the cumulative tracker
        if self.record_obs_min_max and not instability_truncation:
            self._update_obs_min_max()

        # increment time and step
        self.current_time = self.current_time + self.timestep
        self.current_step += 1

        # update data member
        self._update_state()

        # rendering observation
        self.render_obs = {
            "ego_idx": self.sim.ego_idx,
            "poses_x": self.sim.agent_poses[:, 0],
            "poses_y": self.sim.agent_poses[:, 1],
            "poses_theta": self.sim.agent_poses[:, 2],
            "steering_angles": self.sim.agent_steerings,
            "lap_times": self.lap_times,
            "lap_counts": self.lap_counts,
            "collisions": self.sim.collisions,
            "sim_time": self.current_time,
        }
        if self.render_spec is not None and self.render_spec.show_ctr_debug:
            states = [a.standard_state for a in self.sim.agents]
            steer_type, throttle_type = self.action_type.type
            self.render_obs.update(
                {
                    "steering_cmds": np.array([a.curr_steering_cmd for a in self.sim.agents]),
                    "throttle_cmds": np.array([a.curr_throttle_cmd for a in self.sim.agents]),
                    "v_x": np.array([s["v_x"] for s in states]),
                    "delta": np.array([s["delta"] for s in states]),
                    "steer_bounds": self.action_type.steer_bounds,
                    "throttle_bounds": self.action_type.throttle_bounds,
                    "steer_type": steer_type,
                    "throttle_type": throttle_type,
                    "delta_bounds": (self.params["s_min"], self.params["s_max"]),
                    "vx_bounds": (self.params["v_min"], self.params["v_max"]),
                }
            )
        if self.render_spec is not None and self.render_spec.show_obs_debug:
            self.render_obs.update(
                {
                    "obs_debug_getter": self.observation_type.get_debug_features,
                    "obs_debug_normalize": getattr(self, "normalize_obs", False),
                }
            )

        # check done
        terminated, truncated, toggle_list = self._check_done()
        truncated = truncated or instability_truncation
        info = {
            "checkpoint_done": toggle_list,
            "episode_length": self.current_step,
            "instability_truncation": instability_truncation,
        }
        if instability_truncation:
            agent_infos = [self.sim.agents[i]._unstable_info for i in unstable_agents]
            info["unstable_agents"] = unstable_agents
            info["unstable_info"] = agent_infos
            self._instability_count += 1

        # calc reward
        reward = self._get_reward()

        # In recovery mode, update derivative tracking and add recovery success to info dict
        if self.training_mode == "recover":
            agent = self.sim.agents[0]
            self.prev_beta = agent.standard_state["slip"]
            self.prev_r = agent.standard_state["yaw_rate"]
            info["recovered"] = self.recovery_succeeded

        return obs, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        """Reset the environment and return an initial observation.

        Args:
            seed: Random seed for reproducibility.
            options: Optional dict with one of:

                - ``"poses"``: ``np.ndarray`` of shape ``(num_agents, 3)``
                  — ``[x, y, yaw]`` per agent.
                - ``"states"``: ``np.ndarray`` of shape ``(num_agents, n)`` where
                  ``n`` is the model-specific row width; see
                  :meth:`gymkhana.envs.dynamic_models.DynamicModel.user_state_len`
                  for accepted widths and layouts. MB does not support
                  full-state reset.

                Cannot specify both keys simultaneously.

        Returns:
            Tuple ``(obs, info)``.
        """
        if seed is not None:
            np.random.seed(seed=seed)
        super().reset(seed=seed)

        # Re-randomize direction for random
        if self.track_direction_config == "random":
            self._resolve_direction()

        # Swap Track's active centerline/raceline references
        self.track.set_direction(self.direction_reversed)

        # reset counters and data members
        self.current_time = 0.0
        self.collisions = np.zeros((self.num_agents,))
        self.boundary_exceeded = np.zeros((self.num_agents,), dtype=bool)
        self.num_toggles = 0
        self.near_start = True
        self.near_starts = np.array([True] * self.num_agents)
        self.toggle_list = np.zeros((self.num_agents,))
        self.current_step = 0

        # Parse options for poses and states
        poses = None
        states = None

        if options is not None:
            has_poses = "poses" in options
            has_states = "states" in options

            # Mutual exclusion check
            if has_poses and has_states:
                raise ValueError("Cannot provide both 'poses' and 'states' in reset options.")

            if has_states:
                # Per-model expected width (and MB rejection) come from
                # DynamicModel.user_state_len; this layer only validates that
                # the supplied 2-D array matches (num_agents, expected_len).
                expected_state_len = self.model.user_state_len()

                states = options["states"]
                if not isinstance(states, np.ndarray) or states.shape != (
                    self.num_agents,
                    expected_state_len,
                ):
                    raise ValueError(
                        f"States must be a numpy array of shape "
                        f"(num_agents={self.num_agents}, {expected_state_len}) "
                        f"for model {self.model.name}, got "
                        f"{states.shape if isinstance(states, np.ndarray) else type(states).__name__}"
                    )

                # Save poses for downstream use
                poses = np.column_stack([states[:, 0], states[:, 1], states[:, 4]])

            elif has_poses:
                poses = options["poses"]

        # Recovery mode: generate perturbed initial state if no options given
        if self.training_mode == "recover" and states is None and poses is None:
            # uniformly sample ranges for perturbations
            v = np.random.uniform(*self.recovery_v_range)
            beta = np.random.uniform(*self.recovery_beta_range)
            r = np.random.uniform(*self.recovery_r_range)
            yaw_perturbation = np.random.uniform(*self.recovery_yaw_range)

            # generate initial states and poses
            x, y, base_yaw = self.track.frenet_to_cartesian(self.recovery_s_init, ey=0, ephi=0)
            yaw = base_yaw + yaw_perturbation
            states = np.array([[x, y, 0.0, v, yaw, r, beta]])
            poses = np.column_stack([states[:, 0], states[:, 1], states[:, 4]])

        # If no poses derived yet, sample from reset function
        if poses is None:
            poses = self.reset_fn.sample()

        assert isinstance(poses, np.ndarray) and poses.shape == (
            self.num_agents,
            3,
        ), "Initial poses must be a numpy array of shape (num_agents, 3)"

        self.start_xs = poses[:, 0]
        self.start_ys = poses[:, 1]
        self.start_thetas = poses[:, 2]
        self.start_rot = np.array(
            [
                [
                    np.cos(-self.start_thetas[self.ego_idx]),
                    -np.sin(-self.start_thetas[self.ego_idx]),
                ],
                [
                    np.sin(-self.start_thetas[self.ego_idx]),
                    np.cos(-self.start_thetas[self.ego_idx]),
                ],
            ]
        )

        # call reset to simulator (pass states if provided)
        self.sim.reset(poses, states=states)

        # Initialize last_s to actual starting arc lengths for accurate first-step reward
        self.last_s = []
        for i in range(self.num_agents):
            s, _, _ = self.track.cartesian_to_frenet(poses[i, 0], poses[i, 1], poses[i, 2], use_raceline=False)
            self.last_s.append(s)

        # get no input observations without integrating dynamics, so that RaceCar states are not changed
        action = np.zeros((self.num_agents, 2))
        obs, _, _, _, info = self.step(action, skip_integration=True)

        # Initialize derivative tracking for recovery mode
        if self.training_mode == "recover":
            agent = self.sim.agents[0]
            self.prev_beta = agent.standard_state["slip"]
            self.prev_r = agent.standard_state["yaw_rate"]

        return obs, info

    def _correct_wraparound_prog(self, prog: float, track_length: float, margin=10.0) -> float:
        """Correct arc-length progress for track wraparound and clip to physical limits.

        Args:
            prog: Raw progress in metres for the current timestep.
            track_length: Total track length for wraparound correction.
            margin: Multiplier on ``v_max * timestep`` used as the clip limit (default 10).

        Returns:
            Corrected and clipped progress in metres.
        """
        # max progress used for clipping
        max_progress = self.params["v_max"] * self.timestep * margin
        # half track length used for wraparound detection
        half_track = track_length / 2

        # forward wraparound
        if prog < -half_track:
            prog += track_length

        # backward wraparound
        elif prog > half_track:
            prog -= track_length

        # clip in case progress is still out of bounds
        return np.clip(prog, -max_progress, max_progress)

    def update_map(self, map_name: str):
        """Update the map used by the simulation.

        Args:
            map_name: Name of the map.
        """
        self.sim.set_map(map_name)
        self.track = Track.from_track_name(map_name)

    def update_params(self, params, index=-1):
        """Update vehicle parameters for the simulation.

        Args:
            params: Dictionary of vehicle parameters.
            index: If >= 0, update only the specified agent's params; -1 updates all.
        """
        self.sim.update_params(params, agent_idx=index)

    def add_render_callback(self, callback_func):
        """Add an extra drawing function to call during rendering.

        Args:
            callback_func: Callable with signature ``(EnvRenderer) -> None`` invoked each :meth:`render` call.
        """
        if self.renderer is not None:
            self.renderer.add_renderer_callback(callback_func)

    def render(self, mode="human"):
        """Render the environment.

        Mouse scroll zooms in/out; click-drag pans. Displays agents, map,
        current FPS, and race information.

        Args:
            mode: Rendering mode — ``"human"`` (real-time paced) or ``"human_fast"`` (uncapped).
        """
        # NOTE: separate render (manage render-mode) from render_frame (actual rendering with pyglet)

        if self.render_mode not in self.metadata["render_modes"]:
            return

        self.renderer.update(state=self.render_obs)
        return self.renderer.render()

    def disable_obs_min_max_recording(self):
        """Mute the close()-time print. Called by the SubprocVecEnv aggregator
        via ``env_method`` so the flag is set on the inner GKEnv, not the
        Monitor wrapper (which is what ``set_attr`` would target)."""
        self.record_obs_min_max = False

    def close(self):
        """Close the environment and release renderer resources."""
        # Print observation statistics if tracking was enabled
        if self.record_obs_min_max:
            self._print_obs_min_max_stats()

        if self.renderer is not None:
            self.renderer.close()
        super().close()
