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


"""
Prototype of base classes
Replacement of the old RaceCar, Simulator classes in C++
Author: Hongrui Zheng, Teodor Ilie
"""

from __future__ import annotations

import numpy as np

from .action import CarAction
from .collision_models import collision_multiple, get_vertices
from .dynamic_models import DynamicModel
from .integrator import EulerIntegrator, Integrator
from .laser_models import ScanSimulator2D, check_ttc_jit, ray_cast
from .track import Track


class RaceCar(object):
    """Single vehicle physics and laser scan simulation.

    Attributes:
        params: Vehicle parameters dictionary.
        is_ego: Whether this is the ego vehicle.
        time_step: Physics timestep in seconds.
        num_beams: Number of beams in the laser scan.
        fov: Field of view of the laser in radians.
        state: State vector (size depends on model, e.g. 7 for ST).
        accel: Current acceleration input.
        steer_angle_vel: Current steering velocity input.
        in_collision: Whether the vehicle is currently in collision.
    """

    # static objects that don't need to be stored in class instances
    scan_simulator = None
    cosines = None
    scan_angles = None
    side_distances = None

    def __init__(
        self,
        params,
        seed,
        wall_deflection,
        action_type: CarAction,
        integrator=EulerIntegrator(),
        model=DynamicModel.ST,
        is_ego=False,
        time_step=0.01,
        num_beams=1080,
        fov=4.7,
        prevent_instability=False,
        instability_bounds=None,
    ):
        """Initialize a RaceCar instance.

        Args:
            params: Vehicle parameters dictionary.
            seed: Random seed for scan simulation.
            wall_deflection: If True, vehicle stops on wall contact instead of bouncing.
            action_type: Action handler (steering + longitudinal).
            integrator: ODE integrator for dynamics stepping.
            model: Vehicle dynamics model.
            is_ego: Whether this is the ego vehicle.
            time_step: Physics simulation timestep in seconds.
            num_beams: Number of beams in the laser scan.
            fov: Field of view of the laser in radians.
            prevent_instability: If True, run the post-integration sanity check
                and revert state on blow-up.
            instability_bounds: Mapping from standardized-state feature name
                to absolute-value limit (e.g. ``{"yaw_rate": 4*pi, "slip": pi/2}``).
        """

        # initialization
        self.params = params
        self.seed = seed
        self.is_ego = is_ego
        self.time_step = time_step
        self.num_beams = num_beams
        self.fov = fov
        self.integrator = integrator
        self.action_type = action_type
        self.model = model
        self.standard_state_fn = self.model.get_standardized_state_fn()
        self.wall_deflection = wall_deflection
        self.prevent_instability = prevent_instability
        self.instability_bounds = instability_bounds if instability_bounds is not None else {}

        # state of the vehicle
        self.state = self.model.get_initial_state(params=self.params)

        # pose of opponents in the world
        self.opp_poses = None

        # control inputs
        self.accel = 0.0
        self.steer_angle_vel = 0.0

        # steering delay buffer
        self.steer_buffer = np.empty((0,))
        self.steer_buffer_size = 2

        # collision identifier
        self.in_collision = False

        # numerical-instability flag: set when post-RK4 state violates sanity
        # bounds and is reverted to the prior step's state
        self.unstable = False
        self._unstable_info = None

        # collision threshold for iTTC to environment
        self.ttc_thresh = 0.005

        # previous, current steering command for observations
        self.prev_steering_cmd = 0.0
        self.curr_steering_cmd = 0.0

        # previous, current throttle command for observations (either target vel or accl)
        self.prev_throttle_cmd = 0.0
        self.curr_throttle_cmd = 0.0

        # previous, current actual acceleration command
        self.prev_accl_cmd = 0.0
        self.curr_accl_cmd = 0.0

        # previous, current average wheel angular velocity (for STD model)
        self.prev_avg_wheel_omega = 0.0
        self.curr_avg_wheel_omega = 0.0

        # current commanded velocity (integrated from acceleration)
        self.curr_vel_cmd = 0.0

        # initialize scan sim
        if RaceCar.scan_simulator is None:
            self.scan_rng = np.random.default_rng(seed=self.seed)
            RaceCar.scan_simulator = ScanSimulator2D(num_beams, fov)

            scan_ang_incr = RaceCar.scan_simulator.get_increment()

            # angles of each scan beam, distance from lidar to edge of car at each beam, and precomputed cosines of each angle
            RaceCar.cosines = np.zeros((num_beams,))
            RaceCar.scan_angles = np.zeros((num_beams,))
            RaceCar.side_distances = np.zeros((num_beams,))

            dist_sides = params["width"] / 2.0
            dist_fr = (params["lf"] + params["lr"]) / 2.0

            for i in range(num_beams):
                angle = -fov / 2.0 + i * scan_ang_incr
                RaceCar.scan_angles[i] = angle
                RaceCar.cosines[i] = np.cos(angle)

                if angle > 0:
                    if angle < np.pi / 2:
                        # between 0 and pi/2
                        to_side = dist_sides / np.sin(angle)
                        to_fr = dist_fr / np.cos(angle)
                        RaceCar.side_distances[i] = min(to_side, to_fr)
                    else:
                        # between pi/2 and pi
                        to_side = dist_sides / np.cos(angle - np.pi / 2.0)
                        to_fr = dist_fr / np.sin(angle - np.pi / 2.0)
                        RaceCar.side_distances[i] = min(to_side, to_fr)
                else:
                    if angle > -np.pi / 2:
                        # between 0 and -pi/2
                        to_side = dist_sides / np.sin(-angle)
                        to_fr = dist_fr / np.cos(-angle)
                        RaceCar.side_distances[i] = min(to_side, to_fr)
                    else:
                        # between -pi/2 and -pi
                        to_side = dist_sides / np.cos(-angle - np.pi / 2)
                        to_fr = dist_fr / np.sin(-angle - np.pi / 2)
                        RaceCar.side_distances[i] = min(to_side, to_fr)

    def update_params(self, params):
        """Update the physical parameters of the vehicle.

        Args:
            params: New vehicle parameters dictionary.
        """
        self.params = params

    def set_map(self, map: str | Track, map_scale: float = 1.0):
        """Set the map for the scan simulator.

        Args:
            map: Map name or a :class:`Track` object.
            map_scale: Scale factor for the map (default 1.0).
        """
        RaceCar.scan_simulator.set_map(map, map_scale)

    def reset(self, pose, state=None):
        """Reset the vehicle to a pose or full state.

        Args:
            pose: Pose to reset to, shape ``(3,)`` as ``[x, y, yaw]``.
            state: Optional model-specific user-facing state row; see
                :meth:`gymkhana.envs.dynamic_models.DynamicModel.user_state_lens`
                for accepted widths and layouts. MB does not support full-state
                reset. If provided, ``pose`` is ignored.
        """
        # clear control inputs
        self.accel = 0.0
        self.steer_angle_vel = 0.0
        # clear collision indicator
        self.in_collision = False
        # clear instability flag
        self.unstable = False
        self._unstable_info = None
        # clear previous and current steering commands
        self.prev_steering_cmd = 0.0
        self.curr_steering_cmd = 0.0
        # clear previous and current throttle commands
        self.prev_throttle_cmd = 0.0
        self.curr_throttle_cmd = 0.0
        # clear previous, current actual acceleration command
        self.prev_accl_cmd = 0.0
        self.curr_accl_cmd = 0.0
        # clear previous, current average wheel angular velocity
        self.prev_avg_wheel_omega = 0.0
        self.curr_avg_wheel_omega = 0.0
        # init state from pose OR full state
        if state is not None:
            self.state = self.model.get_initial_state(state=state, params=self.params)
        else:
            self.state = self.model.get_initial_state(pose=pose, params=self.params)
        # initialize commanded velocity to match initial state velocity
        self.curr_vel_cmd = self.state[3]

        self.steer_buffer = np.empty((0,))
        # reset scan random generator
        self.scan_rng = np.random.default_rng(seed=self.seed)

    def ray_cast_agents(self, scan):
        """Ray cast onto other agents, returning a modified scan.

        Args:
            scan: Original scan range array, shape ``(n,)``.

        Returns:
            Modified scan with other agents occluding rays, shape ``(n,)``.
        """

        # starting from original scan
        new_scan = scan

        # loop over all opponent vehicle poses
        for opp_pose in self.opp_poses:
            # get vertices of current oppoenent
            opp_vertices = get_vertices(opp_pose, self.params["length"], self.params["width"])

            new_scan = ray_cast(
                np.append(self.state[0:2], self.state[4]),
                new_scan,
                self.scan_angles,
                opp_vertices,
            )

        return new_scan

    def check_ttc(self, current_scan):
        """Check inverse Time-To-Collision (iTTC) against the environment.

        Sets collision state if iTTC threshold is exceeded. Does not check
        collision with other agents (handled separately via GJK).

        Args:
            current_scan: Current laser scan array.

        Returns:
            True if a collision is detected, False otherwise.
        """

        in_collision = check_ttc_jit(
            current_scan,
            self.state[3],
            self.scan_angles,
            self.cosines,
            self.side_distances,
            self.ttc_thresh,
        )

        # if in collision, and wall_deflection feature enabled, stop vehicle
        if in_collision and self.wall_deflection:
            self.state[3:] = 0.0
            self.accel = 0.0
            self.steer_angle_vel = 0.0

        # update state
        self.in_collision = in_collision

        return in_collision

    def update_pose(self, raw_steer, raw_throttle, skip_integration=False):
        """Step the vehicle's physical simulation by one timestep.

        Args:
            raw_steer: Desired steering angle or velocity, depending on action type.
            raw_throttle: Desired longitudinal velocity or acceleration, depending on action type.
            skip_integration: If True, skip dynamics integration (used during reset to generate obs).

        Returns:
            Current laser scan array after updating position.
        """

        # steering: update prev to curr, and current to new action input raw_steer
        self.prev_steering_cmd = self.curr_steering_cmd
        self.curr_steering_cmd = raw_steer

        # throttle: update prev to curr, and current to new action input vel
        self.prev_throttle_cmd = self.curr_throttle_cmd
        self.curr_throttle_cmd = raw_throttle

        # update average wheel angular velocity (for STD model)
        self.prev_avg_wheel_omega = self.curr_avg_wheel_omega
        if len(self.state) >= 9:  # STD model has 9 states including wheel angular velocities
            self.curr_avg_wheel_omega = (self.state[7] + self.state[8]) / 2.0

        # steering delay
        steer = 0.0
        if self.steer_buffer.shape[0] < self.steer_buffer_size:
            steer = 0.0
            self.steer_buffer = np.append(raw_steer, self.steer_buffer)
        else:
            steer = self.steer_buffer[-1]
            self.steer_buffer = self.steer_buffer[:-1]
            self.steer_buffer = np.append(raw_steer, self.steer_buffer)

        if self.action_type.type is None:
            raise ValueError("No Control Action Type Specified.")

        accl, sv = self.action_type.act(action=(steer, raw_throttle), state=self.state, params=self.params)

        # acceleration: update prev to curr, and current to new throttle cmd accl
        self.prev_accl_cmd = self.curr_accl_cmd
        self.curr_accl_cmd = accl

        # commanded velocity: integrate using acceleration and clip between v_min and v_max
        self.curr_vel_cmd = self.curr_vel_cmd + accl * self.time_step
        self.curr_vel_cmd = np.clip(self.curr_vel_cmd, self.params["v_min"], self.params["v_max"])

        u_np = np.array([sv, accl])

        # Conditionally integrate dynamics (skip during reset to preserve exact state)
        self.unstable = False
        self._unstable_info = None
        if not skip_integration:
            f_dynamics = self.model.f_dynamics
            prev_state = self.state.copy()
            self.state = self.integrator.integrate(
                f=f_dynamics, x=self.state, u=u_np, dt=self.time_step, params=self.params
            )

            # bound yaw angle
            self.state[4] %= 2 * np.pi  # TODO: This is a problem waiting to happen

            # numerical-stability check: if RK4 produced a non-physical state,
            # revert to prev_state and flag for env-level truncation
            if self.prevent_instability:
                self._check_state_sanity(prev_state, u_np)

        # update scan
        current_scan = RaceCar.scan_simulator.scan(np.append(self.state[0:2], self.state[4]), self.scan_rng)

        return current_scan

    def _check_state_sanity(self, prev_state, u_np):
        """Validate post-integration state; revert and flag on blow-up.

        Args:
            prev_state: Pre-integration state vector (used to revert on failure).
            u_np: Action vector applied this step (recorded for diagnostics).
        """
        violations = {}

        if not np.all(np.isfinite(self.state)):
            violations["non_finite"] = True
        else:
            std = self.standard_state_fn(self.state)
            for feature, bound in self.instability_bounds.items():
                value = std.get(feature, 0.0)
                if abs(value) > bound:
                    violations[feature] = float(value)

        if violations:
            self.unstable = True
            self._unstable_info = {
                "violations": violations,
                "action": np.asarray(u_np).tolist(),
            }
            self.state = prev_state

    def update_opp_poses(self, opp_poses):
        """Update this vehicle's information about other agents.

        Args:
            opp_poses: Poses of all other agents, shape ``(num_other_agents, 3)``.
        """
        self.opp_poses = opp_poses

    def update_scan(self, agent_scans, agent_index):
        """Update this vehicle's laser scan based on current agent positions.

        Separated from :meth:`update_pose` because scans must be updated after
        all agents have moved to their new positions.

        Args:
            agent_scans: List of all agent scans, modified in-place.
            agent_index: Index of this vehicle in the scans list.
        """

        current_scan = agent_scans[agent_index]

        # check ttc
        self.check_ttc(current_scan)

        # ray cast other agents to modify scan
        new_scan = self.ray_cast_agents(current_scan)

        agent_scans[agent_index] = new_scan

    @property
    def standard_state(self) -> dict:
        """Standardized state dict with keys: x, y, delta, v_x, v_y, yaw, yaw_rate, slip."""
        return self.standard_state_fn(self.state)


class Simulator(object):
    """Multi-agent simulation orchestrator.

    Manages multiple :class:`RaceCar` instances, steps their dynamics simultaneously,
    and checks collisions between agents and with track boundaries.

    Attributes:
        num_agents: Number of agents in the environment.
        time_step: Physics time step in seconds.
        agent_poses: All agent poses as array of shape ``(num_agents, 3)``.
        agents: List of :class:`RaceCar` instances.
        collisions: Collision indicator per agent, shape ``(num_agents,)``.
        collision_idx: Index of the agent each agent is colliding with, ``-1`` if none.
    """

    def __init__(
        self,
        params,
        num_agents,
        seed,
        num_beams,
        wall_deflection,
        action_type: CarAction,
        integrator=Integrator,
        model=DynamicModel.ST,
        time_step=0.01,
        ego_idx=0,
        prevent_instability=False,
        instability_bounds=None,
    ):
        """Initialize the Simulator.

        Args:
            params: Vehicle parameters dictionary.
            num_agents: Number of agents in the environment.
            seed: Random seed for scan simulation.
            num_beams: Number of laser beams per agent.
            wall_deflection: If True, vehicles stop on wall contact.
            action_type: Shared action handler for all agents.
            integrator: ODE integrator for dynamics stepping.
            model: Vehicle dynamics model used by all agents.
            time_step: Physics time step in seconds.
            ego_idx: Index of the ego vehicle in the agents list.
            prevent_instability: Forwarded to each :class:`RaceCar` to enable
                post-integration sanity checks and state revert on blow-up.
            instability_bounds: Forwarded to each :class:`RaceCar` as the
                per-feature absolute-value limits used by the sanity check.
        """
        self.num_agents = num_agents
        self.seed = seed
        self.time_step = time_step
        self.ego_idx = ego_idx
        self.params = params
        self.agent_poses = np.empty((self.num_agents, 3))
        self.agent_steerings = np.empty((self.num_agents,))
        self.agents: list[RaceCar] = []
        self.collisions = np.zeros((self.num_agents,))
        self.collision_idx = -1 * np.ones((self.num_agents,))
        self.model = model
        self.wall_deflection = wall_deflection

        # initializing agents
        for i in range(self.num_agents):
            car = RaceCar(
                params,
                self.seed,
                wall_deflection=self.wall_deflection,
                num_beams=num_beams,
                is_ego=bool(i == ego_idx),
                time_step=self.time_step,
                integrator=integrator,
                model=model,
                action_type=action_type,
                prevent_instability=prevent_instability,
                instability_bounds=instability_bounds,
            )
            self.agents.append(car)

        # initialize agents scan, to be accessed from observation types
        num_beams = self.agents[0].scan_simulator.num_beams
        self.agent_scans = np.empty((self.num_agents, num_beams))

    def set_map(self, map: str | Track, map_scale: float = 1.0):
        """Set the map for all agents' scan simulators.

        Args:
            map: Map name or a :class:`Track` object.
            map_scale: Scale factor for the map (default 1.0).
        """
        for agent in self.agents:
            agent.set_map(map, map_scale)

    def update_params(self, params, agent_idx=-1):
        """Update vehicle parameters for one or all agents.

        Args:
            params: Vehicle parameters dictionary.
            agent_idx: Index of the agent to update. If negative, updates all agents.

        Raises:
            IndexError: If ``agent_idx`` is out of range.
        """
        self.params = params
        if agent_idx < 0:
            # update params for all
            for agent in self.agents:
                agent.update_params(params)
        elif agent_idx >= 0 and agent_idx < self.num_agents:
            # only update one agent's params
            self.agents[agent_idx].update_params(params)
        else:
            # index out of bounds, throw error
            raise IndexError("Index given is out of bounds for list of agents.")

    def check_collision(self):
        """Check for collisions between agents using GJK and body vertices."""
        # get vertices of all agents
        all_vertices = np.empty((self.num_agents, 4, 2))
        for i in range(self.num_agents):
            all_vertices[i, :, :] = get_vertices(
                np.append(self.agents[i].state[0:2], self.agents[i].state[4]),
                self.params["length"],
                self.params["width"],
            )
        self.collisions, self.collision_idx = collision_multiple(all_vertices)

    def step(self, control_inputs, skip_integration=False):
        """Step the simulation by one timestep for all agents.

        Args:
            control_inputs: Control inputs for all agents, shape ``(num_agents, 2)``.
                Each row is ``[steering, longitudinal]``.
            skip_integration: If True, skip dynamics integration (used during reset).
        """

        # looping over agents
        for i, agent in enumerate(self.agents):
            # update each agent's pose
            current_scan = agent.update_pose(
                control_inputs[i, 0], control_inputs[i, 1], skip_integration=skip_integration
            )
            self.agent_scans[i, :] = current_scan

            # update sim's information of agent poses
            self.agent_poses[i, :] = np.append(agent.state[0:2], agent.state[4])
            self.agent_steerings[i] = agent.state[2]

        # check collisions between all agents
        self.check_collision()

        for i, agent in enumerate(self.agents):
            # update agent's information on other agents
            opp_poses = np.concatenate((self.agent_poses[0:i, :], self.agent_poses[i + 1 :, :]), axis=0)
            agent.update_opp_poses(opp_poses)

            # update each agent's current scan based on other agents
            agent.update_scan(self.agent_scans, i)

            # update agent collision with environment
            if agent.in_collision:
                self.collisions[i] = 1.0

    def reset(self, poses, states=None):
        """Reset all agents to given poses or full states.

        Args:
            poses: Poses for all agents, shape ``(num_agents, 3)``.
            states: Optional full states for all agents, shape ``(num_agents, n)``
                where ``n`` is the model-specific row width; see
                :meth:`gymkhana.envs.dynamic_models.DynamicModel.user_state_lens`
                for accepted widths and layouts. MB does not support full-state
                reset. If provided, ``poses`` is ignored.

        Raises:
            ValueError: If the number of poses or states does not match ``num_agents``.
        """

        if poses.shape[0] != self.num_agents:
            raise ValueError("Number of poses for reset does not match number of agents.")

        if states is not None and states.shape[0] != self.num_agents:
            raise ValueError("Number of states for reset does not match number of agents.")

        # loop over poses to reset
        for i in range(self.num_agents):
            agent_state = states[i, :] if states is not None else None
            self.agents[i].reset(poses[i, :], state=agent_state)
