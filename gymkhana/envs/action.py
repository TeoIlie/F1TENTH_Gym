"""Action types and normalization for vehicle control inputs."""

import warnings
from abc import abstractmethod
from enum import Enum
from typing import Any, Dict, Tuple

import gymnasium as gym
import numpy as np

from .dynamic_models import bang_bang_steer, p_accl


class LongitudinalActionEnum(Enum):
    """Enum for longitudinal action types.

    Members:
        Accl: Direct acceleration control.
        Speed: Target speed control (converted to acceleration via P controller).
    """

    Accl = 1
    Speed = 2

    @staticmethod
    def from_string(action: str):
        """Return the action class for the given string identifier.

        Args:
            action: One of ``"accl"`` or ``"speed"``.

        Returns:
            The corresponding action class.

        Raises:
            ValueError: If the action string is not recognised.
        """
        if action == "accl":
            return AcclAction
        elif action == "speed":
            return SpeedAction
        else:
            raise ValueError(f"Unknown action type {action}")

    @staticmethod
    def is_valid(action: str) -> bool:
        """Check whether a string is a valid longitudinal action type."""
        return action in ("accl", "speed")


class LongitudinalAction:
    """Base class for longitudinal (speed/acceleration) actions.

    Subclasses implement :meth:`act` to convert a raw action value into an
    acceleration command suitable for the dynamics model.

    Attributes:
        normalize: Whether actions are normalized to ``[-1, 1]``.
        lower_limit: Lower bound of the action space.
        upper_limit: Upper bound of the action space.
        scale_factor: Multiplier applied when denormalizing actions.
    """

    def __init__(self, normalize: bool) -> None:
        self._type = None
        self.normalize = normalize
        self.lower_limit = None
        self.upper_limit = None
        self.scale_factor = 1.0

    @abstractmethod
    def act(self, longitudinal_action: Any, **kwargs) -> float:
        raise NotImplementedError("longitudinal act method not implemented")

    @property
    def type(self) -> str:
        """String identifier for this action type (e.g. ``"accl"``, ``"speed"``)."""
        return self._type

    @property
    def space(self) -> gym.Space:
        """Gymnasium Box space for this action dimension."""
        return gym.spaces.Box(low=self.lower_limit, high=self.upper_limit, dtype=np.float32)


class AcclAction(LongitudinalAction):
    """Direct acceleration control action.

    When normalized, maps ``[-1, 1]`` to ``[-a_max, a_max]``.
    """

    def __init__(self, params: Dict, normalize: bool) -> None:
        super().__init__(normalize=normalize)
        self._type = "accl"

        if normalize:
            self.lower_limit = -1.0
            self.upper_limit = 1.0
            self.scale_factor = params["a_max"]
        else:
            self.lower_limit = -params["a_max"]
            self.upper_limit = params["a_max"]
            self.scale_factor = 1.0

    def act(self, action: float, state, params) -> float:
        """Return the acceleration command.

        Args:
            action: Normalized ``[-1, 1]`` or raw acceleration value.
            state: Vehicle state vector (unused for direct acceleration).
            params: Vehicle parameters dict.

        Returns:
            Acceleration in m/s^2.
        """
        # When normalize=True: maps [-1, 1] → [-a_max, a_max]
        # When normalize=False: pass-through (scale_factor=1.0)
        return action * self.scale_factor


class SpeedAction(LongitudinalAction):
    """Target speed control action, converted to acceleration via a P controller.

    When normalized, maps ``[-1, 1]`` to ``[v_min, v_max]``.
    """

    def __init__(self, params: Dict, normalize: bool) -> None:
        super().__init__(normalize=normalize)
        self._type = "speed"

        if normalize:
            self.lower_limit = -1.0
            self.upper_limit = 1.0
            # Mapping: normalized * range + center
            self.v_center = (params["v_max"] + params["v_min"]) / 2.0
            self.v_range = (params["v_max"] - params["v_min"]) / 2.0
        else:
            self.lower_limit = params["v_min"]
            self.upper_limit = params["v_max"]

    def act(self, action: float, state: np.ndarray, params: Dict) -> float:
        """Return the acceleration command for a desired speed.

        Args:
            action: Normalized ``[-1, 1]`` or raw target speed value.
            state: Vehicle state vector (index 3 is current velocity).
            params: Vehicle parameters dict.

        Returns:
            Acceleration in m/s^2, computed by a proportional controller.
        """
        if self.normalize:
            desired_speed = action * self.v_range + self.v_center
        else:
            desired_speed = action

        accl = p_accl(
            desired_speed,
            state[3],
            params["a_max"],
            params["v_max"],
            params["v_min"],
        )

        return accl


class SteerAction:
    """Base class for steering actions.

    Subclasses implement :meth:`act` to convert a raw action value into a
    steering velocity command suitable for the dynamics model.

    Attributes:
        normalize: Whether actions are normalized to ``[-1, 1]``.
        lower_limit: Lower bound of the action space.
        upper_limit: Upper bound of the action space.
        scale_factor: Multiplier applied when denormalizing actions.
    """

    def __init__(self, normalize: bool) -> None:
        self._type = None
        self.normalize = normalize
        self.lower_limit = None
        self.upper_limit = None
        self.scale_factor = 1.0

    @abstractmethod
    def act(self, steer_action: Any, **kwargs) -> float:
        raise NotImplementedError("steer act method not implemented")

    @property
    def type(self) -> str:
        """String identifier for this action type (e.g. ``"steering_angle"``)."""
        return self._type

    @property
    def space(self) -> gym.Space:
        """Gymnasium Box space for this action dimension."""
        return gym.spaces.Box(low=self.lower_limit, high=self.upper_limit, dtype=np.float32)


class SteeringAngleAction(SteerAction):
    """Target steering angle action.

    With ``params["T_steer"] > 0`` and a ``timestep``, applies an exact ZOH
    first-order lag ``δ_dot = (δ_ref − δ)/T_steer``; otherwise falls back to
    bang-bang. Normalized actions map ``[-1, 1]`` → ``[s_min, s_max]``.
    """

    def __init__(self, params: Dict, normalize: bool, timestep: float | None = None) -> None:
        super().__init__(normalize=normalize)
        self._type = "steering_angle"

        if normalize:
            self.lower_limit = -1.0
            self.upper_limit = 1.0
            # Guaranteed symmetric: s_min = -s_max
            self.scale_factor = params["s_max"]
        else:
            self.lower_limit = params["s_min"]
            self.upper_limit = params["s_max"]
            self.scale_factor = 1.0

        T_steer = params.get("T_steer", 0.0)
        if T_steer > 0.0 and timestep is not None and timestep > 0.0:
            # Rate gain such that sv = k·(δ_ref − δ) reproduces the exact ZOH
            # solution of δ_dot = (1/T_steer)(δ_ref − δ) over one sim step.
            self.k = (1.0 - np.exp(-timestep / T_steer)) / timestep
        else:
            self.k = None

    def act(self, action: float, state: np.ndarray, params: Dict) -> float:
        """Return steering velocity for a desired steering angle.

        Args:
            action: Normalized ``[-1, 1]`` or raw steering angle in radians.
            state: Vehicle state vector (index 2 is current steering angle).
            params: Vehicle parameters dict.

        Returns:
            Steering velocity in rad/s.
        """
        desired_angle = action * self.scale_factor

        if self.k is not None:
            sv = self.k * (desired_angle - state[2])
        else:
            sv = bang_bang_steer(
                desired_angle,
                state[2],
                params["sv_max"],
            )
        return sv


class SteeringSpeedAction(SteerAction):
    """Direct steering velocity action.

    When normalized, maps ``[-1, 1]`` to ``[sv_min, sv_max]`` (guaranteed symmetric).
    """

    def __init__(self, params: Dict, normalize: bool) -> None:
        super().__init__(normalize=normalize)
        self._type = "steering_speed"

        if normalize:
            self.lower_limit = -1.0
            self.upper_limit = 1.0
            # Guaranteed symmetric: sv_min = -sv_max
            self.scale_factor = params["sv_max"]
        else:
            self.lower_limit = params["sv_min"]
            self.upper_limit = params["sv_max"]
            self.scale_factor = 1.0

    def act(self, action: float, state: np.ndarray, params: Dict) -> float:
        """Return the steering velocity command.

        Args:
            action: Normalized ``[-1, 1]`` or raw steering velocity in rad/s.
            state: Vehicle state vector (unused for direct steering velocity).
            params: Vehicle parameters dict.

        Returns:
            Steering velocity in rad/s.
        """
        # When normalize=True: maps [-1, 1] → [sv_min, sv_max]
        # When normalize=False: pass-through (scale_factor=1.0)
        return action * self.scale_factor


class SteerActionEnum(Enum):
    """Enum for steering action types.

    Members:
        Steering_Angle: Target angle control (uses bang-bang controller).
        Steering_Speed: Direct steering velocity control.
    """

    Steering_Angle = 1
    Steering_Speed = 2

    @staticmethod
    def from_string(action: str):
        """Return the action class for the given string identifier.

        Args:
            action: One of ``"steering_angle"`` or ``"steering_speed"``.

        Returns:
            The corresponding action class.

        Raises:
            ValueError: If the action string is not recognised.
        """
        if action == "steering_angle":
            return SteeringAngleAction
        elif action == "steering_speed":
            return SteeringSpeedAction
        else:
            raise ValueError(f"Unknown action type {action}")

    @staticmethod
    def is_valid(action: str) -> bool:
        """Check whether a string is a valid steering action type."""
        return action in ("steering_angle", "steering_speed")


class CarAction:
    """Combined steering and longitudinal action for a single vehicle.

    Parses ``control_mode`` to select one steering action and one longitudinal
    action, then delegates to them in :meth:`act`.

    Attributes:
        normalize: Whether actions are normalized to ``[-1, 1]``.
    """

    def __init__(
        self, control_mode: str | list[str], params: Dict, normalize: bool, timestep: float | None = None
    ) -> None:
        long_act_type_fn = None
        steer_act_type_fn = None
        if type(control_mode) == str:  # only one control mode specified
            try:
                long_act_type_fn = LongitudinalActionEnum.from_string(control_mode)
            except ValueError:
                try:
                    steer_act_type_fn = SteerActionEnum.from_string(control_mode)
                except ValueError:
                    raise ValueError(f"Unknown control mode {control_mode}")
                if control_mode == "steering_speed":
                    warnings.warn(
                        f"Only one control mode specified, using {control_mode} for steering and defaulting to acceleration for longitudinal control"
                    )
                    long_act_type_fn = LongitudinalActionEnum.from_string("accl")
                else:
                    warnings.warn(
                        f"Only one control mode specified, using {control_mode} for steering and defaulting to speed for longitudinal control"
                    )
                    long_act_type_fn = LongitudinalActionEnum.from_string("speed")

            else:
                if control_mode == "accl":
                    warnings.warn(
                        f"Only one control mode specified, using {control_mode} for longitudinal control and defaulting to steering speed for steering"
                    )
                    steer_act_type_fn = SteerActionEnum.from_string("steering_speed")
                else:
                    warnings.warn(
                        f"Only one control mode specified, using {control_mode} for longitudinal control and defaulting to steering angle for steering"
                    )
                    steer_act_type_fn = SteerActionEnum.from_string("steering_angle")

        elif type(control_mode) == list:
            if len(control_mode) != 2:
                raise ValueError(f"control_input must have exactly 2 elements, got {len(control_mode)}")
            for mode in control_mode:
                if LongitudinalActionEnum.is_valid(mode):
                    if long_act_type_fn is not None:
                        raise ValueError(f"control_input has two longitudinal types: {control_mode}")
                    long_act_type_fn = LongitudinalActionEnum.from_string(mode)
                elif SteerActionEnum.is_valid(mode):
                    if steer_act_type_fn is not None:
                        raise ValueError(f"control_input has two steering types: {control_mode}")
                    steer_act_type_fn = SteerActionEnum.from_string(mode)
                else:
                    raise ValueError(
                        f"Unknown control mode '{mode}'. Valid: 'accl', 'speed', 'steering_angle', 'steering_speed'"
                    )
            if long_act_type_fn is None:
                raise ValueError("control_input must include a longitudinal type ('accl' or 'speed')")
            if steer_act_type_fn is None:
                raise ValueError("control_input must include a steering type ('steering_angle' or 'steering_speed')")
        else:
            raise ValueError(f"Unknown control mode {control_mode}")

        # Store normalize parameter for reference
        self.normalize = normalize

        # Pass normalize parameter to action instances
        self._longitudinal_action: LongitudinalAction = long_act_type_fn(params, normalize=normalize)
        if steer_act_type_fn is SteeringAngleAction:
            self._steer_action: SteerAction = steer_act_type_fn(params, normalize=normalize, timestep=timestep)
        else:
            self._steer_action: SteerAction = steer_act_type_fn(params, normalize=normalize)

    def act(self, action: Any, **kwargs) -> Tuple[float, float]:
        """Convert a ``[steer, longitudinal]`` action pair into control commands.

        Args:
            action: Two-element array ``[steer_action, longitudinal_action]``.
            **kwargs: Forwarded to the underlying action handlers (``state``, ``params``).

        Returns:
            Tuple of ``(acceleration, steering_velocity)``.
        """
        steer_action = self._steer_action.act(action[0], **kwargs)
        longitudinal_action = self._longitudinal_action.act(action[1], **kwargs)
        return longitudinal_action, steer_action

    @property
    def type(self) -> Tuple[str, str]:
        """Tuple of ``(steering_type, longitudinal_type)`` string identifiers."""
        return (self._steer_action.type, self._longitudinal_action.type)

    @property
    def steer_bounds(self) -> Tuple[float, float]:
        """``(lower, upper)`` bounds for the steering action dimension."""
        return (self._steer_action.lower_limit, self._steer_action.upper_limit)

    @property
    def throttle_bounds(self) -> Tuple[float, float]:
        """``(lower, upper)`` bounds for the longitudinal action dimension."""
        return (self._longitudinal_action.lower_limit, self._longitudinal_action.upper_limit)

    @property
    def space(self) -> gym.Space:
        """Combined 2-D Gymnasium Box space ``[steer, longitudinal]``."""
        low = np.array([self._steer_action.lower_limit, self._longitudinal_action.lower_limit]).astype(np.float32)
        high = np.array([self._steer_action.upper_limit, self._longitudinal_action.upper_limit]).astype(np.float32)

        return gym.spaces.Box(low=low, high=high, shape=(2,), dtype=np.float32)


def from_single_to_multi_action_space(single_agent_action_space: gym.spaces.Box, num_agents: int) -> gym.spaces.Box:
    """Tile a single-agent action space into a multi-agent action space.

    Args:
        single_agent_action_space: A 1-D Box space for one agent.
        num_agents: Number of agents.

    Returns:
        A Box space of shape ``(num_agents, action_dim)``.
    """
    return gym.spaces.Box(
        low=single_agent_action_space.low[None].repeat(num_agents, 0),
        high=single_agent_action_space.high[None].repeat(num_agents, 0),
        shape=(num_agents, single_agent_action_space.shape[0]),
        dtype=np.float32,
    )
