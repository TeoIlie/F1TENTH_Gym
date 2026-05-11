"""Dynamic models available in the Gym-Khana environment.

Each submodule contains a single model. Most models originate from the
CommonRoad vehicle model library: https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/

Vehicle Parameters
------------------
All dynamics functions accept a ``params`` dict. The keys used across models are:

**Geometry**

- ``lf`` (float): Distance from centre of gravity to front axle (m).
- ``lr`` (float): Distance from centre of gravity to rear axle (m).
- ``h`` (float): Height of centre of gravity (m).
- ``length`` (float): Vehicle length (m).
- ``width`` (float): Vehicle width (m).

**Inertia**

- ``m`` (float): Vehicle mass (kg).
- ``I`` (float): Moment of inertia about the Z axis (kg·m²). *(KS, ST)*
- ``I_z`` (float): Yaw moment of inertia (kg·m²). *(STD)*
- ``I_y_w`` (float): Wheel moment of inertia about spin axis (kg·m²). *(STD)*

**Tyre / lateral dynamics**

- ``mu`` (float): Tyre–road friction coefficient. *(ST, STP)*
- ``C_Sf`` (float): Cornering stiffness of front tyres (N/rad). *(ST)*
- ``C_Sr`` (float): Cornering stiffness of rear tyres (N/rad). *(ST)*
- ``B_f, C_f, D_f, E_f`` (float): Front-axle Pacejka Magic Formula coefficients. *(STP)*
- ``B_r, C_r, D_r, E_r`` (float): Rear-axle Pacejka Magic Formula coefficients. *(STP)*

**Steering constraints**

- ``s_min`` (float): Minimum steering angle (rad).
- ``s_max`` (float): Maximum steering angle (rad).
- ``sv_min`` (float): Minimum steering velocity (rad/s).
- ``sv_max`` (float): Maximum steering velocity (rad/s).

**Longitudinal constraints**

- ``v_switch`` (float): Velocity above which peak acceleration begins to scale down (m/s).
- ``a_max`` (float): Maximum acceleration magnitude (m/s²).
- ``v_min`` (float): Minimum allowed velocity (m/s).
- ``v_max`` (float): Maximum allowed velocity (m/s).

**STD-only (PAC2002 tyre model)**

- ``h_s`` (float): Height of sprung mass centre of gravity (m).
- ``R_w`` (float): Wheel radius (m).
- ``T_sb`` (float): Brake torque split to front axle (0–1).
- ``T_se`` (float): Engine torque split to front axle (0–1).
- Plus all PAC2002 magic-formula coefficients (``B_f``, ``C_f``, ``D_f``, etc.)
  — see ``gymkhana.envs.gymkhana_env.GKEnv.f1tenth_std_vehicle_params()``.
"""

import warnings
from enum import Enum
from typing import Optional

import numpy as np

from .kinematic import get_standardized_state_ks, vehicle_dynamics_ks
from .multi_body import get_standardized_state_mb, init_mb, vehicle_dynamics_mb
from .single_track import get_standardized_state_st, vehicle_dynamics_st
from .single_track_drift import get_standardized_state_std, init_std, vehicle_dynamics_std
from .single_track_pacejka import vehicle_dynamics_stp
from .utils import bang_bang_steer, p_accl


class DynamicModel(Enum):
    """Available vehicle dynamics models, ordered by increasing complexity.

    Members:
        KS: Kinematic Single Track — pure kinematics, no tire forces.
        ST: Single Track — lateral dynamics without an explicit tire model.
        MB: Multi-Body — full tire modeling and multi-body dynamics.
        STD: Single Track Drift — ST with PAC2002 tire model for drift simulation.
        STP: Single Track Pacejka — ST with lateral-only Pacejka Magic Formula.
    """

    KS = 1  # Kinematic Single Track
    ST = 2  # Single Track
    MB = 3  # Multi-body Model
    STD = 4  # Single Track Drift
    STP = 5  # Single Track Pacejka (lateral-only)

    @staticmethod
    def from_string(model: str):
        """Return the DynamicModel enum value for a string identifier.

        Args:
            model: One of ``"ks"``, ``"st"``, ``"mb"``, ``"std"``, ``"stp"``.

        Returns:
            Corresponding :class:`DynamicModel` enum value.

        Raises:
            ValueError: If the model string is not recognised.
        """
        if model == "ks":
            warnings.warn("Chosen model is KS. This is different from previous versions of the gym.")
            return DynamicModel.KS
        elif model == "st":
            return DynamicModel.ST
        elif model == "mb":
            return DynamicModel.MB
        elif model == "std":
            return DynamicModel.STD
        elif model == "stp":
            return DynamicModel.STP
        else:
            raise ValueError(f"Unknown model type {model}")

    def user_state_lens(self) -> tuple:
        """Return the user-facing state-row widths accepted by full-state reset.

        - KS  -> ``(5,)``  ``[x, y, delta, v, yaw]``
        - ST  -> ``(7,)``  ``[x, y, delta, v, yaw, yaw_rate, slip_angle]``
        - STP -> ``(7,)``  (same layout as ST)
        - STD -> ``(7, 9)``: 7-wide is the same layout as ST with ``omega_f, omega_r``
          derived from no-slip rolling inside ``init_std``; 9-wide additionally
          accepts ``[..., omega_f, omega_r]`` (front and rear wheel angular
          velocities, rad/s) taken as given. The caller owns consistency
          between ``v`` and the wheel speeds.

        Returns:
            Tuple of accepted state-row widths for this model.

        Raises:
            ValueError: For MB, which has a 29D internal state with
                suspension/wheel quantities a user cannot sensibly construct.
        """
        if self == DynamicModel.MB:
            raise ValueError(
                "Full-state initialization is not supported for the MB model. Use pose-based reset instead."
            )
        return {
            DynamicModel.KS: (5,),
            DynamicModel.ST: (7,),
            DynamicModel.STP: (7,),
            DynamicModel.STD: (7, 9),
        }[self]

    def get_initial_state(self, pose=None, state=None, params: Optional[dict] = None):
        """
        Get the initial state for the vehicle model.

        Args:
            pose: (3,) array [x, y, yaw] - legacy pose-only initialization.
            state: Optional model-specific user-facing state row; see
                :meth:`user_state_lens` for accepted widths and layouts. MB
                does not support full-state initialization.
            params: Vehicle parameters dictionary (required for MB and STD models)

        Returns:
            Full state vector for the model

        Note:
            Cannot specify both pose and state - use one or the other.
        """
        # Validation: pose and state are mutually exclusive
        if pose is not None and state is not None:
            raise ValueError("Cannot provide both 'pose' and 'state'. Use one or the other.")

        # Assert that if self is MB or STD, params is not None
        if (self == DynamicModel.MB or self == DynamicModel.STD) and params is None:
            raise ValueError("MultiBody and SingleTrackDrift models require parameters to be provided.")

        # initialize zero state
        if self == DynamicModel.KS:
            # state is [x, y, steer_angle, vel, yaw_angle]
            init_state = np.zeros(5)
        elif self == DynamicModel.ST or self == DynamicModel.STP:
            # state is [x, y, steer_angle, vel, yaw_angle, yaw_rate, slip_angle]
            init_state = np.zeros(7)
        elif self == DynamicModel.MB:
            # state is a 29D vector
            init_state = np.zeros(29)
        elif self == DynamicModel.STD:
            # state is [x, y, steer_angle, vel, yaw_angle, yaw_rate, slip_angle, omega_f, omega_r]
            init_state = np.zeros(9)
        else:
            raise ValueError(f"Unknown model type {self}")

        # If full state provided, dispatch per model. Accepted widths and
        # MB-rejection both come from DynamicModel.user_state_lens so the
        # rules live in one place.
        user_state_len_actual = None
        if state is not None:
            accepted_lens = self.user_state_lens()
            user_state_len_actual = len(state)
            if user_state_len_actual not in accepted_lens:
                raise ValueError(
                    f"{self.name} model requires state of length in {accepted_lens}, got {user_state_len_actual}"
                )
            init_state[:user_state_len_actual] = state
        # set initial pose if provided
        elif pose is not None:
            init_state[0:2] = pose[0:2]
            init_state[4] = pose[2]

        # If state is MultiBody, we must inflate the state to 29D
        if self == DynamicModel.MB:
            init_state = init_mb(init_state, params)
        # If state is SingleTrackDrift, we must inflate to 9D. When the user
        # supplied an explicit 9-wide row, omega_f/omega_r are taken as given;
        # otherwise they're derived from the no-slip rolling assumption.
        elif self == DynamicModel.STD:
            compute_wheel_speeds = user_state_len_actual != 9
            init_state = init_std(init_state, params, compute_wheel_speeds=compute_wheel_speeds)
        return init_state

    @property
    def f_dynamics(self):
        """The dynamics function ``f(x, u, params)`` for this model."""
        if self == DynamicModel.KS:
            return vehicle_dynamics_ks
        elif self == DynamicModel.ST:
            return vehicle_dynamics_st
        elif self == DynamicModel.MB:
            return vehicle_dynamics_mb
        elif self == DynamicModel.STD:
            return vehicle_dynamics_std
        elif self == DynamicModel.STP:
            return vehicle_dynamics_stp
        else:
            raise ValueError(f"Unknown model type {self}")

    def get_standardized_state_fn(self):
        """
        This function returns the standardized state information for the model.
        This needs to be a function, because the state information is different for each model.
        Slip is not directly available from the MB model.
        """
        if self == DynamicModel.KS:
            return get_standardized_state_ks
        elif self == DynamicModel.ST:
            return get_standardized_state_st
        elif self == DynamicModel.MB:
            return get_standardized_state_mb
        elif self == DynamicModel.STD:
            return get_standardized_state_std
        elif self == DynamicModel.STP:
            return get_standardized_state_st
        else:
            raise ValueError(f"Unknown model type {self}")
