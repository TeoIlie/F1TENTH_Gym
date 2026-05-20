"""Single Track Pacejka (STP) vehicle dynamics model.

Lateral-only Pacejka Magic Formula tire model on top of a dynamic single-track
chassis. State shape matches ST (7 elements) — no longitudinal slip or
wheel-spin states. Ported from f110-simulator's STDKinematics::update_pacejka.
"""

from .single_track_pacejka import vehicle_dynamics_stp
