"""ODE integrators for vehicle dynamics.

The ``_*_jit`` steps are jitted and take a :class:`VehicleParams` NamedTuple;
the ``_*_py`` siblings are the Python fallback used by the MB model (dict).
Dispatcher methods pick the path via ``isinstance(params, VehicleParams)``.
"""

from abc import abstractmethod
from enum import Enum

from numba import njit

from .params import VehicleParams


class IntegratorType(Enum):
    RK4 = 1
    Euler = 2

    @staticmethod
    def from_string(integrator: str):
        if integrator == "rk4":
            return RK4Integrator()
        elif integrator == "euler":
            return EulerIntegrator()
        else:
            raise ValueError(f"Unknown integrator type {integrator}")


class Integrator:
    def __init__(self) -> None:
        self._integrator_type = None

    @abstractmethod
    def integrate(self, f, x, u, dt, params):
        raise NotImplementedError("integrate method not implemented")

    @property
    def type(self) -> str:
        return self._integrator_type


@njit(cache=True)
def _rk4_step_jit(f, x, u, dt, params):
    k1 = f(x, u, params)
    k2 = f(x + dt * (k1 / 2), u, params)
    k3 = f(x + dt * (k2 / 2), u, params)
    k4 = f(x + dt * k3, u, params)
    return x + dt * (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def _rk4_step_py(f, x, u, dt, params):
    k1 = f(x, u, params)
    k2 = f(x + dt * (k1 / 2), u, params)
    k3 = f(x + dt * (k2 / 2), u, params)
    k4 = f(x + dt * k3, u, params)
    return x + dt * (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


@njit(cache=True)
def _euler_step_jit(f, x, u, dt, params):
    return x + dt * f(x, u, params)


def _euler_step_py(f, x, u, dt, params):
    return x + dt * f(x, u, params)


class RK4Integrator(Integrator):
    def __init__(self) -> None:
        super().__init__()
        self._integrator_type = "rk4"

    def integrate(self, f, x, u, dt, params):
        # VehicleParams routes to the jitted kernel; dict (MB) falls back to Python.
        if isinstance(params, VehicleParams):
            return _rk4_step_jit(f, x, u, dt, params)
        return _rk4_step_py(f, x, u, dt, params)


class EulerIntegrator(Integrator):
    def __init__(self) -> None:
        super().__init__()
        self._integrator_type = "euler"

    def integrate(self, f, x, u, dt, params):
        # VehicleParams routes to the jitted kernel; dict (MB) falls back to Python.
        if isinstance(params, VehicleParams):
            return _euler_step_jit(f, x, u, dt, params)
        return _euler_step_py(f, x, u, dt, params)
