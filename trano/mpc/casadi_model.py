"""Symbolic CasADi representation of the generated RC Modelica models.

The Modelica source is translated with `rumoca <https://github.com/rumoca/rumoca>`_, a
Modelica compiler exporting explicit ODEs as CasADi functions ``xdot = rhs(t, x, u, p)``.
The model parameters stay symbolic, so the same functions can be used for MPC (fixed
parameters) and for parameter identification (free parameters).
"""

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Literal

import casadi as ca
import numpy as np
import numpy.typing as npt

if TYPE_CHECKING:
    from trano.mpc.building import RCBuilding

HEATING_INPUT = "QHea"
# Real-axis stability limit of the explicit Runge-Kutta 4 method (~2.785) with a safety margin.
RK4_STABILITY_LIMIT = 2.5

FloatArray = npt.NDArray[np.float64]


def is_control_input(name: str) -> bool:
    """Heating inputs are named ``QHea`` (library models) or ``<zone>_QHea`` (building models)."""
    return name == HEATING_INPUT or name.endswith(f"_{HEATING_INPUT}")


class ModelicaTranslationError(RuntimeError):
    """Raised when a Modelica model cannot be translated into CasADi."""


def _compile(source: str, model: str) -> Any:  # noqa: ANN401 - rumoca is untyped
    try:
        import rumoca
    except ImportError as error:  # pragma: no cover - depends on the installed extras
        raise ImportError(
            "Translating Modelica into CasADi requires 'rumoca'. Install it with: pip install 'trano[mpc]'"
        ) from error
    try:
        return rumoca.Session().loads(source, model=model).to_casadi()
    except Exception as error:
        raise ModelicaTranslationError(f"Could not translate {model} into CasADi: {error}") from error


@dataclass(frozen=True)
class CasadiRCModel:
    """Explicit ODE ``xdot = rhs(t, x, u, p)`` of an RC building model.

    ``u`` gathers all the Modelica inputs in declaration order. :meth:`continuous_dynamics`
    and :meth:`discrete_dynamics` split them into control inputs (the heating powers
    ``QHea``/``<zone>_QHea``) and disturbances (weather and internal gains).
    """

    rhs: ca.Function
    state_names: tuple[str, ...]
    input_names: tuple[str, ...]
    parameter_names: tuple[str, ...]
    initial_state: FloatArray
    default_parameters: FloatArray
    building: "RCBuilding | None" = field(default=None, compare=False, repr=False)

    @classmethod
    def from_modelica(cls, source: str, model: str, building: "RCBuilding | None" = None) -> "CasadiRCModel":
        export = _compile(source, model)
        return cls(
            rhs=export.rhs,
            state_names=tuple(export.state_names),
            input_names=tuple(export.input_names),
            parameter_names=tuple(export.parameter_names),
            initial_state=np.asarray(export.default_states, dtype=np.float64),
            default_parameters=np.asarray(export.default_parameters, dtype=np.float64),
            building=building,
        )

    @property
    def control_names(self) -> tuple[str, ...]:
        return tuple(name for name in self.input_names if is_control_input(name))

    @property
    def disturbance_names(self) -> tuple[str, ...]:
        return tuple(name for name in self.input_names if not is_control_input(name))

    @property
    def zone_prefixes(self) -> tuple[str, ...]:
        """Variable prefix of each controlled zone: ``""`` for a library model, ``"<zone>_"`` otherwise."""
        return tuple(name.removesuffix(HEATING_INPUT) for name in self.control_names)

    @property
    def n_states(self) -> int:
        return len(self.state_names)

    @property
    def n_controls(self) -> int:
        return len(self.control_names)

    @property
    def n_disturbances(self) -> int:
        return len(self.disturbance_names)

    @property
    def n_parameters(self) -> int:
        return len(self.parameter_names)

    def state_index(self, name: str) -> int:
        return self.state_names.index(name)

    def parameter_values(self, **overrides: float) -> FloatArray:
        """Default parameter vector, optionally overriding some parameters by name."""
        values = self.default_parameters.copy()
        for name, value in overrides.items():
            values[self.parameter_names.index(name)] = value
        return values

    def continuous_dynamics(self) -> ca.Function:
        """``xdot = f(x, u, d, p)`` with controls ``u`` and disturbances ``d``."""
        x = ca.SX.sym("x", self.n_states)
        u = ca.SX.sym("u", self.n_controls)
        d = ca.SX.sym("d", self.n_disturbances)
        p = ca.SX.sym("p", self.n_parameters)
        signals = {
            **{name: u[i] for i, name in enumerate(self.control_names)},
            **{name: d[i] for i, name in enumerate(self.disturbance_names)},
        }
        inputs = ca.vertcat(*[signals[name] for name in self.input_names])
        xdot = self.rhs(0, x, inputs, p)
        return ca.Function("f", [x, u, d, p], [xdot], ["x", "u", "d", "p"], ["xdot"])

    def fastest_time_constant(self, parameters: FloatArray | None = None) -> float:
        """Smallest time constant [s] of the (linear) model, from the eigenvalues of df/dx."""
        f = self.continuous_dynamics()
        x = ca.SX.sym("x", self.n_states)
        jacobian = ca.Function("A", [x], [ca.jacobian(f(x, 0, 0, self._parameters(parameters)), x)])
        eigenvalues = np.linalg.eigvals(np.array(jacobian(self.initial_state)))
        return float(1 / np.max(np.abs(eigenvalues)))

    def stable_substeps(self, time_step: float, parameters: FloatArray | None = None) -> int:
        """Number of RK4 sub-steps keeping the explicit integration stable over ``time_step``."""
        return max(1, math.ceil(time_step / (RK4_STABILITY_LIMIT * self.fastest_time_constant(parameters))))

    def discrete_dynamics(
        self,
        time_step: float,
        substeps: int | None = None,
        method: Literal["rk4", "euler"] = "rk4",
    ) -> ca.Function:
        """``x[k+1] = F(x[k], u[k], d[k], p)`` with inputs held constant over ``time_step``."""
        substeps = substeps or self.stable_substeps(time_step)
        f = self.continuous_dynamics()
        x = ca.MX.sym("x", self.n_states)
        u = ca.MX.sym("u", self.n_controls)
        d = ca.MX.sym("d", self.n_disturbances)
        p = ca.MX.sym("p", self.n_parameters)
        h = time_step / substeps
        x_next = x
        for _ in range(substeps):
            if method == "euler":
                x_next = x_next + h * f(x_next, u, d, p)
                continue
            k1 = f(x_next, u, d, p)
            k2 = f(x_next + h / 2 * k1, u, d, p)
            k3 = f(x_next + h / 2 * k2, u, d, p)
            k4 = f(x_next + h * k3, u, d, p)
            x_next = x_next + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return ca.Function("F", [x, u, d, p], [x_next], ["x", "u", "d", "p"], ["x_next"])

    def simulate(
        self,
        controls: npt.ArrayLike,
        disturbances: npt.ArrayLike,
        time_step: float,
        initial_state: npt.ArrayLike | None = None,
        parameters: FloatArray | None = None,
    ) -> FloatArray:
        """Simulate ``N`` steps; ``controls`` is ``(N, n_controls)``, ``disturbances`` is ``(N, n_disturbances)``.

        Returns the ``(N + 1, n_states)`` state trajectory.
        """
        controls_ = np.atleast_2d(np.asarray(controls, dtype=np.float64)).reshape(-1, self.n_controls)
        disturbances_ = np.atleast_2d(np.asarray(disturbances, dtype=np.float64)).reshape(-1, self.n_disturbances)
        if controls_.shape[0] != disturbances_.shape[0]:
            raise ValueError("controls and disturbances must have the same number of time steps.")
        steps = controls_.shape[0]
        x0 = self.initial_state if initial_state is None else np.asarray(initial_state, dtype=np.float64)
        trajectory = self.discrete_dynamics(time_step).mapaccum("simulation", steps)
        p = np.tile(self._parameters(parameters).reshape(-1, 1), (1, steps))
        states = np.array(trajectory(x0, controls_.T, disturbances_.T, p))
        return np.vstack([x0, states.T])

    def _parameters(self, parameters: FloatArray | None) -> FloatArray:
        return self.default_parameters if parameters is None else np.asarray(parameters, dtype=np.float64)
