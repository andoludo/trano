"""Radau collocation of a CasADi model over a horizon of steps, shared by optimal control and estimation."""

from collections.abc import Callable
from functools import cache
from typing import Any

import casadi as ca
import numpy as np
from pydantic import BaseModel, ConfigDict, Field

from .model import CasadiModel
from .roles import Roles

KELVIN = 273.15
IpoptOption = float | int | str
SOLVED = ("Solve_Succeeded", "Solved_To_Acceptable_Level")


@cache
def linear_solver() -> str:
    """SPRAL when the IPOPT of CasADi has it, else MUMPS.

    On building models with water circuits SPRAL converges in far fewer iterations than MUMPS.
    """
    x = ca.SX.sym("x")
    options: dict[str, str | int | bool] = {
        "ipopt.linear_solver": "spral",
        "ipopt.print_level": 0,
        "ipopt.sb": "yes",
        "print_time": False,
    }
    try:
        solver = ca.nlpsol("probe", "ipopt", {"x": x, "f": (x - 1) ** 2}, options)
        solver(x0=0)
    except RuntimeError:
        return "mumps"
    return "spral" if solver.stats()["return_status"] == "Solve_Succeeded" else "mumps"


def default_ipopt_options() -> dict[str, IpoptOption]:
    """IPOPT options. Redundant controls (several ways to deliver the same heat) leave flat valleys at the optimum:
    an acceptable level stops the crawl along them."""
    return {
        "print_level": 0,
        "sb": "yes",
        "max_iter": 1000,
        "tol": 1e-6,
        "acceptable_tol": 1e-5,
        "acceptable_iter": 10,
        "linear_solver": linear_solver(),
    }


class CollocationSettings(BaseModel):
    """Discretization of the model and solver options."""

    model_config = ConfigDict(frozen=True)

    time_step: float = Field(900.0, gt=0, description="Length of a step [s]")
    degree: int = Field(3, ge=1, le=9, description="Radau collocation points per step")
    default_heat_bound: float = Field(1e4, gt=0, description="Upper bound of a heat input [W] without a max")
    temperature_bounds: tuple[float, float] = Field(
        (KELVIN - 30, KELVIN + 100), description="Bounds of every temperature state [K]"
    )
    ipopt: dict[str, IpoptOption] = Field(
        default_factory=dict, description="IPOPT options added to (or replacing) default_ipopt_options()"
    )

    def ipopt_options(self) -> dict[str, IpoptOption]:
        return {**default_ipopt_options(), **self.ipopt}


def collocation_matrices(degree: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Radau IIA points ``tau`` (with 0), ``C[j, r]`` (derivative of the Lagrange basis j at point r), ``D[j]``."""
    tau = np.append(0.0, ca.collocation_points(degree, "radau"))
    c = np.zeros((degree + 1, degree + 1))
    d = np.zeros(degree + 1)
    for j in range(degree + 1):
        basis = np.poly1d([1.0])
        for r in range(degree + 1):
            if r != j:
                basis *= np.poly1d([1.0, -tau[r]]) / (tau[j] - tau[r])
        d[j] = basis(1.0)
        derivative = np.polyder(basis)
        c[j] = [derivative(point) for point in tau]
    return tau, c, d


class Collocated:
    """A model collocated on Radau points, with scaled controls and a CVODES simulator.

    The states are shifted (temperatures in degC) and the controls scaled to [0, 1] so that IPOPT sees
    well-conditioned variables.
    """

    def __init__(
        self,
        model: CasadiModel,
        roles: Roles | None = None,
        settings: CollocationSettings | None = None,
        parameters: np.ndarray | None = None,
    ) -> None:
        description = model.description
        self.model = model
        self.description = description
        self.roles = roles or Roles.infer(description)
        self.roles.check(description)
        self.settings = settings or CollocationSettings()
        self.parameters = description.parameter_values if parameters is None else np.asarray(parameters, dtype=float)
        self.control_index = [description.input_names.index(name) for name in self.roles.controls]
        self.disturbance_index = [description.input_names.index(name) for name in self.roles.disturbances]
        self.comfort_index = [description.state_names.index(name) for name in self.roles.comfort]
        self.lower, self.upper = np.array([self._bounds(name) for name in self.roles.controls]).reshape(-1, 2).T
        self.offset = np.array([KELVIN if state.unit == "K" else 0.0 for state in description.states])
        self._interval = self._interval_function()
        self._step = self._step_function()

    @property
    def n_states(self) -> int:
        return len(self.description.states)

    def _bounds(self, name: str) -> tuple[float, float]:
        variable = self.description.input(name)
        lower = variable.minimum if variable.minimum is not None else 0.0
        upper = variable.maximum if variable.maximum is not None else self.settings.default_heat_bound
        return lower, upper

    def _inputs(self, u: ca.MX, d: ca.MX) -> ca.MX:
        """The input vector of the model, controls and disturbances at their places."""
        signals = {index: u[position] for position, index in enumerate(self.control_index)}
        signals |= {index: d[position] for position, index in enumerate(self.disturbance_index)}
        return ca.vertcat(*(signals[index] for index in range(len(self.description.inputs))))

    def _interval_function(self) -> ca.Function:
        """Collocation residuals and end state of one step starting at ``t0``, in scaled variables."""
        n_x, degree, step = self.n_states, self.settings.degree, self.settings.time_step
        tau, c, d = collocation_matrices(degree)
        parameters, offset = ca.DM(self.parameters), ca.DM(self.offset)
        t0 = ca.MX.sym("t0")
        z0 = ca.MX.sym("z0", n_x)
        flat = ca.MX.sym("zc", n_x * degree)  # the collocation points of the step, stacked
        zc = ca.reshape(flat, n_x, degree)
        v = ca.MX.sym("v", len(self.roles.controls))
        w = ca.MX.sym("w", len(self.roles.disturbances))
        u = ca.DM(self.lower) + ca.DM(self.upper - self.lower) * v
        points = [z0] + [zc[:, j] for j in range(degree)]
        residuals = []
        for j in range(1, degree + 1):
            slope = sum(c[r, j] * points[r] for r in range(degree + 1))
            derivative = self.model.rhs(t0 + tau[j] * step, points[j] + offset, self._inputs(u, w), parameters)
            residuals.append(step * derivative - slope)
        end = sum(d[r] * points[r] for r in range(degree + 1))
        return ca.Function("interval", [t0, z0, flat, v, w], [ca.vertcat(*residuals), end])

    def _step_function(self) -> ca.Function:
        """One step of the model with CVODES, controls and disturbances held, from the time ``t0``."""
        x = ca.MX.sym("x", self.n_states)
        u = ca.MX.sym("u", len(self.roles.controls))
        w = ca.MX.sym("w", len(self.roles.disturbances))
        t0 = ca.MX.sym("t0")
        s = ca.MX.sym("s")  # time since the start of the step
        ode = {
            "x": x,
            "t": s,
            "p": ca.vertcat(t0, u, w),
            "ode": self.model.rhs(t0 + s, x, self._inputs(u, w), ca.DM(self.parameters)),
        }
        options = {"abstol": 1e-6, "reltol": 1e-6}
        return ca.integrator("step", "cvodes", ode, 0, self.settings.time_step, options)

    def simulate(
        self, initial_state: np.ndarray, controls: np.ndarray, disturbances: np.ndarray, start_time: float = 0.0
    ) -> np.ndarray:
        """States at the step boundaries, (states, N + 1), for controls (controls, N) held over each step."""
        states = [np.asarray(initial_state, dtype=float)]
        for k in range(controls.shape[1]):
            t0 = start_time + k * self.settings.time_step
            result = self._step(x0=states[-1], p=np.concatenate([[t0], controls[:, k], disturbances[:, k]]))
            states.append(np.asarray(result["xf"]).ravel())
        return np.column_stack(states)

    def steps(
        self, z: ca.MX, zc: ca.MX, v: ca.MX | ca.DM, disturbances: np.ndarray, start_time: float
    ) -> tuple[ca.MX, ca.MX]:
        """Collocation residuals and end states of every step.

        The steps are called one by one and IPOPT expands the problem to SX (``expand``): IPOPT then converges with
        the exact Hessian on these nonconvex problems (a ``map`` of the step made it crawl).
        """
        residuals, ends = [], []
        for k in range(disturbances.shape[1]):
            t0 = start_time + k * self.settings.time_step
            residual, end = self._interval(t0, z[:, k], zc[:, k], v[:, k], ca.DM(disturbances[:, k]))
            residuals.append(residual)
            ends.append(end)
        return ca.horzcat(*residuals), ca.horzcat(*ends)

    def bound_temperatures(self, opti: ca.Opti, z: ca.MX) -> None:
        low, high = self.settings.temperature_bounds
        temperatures = [i for i, state in enumerate(self.description.states) if state.unit == "K"]
        if temperatures:
            opti.subject_to(opti.bounded(low - KELVIN, ca.vec(z[temperatures, :]), high - KELVIN))

    def solve_nlp(self, opti: ca.Opti) -> tuple[Callable[..., Any], str]:
        """Solve with IPOPT; when SPRAL fails (it does on some cold starts), solve again with MUMPS."""
        options = self.settings.ipopt_options()
        attempts = [options]
        if options.get("linear_solver") == "spral" and "linear_solver" not in self.settings.ipopt:
            attempts.append({**options, "linear_solver": "mumps"})
        values: Callable[..., Any] = opti.debug.value
        status = "Not_Solved"
        for attempt in attempts:
            opti.solver("ipopt", {"print_time": False, "expand": True}, attempt)
            try:
                values, status = opti.solve().value, opti.stats()["return_status"]
            except RuntimeError:
                values, status = opti.debug.value, opti.stats()["return_status"]
            if status in SOLVED:
                break
        return values, status

    def scaled(self, controls: np.ndarray) -> np.ndarray:
        """Controls on [0, 1]; a control whose bounds are equal is fixed (0)."""
        span = np.where(self.upper > self.lower, self.upper - self.lower, 1.0)
        return np.asarray((controls - self.lower[:, None]) / span[:, None])

    def unscaled(self, scaled: np.ndarray) -> np.ndarray:
        return np.asarray(self.lower[:, None] + (self.upper - self.lower)[:, None] * scaled)
