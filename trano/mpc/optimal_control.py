"""Optimal control of a ``building_mpc`` model with IPOPT: direct collocation of its CasADi ODE.

The model is the CasADi export of ``building_mpc`` (``xdot = rhs(t, x, u, p)``) and its interface. The problem
over a horizon of ``N`` steps is

    minimize    sum_k price_k * P(u_k) * dt  +  comfort_weight * sum(slack^2 + slack)
                +  move_weight * sum(du^2)  +  effort_weight * sum(u^2)      (u scaled to [0, 1])
    subject to  the ODE, collocated on Radau IIA points in each step (stable for the stiff water nodes of the
                hydronic circuits, whatever the step),
                lower_k - slack_k <= indoor temperature of each zone,   slack >= 0,
                the bounds of each control (valve flows, valve positions, boiler part load ratios),

where ``P(u)*dt`` is the energy drawn over a step: the increase of the fuel and electricity meters of the systems
(boilers, heat pumps: states in kWh, so that a nonlinear power is integrated exactly) and the heat of each ideal
zone heating input. The controls and states are scaled (controls to their
range, temperatures in degC) so that IPOPT sees well-conditioned variables.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from functools import cache
from collections.abc import Callable
from typing import TYPE_CHECKING, Any

import casadi as ca
import numpy as np
from pydantic import BaseModel, ConfigDict, Field

if TYPE_CHECKING:
    from trano.mpc.interface import InputSpec, MPCModelInterface

KELVIN = 273.15
IpoptOption = float | int | str


SOLVED = ("Solve_Succeeded", "Solved_To_Acceptable_Level")
FUEL_METER = "EFue"  # boilers; every other kWh state is an electricity meter but the production of PV arrays
PRODUCTION_METER = "EPro"
NET_SMOOTHING = 1e-3  # [kWh] of the positive part of the net electricity


@cache
def linear_solver() -> str:
    """SPRAL when the IPOPT of CasADi has it, else MUMPS.

    On the hydronic models SPRAL converges in far fewer iterations (24 hourly steps of the six-zone heat pump house:
    35 iterations, against no convergence in 1000 with MUMPS).
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


def _default_ipopt() -> dict[str, IpoptOption]:
    """IPOPT options. The heating controls are redundant (branch flows, mixing, production all deliver heat), so that
    the last digits of the optimum lie along flat valleys: an acceptable level stops the crawl along them."""
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
    """Discretization of the model and solver options, shared by control and estimation."""

    model_config = ConfigDict(frozen=True)

    time_step: float = Field(900.0, gt=0, description="Length of a step [s]")
    degree: int = Field(3, ge=1, le=9, description="Radau collocation points per step")
    default_heat_bound: float = Field(1e4, gt=0, description="Upper bound of a heat input without one [W]")
    temperature_bounds: tuple[float, float] = Field(
        (KELVIN - 30, KELVIN + 100), description="Bounds of every temperature state (air, walls, water) [K]"
    )
    ipopt: dict[str, IpoptOption] = Field(
        default_factory=dict, description="IPOPT options, added to (or replacing) the defaults of _default_ipopt"
    )

    def ipopt_options(self) -> dict[str, IpoptOption]:
        return {**_default_ipopt(), **self.ipopt}


class OptimalControlSettings(CollocationSettings):
    comfort_weight: float = Field(1e3, ge=0, description="Cost of one kelvin-hour of discomfort")
    move_weight: float = Field(1e-2, ge=0, description="Cost of a full-range change of a control between steps")
    effort_weight: float = Field(
        1e-3,
        ge=0,
        description="Cost of a control at full range per step: separates the controls that deliver the same heat "
        "(boiler temperature, mixing, branch flow), whose flat valley IPOPT would otherwise crawl along",
    )


class EstimationSettings(CollocationSettings):
    measurement_noise: float = Field(0.1, gt=0, description="Standard deviation of a measurement [K]")
    process_noise: float = Field(0.05, gt=0, description="Standard deviation of the model error over a step [K]")
    arrival_noise: float = Field(2.0, gt=0, description="Standard deviation of the prior of the first state [K]")


@dataclass(frozen=True)
class OptimalControlResult:
    states: np.ndarray  # (states, N + 1) at the step boundaries
    controls: np.ndarray  # (controls, N), in the units of the model
    slack: np.ndarray  # (zones, N) [K]
    energy_cost: float
    status: str
    solve_time: float

    @property
    def success(self) -> bool:
        return self.status in SOLVED


def _collocation(degree: int) -> tuple[np.ndarray, np.ndarray]:
    """Radau IIA collocation: ``C[j, r]`` (derivative of the Lagrange basis j at point r) and ``D[j]`` (value at 1)."""
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
    return c, d


class _Collocated:
    """The model of an interface, collocated on Radau points, with its scaled controls and a CVODES simulator."""

    def __init__(self, interface: MPCModelInterface, rhs: ca.Function, settings: CollocationSettings) -> None:
        self.interface = interface
        self.settings = settings
        self.controls = interface.controls
        self.disturbances = interface.disturbances
        self.control_index = [interface.input_names.index(signal.name) for signal in self.controls]
        self.disturbance_index = [interface.input_names.index(signal.name) for signal in self.disturbances]
        self.indoor = [interface.state_names.index(zone.indoor_temperature) for zone in interface.zones]
        self.lower, self.upper = np.array([self._bounds(signal) for signal in self.controls]).T
        self.power = self._power_coefficients()
        meters = {i: state.name.rsplit(".", 1)[-1] for i, state in enumerate(interface.states) if state.unit == "kWh"}
        self.fuel_meters = [i for i, name in meters.items() if name == FUEL_METER]
        self.production_meters = [i for i, name in meters.items() if name == PRODUCTION_METER]
        self.electricity_meters = [i for i, name in meters.items() if name not in (FUEL_METER, PRODUCTION_METER)]
        self.offset = np.array([KELVIN if state.unit == "K" else 0.0 for state in interface.states])
        self._interval = self._interval_function(rhs)
        self._step = self._step_function(rhs)

    # ------------------------------------------------------------------ setup
    def _bounds(self, signal: InputSpec) -> tuple[float, float]:
        if signal.minimum is not None and signal.maximum is not None:
            return signal.minimum, signal.maximum
        zone = next((zone for zone in self.interface.zones if zone.heating_input == signal.name), None)
        upper = (zone.design_heating_power if zone else None) or self.settings.default_heat_bound
        return signal.minimum if signal.minimum is not None else 0.0, upper

    def _power_coefficients(self) -> np.ndarray:
        """Heat drawn per unit of each control [W]: the ideal zone heat inputs (systems have energy meters)."""
        return np.array([1.0 if signal.unit == "W" else 0.0 for signal in self.controls])

    def _inputs(self, u: ca.MX, d: ca.MX) -> ca.MX:
        """The input vector of the model, controls and disturbances at their places."""
        signals = {index: u[position] for position, index in enumerate(self.control_index)}
        signals |= {index: d[position] for position, index in enumerate(self.disturbance_index)}
        return ca.vertcat(*(signals[index] for index in range(len(self.interface.inputs))))

    def _interval_function(self, rhs: ca.Function) -> ca.Function:
        """Collocation residuals and end state of one step, in scaled variables."""
        n_x, degree, step = len(self.interface.states), self.settings.degree, self.settings.time_step
        c, d = _collocation(degree)
        parameters = ca.DM(self.interface.parameter_values)
        offset = ca.DM(self.offset)
        z0 = ca.MX.sym("z0", n_x)
        flat = ca.MX.sym("zc", n_x * degree)  # the collocation points of the step, stacked
        zc = ca.reshape(flat, n_x, degree)
        v = ca.MX.sym("v", len(self.controls))
        w = ca.MX.sym("w", len(self.disturbances))
        u = ca.DM(self.lower) + ca.DM(self.upper - self.lower) * v
        residuals = []
        points = [z0] + [zc[:, j] for j in range(degree)]
        for j in range(1, degree + 1):
            slope = sum(c[r, j] * points[r] for r in range(degree + 1))
            residuals.append(step * rhs(0, points[j] + offset, self._inputs(u, w), parameters) - slope)
        end = sum(d[r] * points[r] for r in range(degree + 1))
        return ca.Function("interval", [z0, flat, v, w], [ca.vertcat(*residuals), end])

    def _step_function(self, rhs: ca.Function) -> ca.Function:
        """One step of the model with CVODES, controls and disturbances held."""
        x = ca.MX.sym("x", len(self.interface.states))
        u = ca.MX.sym("u", len(self.controls))
        w = ca.MX.sym("w", len(self.disturbances))
        ode = {
            "x": x,
            "p": ca.vertcat(u, w),
            "ode": rhs(0, x, self._inputs(u, w), ca.DM(self.interface.parameter_values)),
        }
        return ca.integrator("step", "cvodes", ode, 0, self.settings.time_step, {"abstol": 1e-6, "reltol": 1e-6})

    def simulate(self, initial_state: np.ndarray, controls: np.ndarray, disturbances: np.ndarray) -> np.ndarray:
        """States at the step boundaries, (states, N + 1), for controls (controls, N) held over each step."""
        states = [np.asarray(initial_state, dtype=float)]
        for k in range(controls.shape[1]):
            result = self._step(x0=states[-1], p=np.concatenate([controls[:, k], disturbances[:, k]]))
            states.append(np.asarray(result["xf"]).ravel())
        return np.column_stack(states)

    def _metered_cost(
        self, z: ca.MX, price: np.ndarray, feed_in_price: np.ndarray | None, fuel_price: np.ndarray | None
    ) -> ca.MX:
        """Cost read from the meters of the systems [kWh over each step], exact for any power law.

        Electricity is netted over each step: the import is the smooth positive part of consumption minus
        production, the export the rest (``0.5*(x + sqrt(x^2 + d^2))``, ``d`` = 1 Wh: smooth for IPOPT).
        """
        cost = ca.MX(0)

        def drawn(rows: list[int]) -> ca.MX:
            return ca.sum1(z[rows, 1:] - z[rows, :-1]) if rows else ca.MX.zeros(1, z.shape[1] - 1)

        if self.fuel_meters:
            cost += ca.sum2(ca.DM(fuel_price if fuel_price is not None else price).T * drawn(self.fuel_meters))
        if self.electricity_meters or self.production_meters:
            net = drawn(self.electricity_meters) - drawn(self.production_meters)
            bought = 0.5 * (net + ca.sqrt(net**2 + NET_SMOOTHING**2))
            sold = bought - net
            sell = feed_in_price if feed_in_price is not None else price
            cost += ca.sum2(ca.DM(price).T * bought) - ca.sum2(ca.DM(sell).T * sold)
        return cost

    def _bound_temperatures(self, opti: ca.Opti, z: ca.MX) -> None:
        low, high = self.settings.temperature_bounds
        temperatures = [i for i, state in enumerate(self.interface.states) if state.unit == "K"]
        if temperatures:
            opti.subject_to(opti.bounded(low - KELVIN, ca.vec(z[temperatures, :]), high - KELVIN))

    def _steps(self, z: ca.MX, zc: ca.MX, v: ca.MX | ca.DM, disturbances: np.ndarray) -> tuple[ca.MX, ca.MX]:
        """Collocation residuals and end states of every step.

        The steps are called one by one and the whole problem is expanded to SX by IPOPT's ``expand`` option: the
        exact Hessian is then the one IPOPT converges with on these nonconvex problems (a ``map`` of the step,
        mathematically equal, made IPOPT crawl).
        """
        residuals, ends = [], []
        for k in range(disturbances.shape[1]):
            residual, end = self._interval(z[:, k], zc[:, k], v[:, k], ca.DM(disturbances[:, k]))
            residuals.append(residual)
            ends.append(end)
        return ca.horzcat(*residuals), ca.horzcat(*ends)

    def _solve(self, opti: ca.Opti) -> tuple[Callable[..., Any], str]:
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

    def _scaled(self, controls: np.ndarray) -> np.ndarray:
        """Controls on [0, 1]; a control whose bounds are equal is fixed (0)."""
        span = np.where(self.upper > self.lower, self.upper - self.lower, 1.0)
        return np.asarray((controls - self.lower[:, None]) / span[:, None])


class OptimalControl(_Collocated):
    """Optimal control problem of a building model, solved with IPOPT."""

    settings: OptimalControlSettings

    def __init__(
        self, interface: MPCModelInterface, rhs: ca.Function, settings: OptimalControlSettings | None = None
    ) -> None:
        super().__init__(interface, rhs, settings or OptimalControlSettings())

    def solve(  # noqa: PLR0913
        self,
        initial_state: np.ndarray,
        disturbances: np.ndarray,
        comfort_lower: np.ndarray,
        price: np.ndarray,
        guess_states: np.ndarray | None = None,
        guess_controls: np.ndarray | None = None,
        feed_in_price: np.ndarray | None = None,
        fuel_price: np.ndarray | None = None,
    ) -> OptimalControlResult:
        """Optimal controls over the horizon of ``disturbances`` (disturbances, N).

        ``comfort_lower`` is the lowest indoor temperature, (N,) or (zones, N) [K]. ``price`` is the cost of the
        electricity bought and of the heat of ideal emitters, (N,) [per kWh]; the PV production first covers the
        electricity of the step, the rest is sold at ``feed_in_price`` (``price`` by default); ``fuel_price`` is
        the cost of the boiler fuel (``price`` by default).
        """
        settings = self.settings
        n_x, horizon, degree = len(self.interface.states), disturbances.shape[1], settings.degree
        n_u, n_z = len(self.controls), len(self.indoor)
        lower = np.broadcast_to(comfort_lower, (n_z, horizon))
        opti = ca.Opti()
        z = opti.variable(n_x, horizon + 1)
        zc = opti.variable(n_x * degree, horizon)
        v = opti.variable(n_u, horizon)
        slack = opti.variable(n_z, horizon)

        residuals, ends = self._steps(z, zc, v, disturbances)
        opti.subject_to(ca.vec(residuals) == 0)
        opti.subject_to(z[:, 1:] == ends)
        opti.subject_to(z[:, 0] == initial_state - self.offset)
        opti.subject_to(opti.bounded(0, ca.vec(v), 1))
        self._bound_temperatures(opti, z[:, 1:])
        opti.subject_to(ca.vec(slack) >= 0)
        for zone, index in enumerate(self.indoor):
            opti.subject_to(z[index, 1:] + KELVIN + slack[zone, :] >= ca.DM(lower[zone, :]).T)

        power = ca.DM(self.power * (self.upper - self.lower)).T @ v + float(self.power @ self.lower)
        energy = ca.sum2(ca.DM(price).T * power) * settings.time_step / 3.6e6
        energy += self._metered_cost(z, price, feed_in_price, fuel_price)
        hours = settings.time_step / 3600
        comfort = settings.comfort_weight * hours * (ca.sumsqr(slack) + ca.sum1(ca.sum2(slack)))
        moves = settings.move_weight * ca.sumsqr(v[:, 1:] - v[:, :-1]) if horizon > 1 else 0
        effort = settings.effort_weight * ca.sumsqr(v)
        opti.minimize(energy + comfort + moves + effort)

        if guess_controls is None:
            guess_controls = np.tile(((self.lower + self.upper) / 2)[:, None], horizon)
        if guess_states is None:  # a trajectory of the model: consistent with the dynamics, a good start for IPOPT
            guess_states = self.simulate(initial_state, guess_controls, disturbances)
        scaled_guess = guess_states - self.offset[:, None]
        opti.set_initial(z, scaled_guess)
        opti.set_initial(zc, np.tile(scaled_guess[:, 1:], (degree, 1)))
        opti.set_initial(v, self._scaled(guess_controls))
        started = time.monotonic()
        values, status = self._solve(opti)
        elapsed = time.monotonic() - started
        scaled = np.atleast_2d(values(v)).reshape(n_u, horizon)
        controls = self.lower[:, None] + (self.upper - self.lower)[:, None] * scaled
        return OptimalControlResult(
            states=np.atleast_2d(values(z)).reshape(n_x, horizon + 1) + self.offset[:, None],
            controls=controls,
            slack=np.atleast_2d(values(slack)).reshape(n_z, horizon),
            energy_cost=float(values(energy)),
            status=status,
            solve_time=elapsed,
        )


@dataclass(frozen=True)
class EstimationResult:
    states: np.ndarray  # (states, N + 1) over the window; the last column is the current estimate
    process_noise: np.ndarray  # (states, N) estimated model errors
    status: str
    solve_time: float

    @property
    def current(self) -> np.ndarray:
        return self.states[:, -1]

    @property
    def success(self) -> bool:
        return self.status in SOLVED


class MovingHorizonEstimation(_Collocated):
    """State estimation over a past window: the trajectory of the model closest to the measurements.

    Minimizes, over the states of the window, the weighted squares of the measurement errors, of the model errors
    over each step (process noise added to the end state of the step) and of the deviation of the first state
    from its prior (arrival cost). The unmeasured states (walls, water, buffers) follow from the dynamics.
    """

    settings: EstimationSettings

    def __init__(
        self, interface: MPCModelInterface, rhs: ca.Function, settings: EstimationSettings | None = None
    ) -> None:
        super().__init__(interface, rhs, settings or EstimationSettings())

    def estimate(
        self,
        prior: np.ndarray,
        controls: np.ndarray,
        disturbances: np.ndarray,
        measurements: np.ndarray,
        measured: list[str] | None = None,
    ) -> EstimationResult:
        """States over the window of ``controls`` (controls, N) as applied and ``disturbances`` (disturbances, N).

        ``measurements`` (measured states, N + 1) are the measured values of the states ``measured`` (the zone air
        temperatures by default) at the step boundaries; NaN marks a missing measurement.
        """
        settings = self.settings
        measured = measured or [zone.indoor_temperature for zone in self.interface.zones]
        rows = [self.interface.state_names.index(name) for name in measured]
        n_x, horizon, degree = len(self.interface.states), controls.shape[1], settings.degree
        opti = ca.Opti()
        z = opti.variable(n_x, horizon + 1)
        zc = opti.variable(n_x * degree, horizon)
        noise = opti.variable(n_x, horizon)
        residuals, ends = self._steps(z, zc, ca.DM(self._scaled(controls)), disturbances)
        opti.subject_to(ca.vec(residuals) == 0)
        opti.subject_to(z[:, 1:] == ends + noise)
        self._bound_temperatures(opti, z)

        offsets = self.offset[rows][:, None]
        observed = ~np.isnan(measurements)
        errors = (z[rows, :] - ca.DM(np.where(observed, measurements - offsets, 0.0))) * ca.DM(observed.astype(float))
        objective = ca.sumsqr(errors) / settings.measurement_noise**2
        objective += ca.sumsqr(noise) / settings.process_noise**2
        objective += ca.sumsqr(z[:, 0] - (prior - self.offset)) / settings.arrival_noise**2
        opti.minimize(objective)

        guess = self.simulate(prior, controls, disturbances) - self.offset[:, None]
        opti.set_initial(z, guess)
        opti.set_initial(zc, np.tile(guess[:, 1:], (degree, 1)))
        started = time.monotonic()
        values, status = self._solve(opti)
        return EstimationResult(
            states=np.atleast_2d(values(z)).reshape(n_x, horizon + 1) + self.offset[:, None],
            process_noise=np.atleast_2d(values(noise)).reshape(n_x, horizon),
            status=status,
            solve_time=time.monotonic() - started,
        )


@dataclass(frozen=True)
class ClosedLoopResult:
    states: np.ndarray  # (states, steps + 1) of the plant
    controls: np.ndarray  # (controls, steps) applied
    statuses: list[str]
    solve_times: list[float]


Plant = Callable[[np.ndarray, np.ndarray, np.ndarray], np.ndarray]
"""One step of the controlled system: (state, controls, disturbances) -> next state."""


def receding_horizon(  # noqa: PLR0913
    ocp: OptimalControl,
    initial_state: np.ndarray,
    disturbances: np.ndarray,
    comfort_lower: np.ndarray,
    price: np.ndarray,
    horizon: int,
    plant: Plant | None = None,
    feed_in_price: np.ndarray | None = None,
) -> ClosedLoopResult:
    """Model predictive control: at each step, solve over ``horizon`` steps and apply the first controls.

    ``disturbances`` (disturbances, steps + horizon), ``comfort_lower`` and ``price`` (steps + horizon,) cover the
    closed loop and the last horizon; ``plant`` defaults to the model itself (CVODES). Each solve is warm-started
    from the previous solution, shifted by one step.
    """

    def model_plant(state: np.ndarray, controls: np.ndarray, forecast: np.ndarray) -> np.ndarray:
        return ocp.simulate(state, controls[:, None], forecast[:, None])[:, -1]

    step = plant or model_plant
    steps = disturbances.shape[1] - horizon
    states, applied, statuses, times = [np.asarray(initial_state, dtype=float)], [], [], []
    guess_states = guess_controls = None
    for k in range(steps):
        window = slice(k, k + horizon)
        result = ocp.solve(
            states[-1],
            disturbances[:, window],
            np.asarray(comfort_lower)[..., window],
            np.asarray(price)[window],
            guess_states=guess_states,
            guess_controls=guess_controls,
            feed_in_price=None if feed_in_price is None else np.asarray(feed_in_price)[window],
        )
        statuses.append(result.status)
        times.append(result.solve_time)
        applied.append(result.controls[:, 0])
        states.append(step(states[-1], result.controls[:, 0], disturbances[:, k]))
        guess_controls = np.column_stack([result.controls[:, 1:], result.controls[:, -1:]])
        guess_states = np.column_stack([states[-1], result.states[:, 2:], result.states[:, -1:]])
    return ClosedLoopResult(
        states=np.column_stack(states), controls=np.column_stack(applied), statuses=statuses, solve_times=times
    )
