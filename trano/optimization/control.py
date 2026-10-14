"""Optimal control of a building model with IPOPT: direct collocation of its CasADi ODE.

Over a horizon of ``N`` steps from the time ``t0``, the problem is

    minimize    energy cost  +  comfort_weight * sum(slack^2 + slack)
                +  move_weight * sum(du^2)  +  effort_weight * sum(u^2)          (u scaled to [0, 1])
    subject to  the ODE, collocated on Radau IIA points in each step (stable for stiff water nodes),
                lower_k - slack_k <= each comfort temperature,   slack >= 0,
                the bounds of each control,

where the energy cost reads the meters of the systems (states in kWh, so that any nonlinear power is integrated
exactly) and the heat of each control in W. Electricity is netted over each step: the production first covers the
consumption, the rest is sold at the feed-in price.
"""

import time
from collections.abc import Callable
from dataclasses import dataclass

import casadi as ca
import numpy as np
from pydantic import Field

from .collocation import KELVIN, SOLVED, Collocated, CollocationSettings
from .model import CasadiModel
from .roles import Roles

NET_SMOOTHING = 1e-3  # [kWh] of the positive part of the net electricity


class OptimalControlSettings(CollocationSettings):
    comfort_weight: float = Field(1e3, ge=0, description="Cost of one kelvin-hour of discomfort")
    move_weight: float = Field(1e-2, ge=0, description="Cost of a full-range change of a control between steps")
    effort_weight: float = Field(
        1e-3,
        ge=0,
        description="Cost of a control at full range per step: separates the controls that deliver the same heat, "
        "whose flat valley IPOPT would otherwise crawl along",
    )


@dataclass(frozen=True)
class OptimalControlResult:
    states: np.ndarray  # (states, N + 1) at the step boundaries
    controls: np.ndarray  # (controls, N), in the units of the model
    slack: np.ndarray  # (comfort temperatures, N) [K]
    energy_cost: float
    status: str
    solve_time: float

    @property
    def success(self) -> bool:
        return self.status in SOLVED


class OptimalControl(Collocated):
    """Optimal control problem of a model, solved with IPOPT."""

    settings: OptimalControlSettings

    def __init__(
        self,
        model: CasadiModel,
        roles: Roles | None = None,
        settings: OptimalControlSettings | None = None,
        parameters: np.ndarray | None = None,
    ) -> None:
        super().__init__(model, roles, settings or OptimalControlSettings(), parameters)
        self.heat_inputs = np.array(
            [1.0 if self.description.input(name).unit == "W" else 0.0 for name in self.roles.controls]
        )
        meters = self.roles.meters
        names = self.description.state_names
        self.fuel_meters = [names.index(name) for name, carrier in meters.items() if carrier == "fuel"]
        self.production_meters = [
            names.index(name) for name, carrier in meters.items() if carrier == "electricity_production"
        ]
        self.electricity_meters = [names.index(name) for name, carrier in meters.items() if carrier == "electricity"]

    def _metered_cost(
        self, z: ca.MX, price: np.ndarray, feed_in_price: np.ndarray | None, fuel_price: np.ndarray | None
    ) -> ca.MX:
        """Cost read from the meters [kWh over each step]; electricity netted with a smooth positive part."""
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

    def solve(  # noqa: PLR0913
        self,
        initial_state: np.ndarray,
        disturbances: np.ndarray,
        comfort_lower: np.ndarray,
        price: np.ndarray,
        start_time: float = 0.0,
        guess_states: np.ndarray | None = None,
        guess_controls: np.ndarray | None = None,
        feed_in_price: np.ndarray | None = None,
        fuel_price: np.ndarray | None = None,
    ) -> OptimalControlResult:
        """Optimal controls over the horizon of ``disturbances`` (disturbances, N) from ``start_time`` [s].

        ``comfort_lower`` is the lowest comfort temperature, (N,) or (comfort temperatures, N) [K]. ``price`` is the
        cost of the electricity bought and of the heat of the W controls, (N,) [per kWh]; the production is sold at
        ``feed_in_price`` (``price`` by default); ``fuel_price`` is the cost of the fuel (``price`` by default).
        """
        settings = self.settings
        n_x, horizon, degree = self.n_states, disturbances.shape[1], settings.degree
        n_u, n_c = len(self.roles.controls), len(self.comfort_index)
        lower = np.broadcast_to(comfort_lower, (n_c, horizon))
        opti = ca.Opti()
        z = opti.variable(n_x, horizon + 1)
        zc = opti.variable(n_x * degree, horizon)
        v = opti.variable(n_u, horizon)
        slack = opti.variable(n_c, horizon)

        residuals, ends = self.steps(z, zc, v, disturbances, start_time)
        opti.subject_to(ca.vec(residuals) == 0)
        opti.subject_to(z[:, 1:] == ends)
        opti.subject_to(z[:, 0] == np.asarray(initial_state, dtype=float) - self.offset)
        opti.subject_to(opti.bounded(0, ca.vec(v), 1))
        self.bound_temperatures(opti, z[:, 1:])
        opti.subject_to(ca.vec(slack) >= 0)
        for position, index in enumerate(self.comfort_index):
            opti.subject_to(z[index, 1:] + KELVIN + slack[position, :] >= ca.DM(lower[position, :]).T)

        heat = ca.DM(self.heat_inputs * (self.upper - self.lower)).T @ v + float(self.heat_inputs @ self.lower)
        energy = ca.sum2(ca.DM(price).T * heat) * settings.time_step / 3.6e6
        energy += self._metered_cost(z, price, feed_in_price, fuel_price)
        hours = settings.time_step / 3600
        comfort = settings.comfort_weight * hours * (ca.sumsqr(slack) + ca.sum1(ca.sum2(slack)))
        moves = settings.move_weight * ca.sumsqr(v[:, 1:] - v[:, :-1]) if horizon > 1 else 0
        effort = settings.effort_weight * ca.sumsqr(v)
        opti.minimize(energy + comfort + moves + effort)

        if guess_controls is None:
            guess_controls = np.tile(((self.lower + self.upper) / 2)[:, None], horizon)
        if guess_states is None:  # a trajectory of the model: consistent with the dynamics, a good start for IPOPT
            guess_states = self.simulate(initial_state, guess_controls, disturbances, start_time)
        scaled_guess = guess_states - self.offset[:, None]
        opti.set_initial(z, scaled_guess)
        opti.set_initial(zc, np.tile(scaled_guess[:, 1:], (degree, 1)))
        opti.set_initial(v, self.scaled(guess_controls))
        started = time.monotonic()
        values, status = self.solve_nlp(opti)
        elapsed = time.monotonic() - started
        return OptimalControlResult(
            states=np.atleast_2d(values(z)).reshape(n_x, horizon + 1) + self.offset[:, None],
            controls=self.unscaled(np.atleast_2d(values(v)).reshape(n_u, horizon)),
            slack=np.atleast_2d(values(slack)).reshape(n_c, horizon),
            energy_cost=float(values(energy)),
            status=status,
            solve_time=elapsed,
        )


@dataclass(frozen=True)
class ClosedLoopResult:
    states: np.ndarray  # (states, steps + 1) of the plant
    controls: np.ndarray  # (controls, steps) applied
    statuses: list[str]
    solve_times: list[float]


Plant = Callable[[float, np.ndarray, np.ndarray, np.ndarray], np.ndarray]
"""One step of the controlled system: (time, state, controls, disturbances) -> next state."""


def receding_horizon(  # noqa: PLR0913
    ocp: OptimalControl,
    initial_state: np.ndarray,
    disturbances: np.ndarray,
    comfort_lower: np.ndarray,
    price: np.ndarray,
    horizon: int,
    start_time: float = 0.0,
    plant: Plant | None = None,
    feed_in_price: np.ndarray | None = None,
) -> ClosedLoopResult:
    """Model predictive control: at each step, solve over ``horizon`` steps and apply the first controls.

    ``disturbances`` (disturbances, steps + horizon), ``comfort_lower`` and ``price`` (steps + horizon,) cover the
    closed loop and the last horizon; ``plant`` defaults to the model itself (CVODES). Each solve is warm-started
    from the previous solution, shifted by one step.
    """
    step_length = ocp.settings.time_step

    def model_plant(time_: float, state: np.ndarray, controls: np.ndarray, forecast: np.ndarray) -> np.ndarray:
        return ocp.simulate(state, controls[:, None], forecast[:, None], time_)[:, -1]

    step = plant or model_plant
    steps = disturbances.shape[1] - horizon
    states, applied, statuses, times = [np.asarray(initial_state, dtype=float)], [], [], []
    guess_states = guess_controls = None
    for k in range(steps):
        window = slice(k, k + horizon)
        now = start_time + k * step_length
        result = ocp.solve(
            states[-1],
            disturbances[:, window],
            np.asarray(comfort_lower)[..., window],
            np.asarray(price)[window],
            start_time=now,
            guess_states=guess_states,
            guess_controls=guess_controls,
            feed_in_price=None if feed_in_price is None else np.asarray(feed_in_price)[window],
        )
        statuses.append(result.status)
        times.append(result.solve_time)
        applied.append(result.controls[:, 0])
        states.append(step(now, states[-1], result.controls[:, 0], disturbances[:, k]))
        guess_controls = np.column_stack([result.controls[:, 1:], result.controls[:, -1:]])
        guess_states = np.column_stack([states[-1], result.states[:, 2:], result.states[:, -1:]])
    return ClosedLoopResult(
        states=np.column_stack(states), controls=np.column_stack(applied), statuses=statuses, solve_times=times
    )
