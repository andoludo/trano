"""Economic model predictive controller for the RC building models (CasADi + IPOPT).

At every control step the controller solves the optimal control problem::

    min   sum_k  price[k] * sum_z QHea[k, z] * dt / 3.6e6                       (energy cost)
        + comfort_weight * sum_k sum_z (sLow[k, z] + sHigh[k, z])              (discomfort, exact L1 penalty)
        + comfort_quadratic_weight * sum_k sum_z (sLow[k, z]^2 + sHigh[k, z]^2)
        + smoothness_weight * sum_k sum_z ((QHea[k, z] - QHea[k-1, z]) / QMax)^2
    s.t.  x[k+1] = F(x[k], QHea[k], d[k], p)                                   (RC model, multiple shooting)
          TLow[k, z] - sLow[k, z] <= Ti[k+1, z] <= THigh[k, z] + sHigh[k, z]  (soft comfort band)
          0 <= QHea[k, z] <= QMax[z],  sLow, sHigh >= 0

The problem is built once as a parametric NLP (``casadi.Opti``); the initial state and the
forecasts are Opti parameters, so re-solving at each step only updates numbers and warm
starts IPOPT from the previous solution.
"""

from dataclasses import dataclass
from typing import Any

import casadi as ca
import numpy as np
import numpy.typing as npt
from pydantic import BaseModel, ConfigDict, Field, PositiveFloat, PositiveInt

from trano.mpc.casadi_model import CasadiRCModel, FloatArray

KELVIN = 273.15
JOULE_PER_KWH = 3.6e6


class MPCSettings(BaseModel):
    model_config = ConfigDict(frozen=True)

    horizon: PositiveInt = Field(24, description="Number of steps of the prediction horizon")
    time_step: PositiveFloat = Field(3600.0, description="Control time step [s]")
    max_heating_power: PositiveFloat | dict[str, PositiveFloat] = Field(
        5000.0, description="Maximum heating power [W], for all zones or per zone name"
    )
    comfort_weight: PositiveFloat = Field(
        100.0, description="Linear (exact) penalty on comfort violations [cost/(K.step)]"
    )
    comfort_quadratic_weight: float = Field(
        10.0, ge=0, description="Quadratic penalty on comfort violations [cost/(K^2.step)]"
    )
    smoothness_weight: float = Field(1e-3, ge=0, description="Penalty on the heating power variations")
    substeps: PositiveInt | None = Field(None, description="RK4 sub-steps, computed for stability when None")
    ipopt_options: dict[str, Any] = Field(
        default_factory=lambda: {"print_level": 0, "sb": "yes", "max_iter": 500, "tol": 1e-8}
    )


@dataclass(frozen=True)
class Forecast:
    """Disturbances and comfort band over the prediction horizon (``N`` steps).

    Temperatures are in Kelvin. ``internal_gains``, ``lower_temperature`` and
    ``upper_temperature`` are either ``(N,)`` (same for every zone) or ``(N, n_zones)``.
    ``price`` is the energy price per kWh of heat, ``(N,)``.
    """

    outdoor_temperature: npt.ArrayLike
    solar_irradiance: npt.ArrayLike
    internal_gains: npt.ArrayLike
    lower_temperature: npt.ArrayLike
    upper_temperature: npt.ArrayLike
    price: npt.ArrayLike

    def disturbances(self, model: CasadiRCModel, horizon: int) -> FloatArray:
        """``(n_disturbances, N)`` matrix ordered as ``model.disturbance_names``."""
        internal_gains = _per_zone(self.internal_gains, horizon, model.n_controls)
        signals = {
            "TOut": _series(self.outdoor_temperature, horizon),
            "HGlo": _series(self.solar_irradiance, horizon),
            **{f"{prefix}QInt": internal_gains[:, index] for index, prefix in enumerate(model.zone_prefixes)},
        }
        missing = set(model.disturbance_names) - set(signals)
        if missing:
            raise ValueError(f"No forecast available for the disturbances {sorted(missing)}.")
        return np.vstack([signals[name] for name in model.disturbance_names])

    def slice(self, start: int, horizon: int) -> "Forecast":
        """Forecast window ``[start, start + horizon)`` of a longer forecast."""

        def window(values: npt.ArrayLike) -> FloatArray:
            return np.asarray(values, dtype=np.float64)[start : start + horizon]

        return Forecast(
            outdoor_temperature=window(self.outdoor_temperature),
            solar_irradiance=window(self.solar_irradiance),
            internal_gains=window(self.internal_gains),
            lower_temperature=window(self.lower_temperature),
            upper_temperature=window(self.upper_temperature),
            price=window(self.price),
        )


def _series(values: npt.ArrayLike, horizon: int) -> FloatArray:
    array = np.asarray(values, dtype=np.float64).reshape(-1)
    if array.size < horizon:
        raise ValueError(f"Forecast of length {array.size} is shorter than the horizon {horizon}.")
    return array[:horizon]


def _per_zone(values: npt.ArrayLike, horizon: int, n_zones: int) -> FloatArray:
    array = np.asarray(values, dtype=np.float64)
    if array.ndim == 1:
        array = np.tile(array.reshape(-1, 1), (1, n_zones))
    if array.shape[0] < horizon:
        raise ValueError(f"Forecast of length {array.shape[0]} is shorter than the horizon {horizon}.")
    if array.shape[1] != n_zones:
        raise ValueError(f"Expected a ({horizon}, {n_zones}) forecast, got {array.shape}.")
    return array[:horizon]


@dataclass(frozen=True)
class MPCSolution:
    success: bool
    status: str
    objective: float
    heating_power: FloatArray
    """``(N, n_zones)`` optimal heating power [W]."""
    states: FloatArray
    """``(N + 1, n_states)`` predicted state trajectory [K]."""
    indoor_temperature: FloatArray
    """``(N + 1, n_zones)`` predicted indoor temperatures [K]."""
    comfort_violation: FloatArray
    """``(N, n_zones)`` predicted comfort violation [K] (lower + upper slack)."""
    energy_cost: float


class ModelPredictiveController:
    """Economic MPC of the heating power of each zone of an RC building model."""

    def __init__(
        self,
        model: CasadiRCModel,
        settings: MPCSettings | None = None,
        parameters: FloatArray | None = None,
    ) -> None:
        self.model = model
        self.settings = settings or MPCSettings()
        self.zones = [prefix.removesuffix("_") or "zone" for prefix in model.zone_prefixes]
        self.indoor_indices = [model.state_index(f"{prefix}Ti") for prefix in model.zone_prefixes]
        self.max_power = self._max_power()
        self._parameters = model.default_parameters if parameters is None else np.asarray(parameters, dtype=np.float64)
        self._build()
        self._previous: ca.OptiSol | None = None

    def _max_power(self) -> FloatArray:
        max_power = self.settings.max_heating_power
        if isinstance(max_power, dict):
            return np.array([max_power[zone] for zone in self.zones], dtype=np.float64)
        return np.full(len(self.zones), max_power, dtype=np.float64)

    def _build(self) -> None:
        horizon, dt = self.settings.horizon, self.settings.time_step
        n_zones, n_states = len(self.zones), self.model.n_states
        substeps = self.settings.substeps or self.model.stable_substeps(dt, self._parameters)
        dynamics = self.model.discrete_dynamics(dt, substeps=substeps)

        opti = ca.Opti()
        states = opti.variable(n_states, horizon + 1)
        # Heating power scaled by the maximum power: better conditioned for IPOPT.
        power_fraction = opti.variable(n_zones, horizon)
        slack_low = opti.variable(n_zones, horizon)
        slack_high = opti.variable(n_zones, horizon)

        initial_state = opti.parameter(n_states)
        disturbances = opti.parameter(self.model.n_disturbances, horizon)
        lower = opti.parameter(n_zones, horizon)
        upper = opti.parameter(n_zones, horizon)
        price = opti.parameter(1, horizon)
        parameters = opti.parameter(self.model.n_parameters)

        power = ca.diag(ca.DM(self.max_power)) @ power_fraction
        indoor = ca.vertcat(*[states[index, :] for index in self.indoor_indices])

        opti.subject_to(states[:, 0] == initial_state)
        for k in range(horizon):
            opti.subject_to(states[:, k + 1] == dynamics(states[:, k], power[:, k], disturbances[:, k], parameters))
        opti.subject_to(opti.bounded(0, ca.vec(power_fraction), 1))
        opti.subject_to(ca.vec(slack_low) >= 0)
        opti.subject_to(ca.vec(slack_high) >= 0)
        opti.subject_to(ca.vec(indoor[:, 1:] + slack_low - lower) >= 0)
        opti.subject_to(ca.vec(upper + slack_high - indoor[:, 1:]) >= 0)

        energy_cost = ca.sum2(price * ca.sum1(power)) * dt / JOULE_PER_KWH
        discomfort = ca.sum1(ca.sum2(slack_low + slack_high))
        discomfort_quadratic = ca.sumsqr(slack_low) + ca.sumsqr(slack_high)
        smoothness = ca.sumsqr(ca.diff(power_fraction, 1, 1))
        opti.minimize(
            energy_cost
            + self.settings.comfort_weight * discomfort
            + self.settings.comfort_quadratic_weight * discomfort_quadratic
            + self.settings.smoothness_weight * smoothness
        )
        opti.solver("ipopt", {"print_time": False, "error_on_fail": False}, self.settings.ipopt_options)

        self._opti = opti
        self._variables = {
            "states": states,
            "power_fraction": power_fraction,
            "slack_low": slack_low,
            "slack_high": slack_high,
        }
        self._parameters_symbols = {
            "initial_state": initial_state,
            "disturbances": disturbances,
            "lower": lower,
            "upper": upper,
            "price": price,
            "parameters": parameters,
        }
        self._expressions = {"power": power, "indoor": indoor, "energy_cost": energy_cost}

    def solve(self, forecast: Forecast, initial_state: npt.ArrayLike | None = None) -> MPCSolution:
        """Solve the optimal control problem over the horizon starting at ``initial_state``."""
        horizon, n_zones = self.settings.horizon, len(self.zones)
        x0 = self.model.initial_state if initial_state is None else np.asarray(initial_state, dtype=np.float64)
        disturbances = forecast.disturbances(self.model, horizon)
        values = {
            "initial_state": x0,
            "disturbances": disturbances,
            "lower": _per_zone(forecast.lower_temperature, horizon, n_zones).T,
            "upper": _per_zone(forecast.upper_temperature, horizon, n_zones).T,
            "price": _series(forecast.price, horizon).reshape(1, -1),
            "parameters": self._parameters,
        }
        for name, numeric_value in values.items():
            self._opti.set_value(self._parameters_symbols[name], numeric_value)
        self._warm_start(x0, disturbances)

        solution = self._opti.solve()
        stats = self._opti.stats()
        success = bool(stats.get("success", False))
        if success:
            self._previous = solution

        def value(expression: ca.MX) -> FloatArray:
            return np.atleast_2d(np.array(solution.value(expression), dtype=np.float64))

        slack = value(self._variables["slack_low"]) + value(self._variables["slack_high"])
        return MPCSolution(
            success=success,
            status=str(stats.get("return_status", "")),
            objective=float(solution.value(self._opti.f)),
            # IPOPT relaxes the bounds by ~1e-8: clip to the physical bounds.
            heating_power=np.clip(value(self._expressions["power"]).reshape(n_zones, horizon).T, 0, self.max_power),
            states=value(self._variables["states"]).reshape(self.model.n_states, horizon + 1).T,
            indoor_temperature=value(self._expressions["indoor"]).reshape(n_zones, horizon + 1).T,
            comfort_violation=slack.reshape(n_zones, horizon).T,
            energy_cost=float(solution.value(self._expressions["energy_cost"])),
        )

    def _warm_start(self, initial_state: FloatArray, disturbances: FloatArray) -> None:
        if self._previous is None:
            # Cold start: free-floating prediction with zero heating.
            n_steps = self.settings.horizon
            prediction = self.model.simulate(
                np.zeros((n_steps, len(self.zones))),
                disturbances.T,
                self.settings.time_step,
                initial_state=initial_state,
                parameters=self._parameters,
            )
            self._opti.set_initial(self._variables["states"], prediction.T)
            return
        # Shift the previous solution by one step.
        for variable in self._variables.values():
            previous = np.atleast_2d(np.array(self._previous.value(variable), dtype=np.float64))
            previous = previous.reshape(variable.shape)
            self._opti.set_initial(variable, np.hstack([previous[:, 1:], previous[:, -1:]]))
        self._opti.set_initial(self._variables["states"][:, 0], initial_state)


@dataclass(frozen=True)
class ClosedLoopResult:
    time: FloatArray
    """``(n_steps + 1,)`` time [s]."""
    states: FloatArray
    """``(n_steps + 1, n_states)`` simulated states [K]."""
    heating_power: FloatArray
    """``(n_steps, n_zones)`` applied heating power [W]."""
    energy_cost: float
    comfort_violation: float
    """Sum over the steps and zones of the comfort band violation [K.step]."""


def run_closed_loop(
    controller: ModelPredictiveController,
    forecast: Forecast,
    n_steps: int,
    plant: CasadiRCModel | None = None,
    initial_state: npt.ArrayLike | None = None,
) -> ClosedLoopResult:
    """Receding horizon simulation: apply the first optimal move, simulate the plant, repeat.

    ``forecast`` must cover ``n_steps + horizon`` steps. The plant defaults to the controller
    model (perfect model). To study the model mismatch, pass another model, e.g. with other
    parameters: ``dataclasses.replace(model, default_parameters=model.parameter_values(zone_Ci=1e7))``.
    """
    plant = plant or controller.model
    horizon, dt = controller.settings.horizon, controller.settings.time_step
    if np.asarray(forecast.price).size < n_steps + horizon:
        raise ValueError("The forecast must cover n_steps + horizon steps.")
    step = plant.discrete_dynamics(dt)
    state = plant.initial_state if initial_state is None else np.asarray(initial_state, dtype=np.float64)
    states, powers, cost, violation = [state], [], 0.0, 0.0
    for k in range(n_steps):
        window = forecast.slice(k, horizon)
        solution = controller.solve(window, initial_state=state)
        power = solution.heating_power[0]
        disturbances = window.disturbances(plant, horizon)[:, 0]
        state = np.array(step(state, power, disturbances, plant.default_parameters), dtype=np.float64).reshape(-1)
        states.append(state)
        powers.append(power)
        cost += float(np.asarray(window.price, dtype=np.float64)[0] * power.sum() * dt / JOULE_PER_KWH)
        indoor = state[controller.indoor_indices]
        lower = _per_zone(window.lower_temperature, horizon, len(controller.zones))[0]
        upper = _per_zone(window.upper_temperature, horizon, len(controller.zones))[0]
        violation += float(np.sum(np.maximum(lower - indoor, 0) + np.maximum(indoor - upper, 0)))
    return ClosedLoopResult(
        time=np.arange(n_steps + 1, dtype=np.float64) * dt,
        states=np.vstack(states),
        heating_power=np.vstack(powers),
        energy_cost=cost,
        comfort_violation=violation,
    )
