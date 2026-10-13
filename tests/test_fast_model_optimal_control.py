"""Optimal control of the hydronic multilayer model with IPOPT (``trano.fast_model.optimal_control``)."""

import warnings
from pathlib import Path

import casadi as ca
import numpy as np
import pytest
import rumoca

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.fast_model.interface import FastModelInterface
from trano.fast_model.modelica import network_interface
from trano.fast_model.optimal_control import (
    KELVIN,
    EstimationSettings,
    MovingHorizonEstimation,
    OptimalControl,
    SOLVED,
    OptimalControlSettings,
    _collocation,
    receding_horizon,
)

MODELS = Path(__file__).parent / "models"


def _exports_connected_models() -> bool:
    from tests.test_fast_model_connected import _exports_connected_models as probe

    return probe()


pytestmark = pytest.mark.skipif(
    not _exports_connected_models(), reason="the CasADi export of connected models needs rumoca >= 0.10.3"
)


@pytest.fixture(scope="module")
def hydronic() -> tuple[FastModelInterface, object]:
    path = MODELS / "single_zone_hydronic.yaml"
    library = Library.from_configuration("FastModel")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        network = convert_network(path.stem, path, library=library)
    interface = network_interface(network)
    export = rumoca.Session().loads(network.model(), model=interface.model).to_casadi()
    return interface, export


def _forecast(interface: FastModelInterface, hours: np.ndarray) -> np.ndarray:
    """Synthetic winter day: weather, sun on the facades and occupancy, from the disturbance sources."""
    sun = np.clip(np.sin(np.pi * (hours - 7) / 10), 0, None) * (hours <= 17)
    rows = []
    for signal in interface.disturbances:
        source = signal.source
        assert source is not None
        if source.kind == "weather":
            weather = {
                "TDryBul": KELVIN + 2 + 4 * np.sin(2 * np.pi * (hours - 9) / 24),
                "TBlaSky": np.full(hours.shape, KELVIN - 8),
                "winSpe": np.full(hours.shape, 3.0),
                "winDir": np.zeros(hours.shape),
            }
            rows.append(weather[source.variable])
        elif source.kind == "irradiance":
            rows.append(
                {"direct": 300 * sun, "diffuse_perez": 100 * sun, "incidence_cosine": 0.6 * sun}[source.component]
            )
        else:
            rows.append(np.where((hours >= 7) & (hours < 19), 200.0, 0.0))
    return np.vstack(rows)


@pytest.mark.parametrize("degree", [1, 2, 3])
def test_radau_collocation_is_exact_for_polynomials(degree: int) -> None:
    """The Lagrange basis on the Radau points reproduces t^p and its derivative for p <= degree."""
    c, d = _collocation(degree)
    tau = np.append(0.0, ca.collocation_points(degree, "radau"))
    for power in range(degree + 1):
        values = tau**power
        assert values @ d == pytest.approx(1.0)  # value at the end of the step
        expected = power * tau[1:] ** (power - 1) if power else np.zeros(degree)
        assert (c.T @ values)[1:] == pytest.approx(expected)


def test_simulation_follows_the_controls(hydronic: tuple[FastModelInterface, object]) -> None:
    interface, export = hydronic
    ocp = OptimalControl(interface, export.rhs, OptimalControlSettings(time_step=3600))  # type: ignore[attr-defined]
    hours = np.arange(6.0)
    disturbances = _forecast(interface, hours)
    x0 = np.array(interface.initial_state)
    off = ocp.simulate(x0, np.tile(ocp.lower[:, None], 6), disturbances)
    on = ocp.simulate(x0, np.tile(ocp.upper[:, None], 6), disturbances)
    indoor = interface.state_names.index(interface.zones[0].indoor_temperature)
    assert on[indoor, -1] > off[indoor, -1] + 1


def test_the_optimal_control_heats_just_enough(hydronic: tuple[FastModelInterface, object]) -> None:
    interface, export = hydronic
    ocp = OptimalControl(interface, export.rhs, OptimalControlSettings(time_step=3600))  # type: ignore[attr-defined]
    hours = np.arange(24.0)
    comfort = np.where((hours >= 7) & (hours < 22), KELVIN + 20, KELVIN + 16)
    x0 = np.array(interface.initial_state)
    result = ocp.solve(x0, _forecast(interface, hours), comfort, price=np.full(24, 0.1))

    assert result.success, result.status
    indoor = result.states[interface.state_names.index(interface.zones[0].indoor_temperature)]
    assert np.all(indoor[1:] >= comfort - 0.05)
    assert np.all(indoor[9:21] <= KELVIN + 20.5)  # no overheating: the comfort bound is tracked, not exceeded
    assert np.all(result.controls >= ocp.lower[:, None] - 1e-6)
    assert np.all(result.controls <= ocp.upper[:, None] + 1e-6)
    boiler = [i for i, signal in enumerate(ocp.controls) if signal.name == "boiler_001_y"]
    assert result.controls[boiler, :6].max() < 0.05  # the night setback is used: no firing before preheating
    assert result.energy_cost > 0

    # the collocated trajectory is the one of the model under the optimal controls
    replay = ocp.simulate(x0, result.controls, _forecast(interface, hours))
    assert np.abs(replay[interface.state_names.index(interface.zones[0].indoor_temperature)] - indoor).max() < 0.2


def test_moving_horizon_estimation_recovers_the_unmeasured_states(
    hydronic: tuple[FastModelInterface, object],
) -> None:
    """The house starts 2 K colder than the prior believes; only the zone air is measured, with noise."""
    interface, export = hydronic
    settings = EstimationSettings(time_step=3600)
    estimator = MovingHorizonEstimation(interface, export.rhs, settings)  # type: ignore[attr-defined]
    hours = np.arange(12.0)
    disturbances = _forecast(interface, hours)
    controls = np.tile(((estimator.lower + estimator.upper) / 2)[:, None], len(hours))
    prior = np.array(interface.initial_state)
    temperatures = np.array([state.unit == "K" for state in interface.states])
    truth = estimator.simulate(prior - 2.0 * temperatures, controls, disturbances)
    air = interface.state_names.index(interface.zones[0].indoor_temperature)
    noise = np.random.default_rng(0).normal(0, 0.05, truth.shape[1])
    result = estimator.estimate(prior, controls, disturbances, truth[[air]] + noise)

    assert result.success, result.status
    error = np.abs(result.current - truth[:, -1])[temperatures]
    believed = np.abs(estimator.simulate(prior, controls, disturbances)[:, -1] - truth[:, -1])[temperatures]
    assert error[interface.state_names.index(interface.zones[0].indoor_temperature)] < 0.1
    assert np.max(error) < 0.5 * np.max(believed)


def test_the_receding_horizon_keeps_the_zone_comfortable(hydronic: tuple[FastModelInterface, object]) -> None:
    """Closed loop from 5:00: six MPC steps over a 12-hour horizon, the model as plant."""
    interface, export = hydronic
    ocp = OptimalControl(interface, export.rhs, OptimalControlSettings(time_step=3600))  # type: ignore[attr-defined]
    hours = 5.0 + np.arange(18.0)
    comfort = np.where((hours >= 7) & (hours < 22), KELVIN + 20, KELVIN + 16)
    loop = receding_horizon(
        ocp,
        np.array(interface.initial_state),
        _forecast(interface, hours),
        comfort,
        np.full(18, 0.1),
        horizon=12,
    )
    assert all(status in SOLVED for status in loop.statuses), loop.statuses
    indoor = loop.states[interface.state_names.index(interface.zones[0].indoor_temperature)]
    assert np.all(indoor[1:] >= comfort[:6] - 0.1)
    assert np.all(loop.controls >= ocp.lower[:, None] - 1e-6)
    assert np.all(loop.controls <= ocp.upper[:, None] + 1e-6)


def test_the_heat_pump_house_is_optimized() -> None:
    """Six zones, a heat pump with its buffer, six radiator branches and a mixing valve: 6 hourly steps."""
    path = MODELS / "house_infiltration_air_water_heat_pump.yaml"
    library = Library.from_configuration("FastModel")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        network = convert_network(path.stem, path, library=library)
    interface = network_interface(network)
    export = rumoca.Session().loads(network.model(), model=interface.model).to_casadi()
    ocp = OptimalControl(interface, export.rhs, OptimalControlSettings(time_step=3600))
    hours = 5.0 + np.arange(6.0)
    comfort = np.where(hours >= 7, KELVIN + 19, KELVIN + 16)
    result = ocp.solve(np.array(interface.initial_state), _forecast(interface, hours), comfort, np.full(6, 0.3))

    assert result.success, result.status
    indoor = [interface.state_names.index(zone.indoor_temperature) for zone in interface.zones]
    assert np.all(result.states[indoor, 1:] >= comfort - 0.05)
    assert "boiler_001.EEle" in interface.state_names  # the electricity of the heat pump is the energy cost
    assert result.energy_cost >= 0
