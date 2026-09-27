from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("casadi")
pytest.importorskip("rumoca")

from trano.mpc import ISO13790Parameters, R3C2Parameters, RCBuilding, RCModelType, RCZone, rc_building_from_yaml
from trano.mpc.casadi_model import CasadiRCModel
from trano.mpc.controller import (
    Forecast,
    ModelPredictiveController,
    MPCSettings,
    run_closed_loop,
)
from trano.mpc.estimation import reference_zone_parameters

THREE_ZONES = Path(__file__).parent / "models" / "three_zones_ideal_heaters.yaml"
KELVIN = 273.15


@pytest.fixture(scope="module")
def three_zones() -> dict[RCModelType, RCBuilding]:
    return {model_type: rc_building_from_yaml(THREE_ZONES, model_type=model_type) for model_type in RCModelType}


def _single_zone(model_type: RCModelType) -> RCBuilding:
    return RCBuilding(zones=[RCZone(name="zone", parameters=reference_zone_parameters(model_type))])


@pytest.mark.parametrize("model_type", list(RCModelType))
def test_library_models_translate_to_casadi(model_type: RCModelType) -> None:
    source = _single_zone(model_type).to_modelica()
    model = CasadiRCModel.from_modelica(source, model=f"TranoRC.Zones.{model_type.value}")
    assert model.control_names == ("QHea",)
    assert model.disturbance_names == ("TOut", "HGlo", "QInt")


@pytest.mark.parametrize("model_type", list(RCModelType))
def test_building_translates_to_casadi(three_zones: dict[RCModelType, RCBuilding], model_type: RCModelType) -> None:
    building = three_zones[model_type]
    model = building.to_casadi()
    assert list(model.state_names) == building.state_names
    assert model.control_names == ("space_001_QHea", "space_002_QHea", "space_003_QHea")
    assert model.disturbance_names == ("TOut", "HGlo", "space_001_QInt", "space_002_QInt", "space_003_QInt")


def _evaluate(model: CasadiRCModel, x: list[float], u: list[float], d: list[float]) -> np.ndarray:
    return np.array(model.continuous_dynamics()(x, u, d, model.default_parameters)).reshape(-1)


def test_r3c2_equations_match_the_network() -> None:
    building = _single_zone(RCModelType.r3c2)
    p = building.zones[0].parameters
    assert isinstance(p, R3C2Parameters)
    ti, te, t_out, h_glo, q_int, q_hea = 293.0, 290.0, 273.0, 300.0, 200.0, 1500.0
    expected = [
        (
            (te - ti) / p.indoor_envelope_resistance
            + (t_out - ti) / p.indoor_outdoor_resistance
            + p.solar_aperture * h_glo
            + q_int
            + q_hea
        )
        / p.air_capacitance,
        (
            (ti - te) / p.indoor_envelope_resistance
            + (t_out - te) / p.envelope_outdoor_resistance
            + (building.ground_temperature - te) / p.envelope_ground_resistance
            + p.envelope_solar_aperture * h_glo
        )
        / p.envelope_capacitance,
    ]
    model = building.to_casadi()
    assert _evaluate(model, [ti, te], [q_hea], [t_out, h_glo, q_int]) == pytest.approx(expected, rel=1e-10)


def test_iso13790_equations_match_the_standard() -> None:
    building = _single_zone(RCModelType.iso13790)
    p = building.zones[0].parameters
    assert isinstance(p, ISO13790Parameters)
    ti, tm, t_out, h_glo, q_int, q_hea = 293.0, 291.0, 273.0, 300.0, 200.0, 1500.0
    radiant = (1 - p.convective_fraction) * q_int + p.solar_aperture * h_glo
    # ISO 13790 C.3: the surface node is massless, its temperature follows from its heat balance.
    ts = (
        p.air_surface_conductance * ti
        + p.window_conductance * t_out
        + p.surface_mass_conductance * tm
        + p.surface_fraction * radiant
    ) / (p.air_surface_conductance + p.window_conductance + p.surface_mass_conductance)
    expected = [
        (
            p.ventilation_conductance * (t_out - ti)
            + p.air_surface_conductance * (ts - ti)
            + p.convective_fraction * q_int
            + q_hea
        )
        / p.air_capacitance,
        (
            p.surface_mass_conductance * (ts - tm)
            + p.mass_outdoor_conductance * (t_out - tm)
            + p.ground_conductance * (building.ground_temperature - tm)
            + p.mass_fraction * radiant
        )
        / p.mass_capacitance,
    ]
    model = building.to_casadi()
    assert _evaluate(model, [ti, tm], [q_hea], [t_out, h_glo, q_int]) == pytest.approx(expected, rel=1e-10)


def test_coupling_conserves_energy(three_zones: dict[RCModelType, RCBuilding]) -> None:
    """With adiabatic zones (huge resistances), inter-zone flows only redistribute heat."""
    model = three_zones[RCModelType.r1c1].to_casadi()
    insulated = model.parameter_values(
        **{f"{zone}_{name}": 1e12 for zone in ("space_001", "space_002", "space_003") for name in ("Ria", "Rig")}
    )
    x = np.array([295.0, 290.0, 285.0])
    xdot = np.array(model.continuous_dynamics()(x, np.zeros(3), np.zeros(5), insulated)).reshape(-1)
    capacities = [insulated[model.parameter_names.index(f"{z}_Ci")] for z in ("space_001", "space_002", "space_003")]
    assert float(np.dot(capacities, xdot)) == pytest.approx(0, abs=1e-9)


def test_simulation_reaches_the_analytical_steady_state() -> None:
    building = _single_zone(RCModelType.r1c1)
    p = building.zones[0].parameters
    model = building.to_casadi()
    t_out, q_hea, steps = 273.15, 2000.0, 24 * 60
    states = model.simulate(np.full((steps, 1), q_hea), np.tile([t_out, 0.0, 0.0], (steps, 1)), 3600)
    ria, rig = p.indoor_outdoor_resistance, p.ground_resistance  # type: ignore[union-attr]
    steady_state = (t_out / ria + building.ground_temperature / rig + q_hea) / (1 / ria + 1 / rig)
    assert states[-1, 0] == pytest.approx(steady_state, abs=1e-3)


def test_stable_substeps_follow_the_fastest_dynamics() -> None:
    model = _single_zone(RCModelType.r4c3).to_casadi()
    tau = model.fastest_time_constant()
    assert model.stable_substeps(0.1 * tau) == 1
    assert model.stable_substeps(10 * tau) == 4


def _forecast(n_steps: int, n_zones: int) -> Forecast:
    hour = np.arange(n_steps) % 24
    occupied = (hour >= 7) & (hour < 22)
    return Forecast(
        outdoor_temperature=KELVIN + 2 + 5 * np.sin(2 * np.pi * (hour - 9) / 24),
        solar_irradiance=np.clip(600 * np.sin(np.pi * (hour - 7) / 10), 0, None) * (hour <= 17),
        internal_gains=np.tile(np.where(occupied, 300.0, 100.0).reshape(-1, 1), (1, n_zones)),
        lower_temperature=np.where(occupied, KELVIN + 20, KELVIN + 16),
        upper_temperature=np.where(occupied, KELVIN + 24, KELVIN + 26),
        price=np.where((hour >= 17) & (hour < 21), 0.4, 0.2),
    )


@pytest.mark.parametrize("model_type", list(RCModelType))
def test_mpc_keeps_comfort_with_bounded_power(
    three_zones: dict[RCModelType, RCBuilding], model_type: RCModelType
) -> None:
    model = three_zones[model_type].to_casadi()
    settings = MPCSettings(horizon=24, max_heating_power={"space_001": 6000, "space_002": 5000, "space_003": 4000})
    controller = ModelPredictiveController(model, settings)
    forecast = _forecast(24, 3)
    solution = controller.solve(forecast)
    assert solution.success, solution.status
    assert solution.heating_power.shape == (24, 3)
    assert np.all(solution.heating_power >= 0)
    assert np.all(solution.heating_power <= np.array([6000, 5000, 4000]))
    assert solution.indoor_temperature.shape == (25, 3)
    # The predicted trajectory is consistent with the model.
    predicted = model.simulate(solution.heating_power, forecast.disturbances(model, 24).T, settings.time_step)
    assert predicted == pytest.approx(solution.states, abs=1e-5)
    # Comfort is met once the controller had time to heat up the building.
    assert np.max(solution.comfort_violation[8:]) < 0.1


def test_mpc_does_not_heat_when_free_floating_is_comfortable() -> None:
    model = _single_zone(RCModelType.r3c2).to_casadi()
    controller = ModelPredictiveController(model, MPCSettings(horizon=12))
    forecast = Forecast(
        outdoor_temperature=np.full(12, KELVIN + 21),
        solar_irradiance=np.zeros(12),
        internal_gains=np.zeros(12),
        lower_temperature=np.full(12, KELVIN + 18),
        upper_temperature=np.full(12, KELVIN + 26),
        price=np.full(12, 0.3),
    )
    solution = controller.solve(forecast)
    assert solution.success
    assert solution.heating_power == pytest.approx(np.zeros((12, 1)), abs=1e-3)
    assert solution.energy_cost == pytest.approx(0, abs=1e-5)


def test_closed_loop(three_zones: dict[RCModelType, RCBuilding]) -> None:
    model = three_zones[RCModelType.r3c2].to_casadi()
    controller = ModelPredictiveController(model, MPCSettings(horizon=12))
    result = run_closed_loop(controller, _forecast(24, 3), n_steps=12)
    assert result.states.shape == (13, model.n_states)
    assert result.heating_power.shape == (12, 3)
    assert result.energy_cost > 0
    indoor = result.states[:, controller.indoor_indices]
    assert np.all(indoor[9:] > KELVIN + 19.9)


def test_forecast_too_short() -> None:
    model = _single_zone(RCModelType.r1c1).to_casadi()
    controller = ModelPredictiveController(model, MPCSettings(horizon=24))
    with pytest.raises(ValueError, match="shorter than the horizon"):
        controller.solve(_forecast(12, 1))
