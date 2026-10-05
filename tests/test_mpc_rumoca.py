"""The models of the ``mpc`` library must stay translatable into a CasADi ODE (MPC ready).

rumoca compiles the Modelica source into an explicit ODE ``xdot = rhs(t, x, u, p)`` built with
CasADi. It only accepts flat models without algebraic variables, events or functions, which is
exactly the contract of the ``mpc`` library. These tests guard that contract.
"""

from pathlib import Path
from typing import Any

import numpy as np
import pytest
import rumoca

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.mpc import ISO13790Parameters, R3C2Parameters, RCBuilding, RCModelType, RCZone, SolarAperture
from trano.mpc.building import Orientation
from trano.mpc.estimation import reference_zone_parameters

MODELS = Path(__file__).parent / "models"
SOUTH = Orientation(azimuth=0, tilt=90)


def to_casadi(source: str, model: str) -> Any:  # noqa: ANN401
    compiled = rumoca.Session().loads(source, model=model)
    assert "0 algebraic" in compiled.summary(), compiled.summary()
    return compiled.to_casadi()


def mpc_library(model_type: RCModelType) -> Library:
    return Library.from_configuration("mpc").model_copy(update={"rc_model_type": model_type})


def _convertible_models() -> list[Path]:
    """Every test building that the ``mpc`` library can generate."""
    paths = []
    for path in sorted(MODELS.glob("*.yaml")):
        try:
            convert_network(path.stem, path, library=Library.from_configuration("mpc")).model()
        except Exception:  # noqa: S112 - invalid buildings, systems-only models, missing data...
            continue
        paths.append(path)
    return paths


CONVERTIBLE_MODELS = _convertible_models()


def test_many_buildings_are_covered() -> None:
    assert len(CONVERTIBLE_MODELS) >= 10


@pytest.mark.parametrize("model_type", list(RCModelType))
@pytest.mark.parametrize("path", CONVERTIBLE_MODELS, ids=[path.stem for path in CONVERTIBLE_MODELS])
def test_generated_mpc_model_translates_to_casadi(path: Path, model_type: RCModelType) -> None:
    network = convert_network(path.stem, path, library=mpc_library(model_type))
    export = to_casadi(network.model(), f"{path.stem}.building_mpc")
    zones = [name.removesuffix("_QHea") for name in export.input_names if name.endswith("_QHea")]
    assert zones
    states_per_zone = {RCModelType.r1c1: 1, RCModelType.r4c3: 3}.get(model_type, 2)
    assert len(export.state_names) == states_per_zone * len(zones)
    assert "TOut" in export.input_names
    for zone in zones:
        assert f"{zone}_Ti" in export.state_names
        assert f"{zone}_QInt" in export.input_names
    # The exported right-hand side is a differentiable CasADi function.
    xdot = export.rhs(0, export.default_states, np.zeros(len(export.input_names)), export.default_parameters)
    assert np.all(np.isfinite(np.array(xdot)))


@pytest.mark.parametrize("model_type", list(RCModelType))
def test_library_zone_models_translate_to_casadi(model_type: RCModelType) -> None:
    source = convert_network("house", MODELS / "three_zones_mpc.yaml", library=mpc_library(model_type)).model()
    export = to_casadi(source, f"house.Trano.MPC.Zones.{model_type.value}")
    assert export.input_names == ["TOut", "HSol", "QInt", "QHea"]


def _single_zone(model_type: RCModelType) -> RCBuilding:
    zone = RCZone(
        name="zone",
        parameters=reference_zone_parameters(model_type),
        solar_apertures=[SolarAperture(orientation=SOUTH, window=3.0, opaque=0.5)],
    )
    return RCBuilding(zones=[zone])


def _rhs(building: RCBuilding, x: list[float], inputs: dict[str, float]) -> np.ndarray:
    export = to_casadi(building.to_modelica(), "TranoRC.building_mpc")
    u = [inputs[name.removeprefix("zone_")] for name in export.input_names]
    return np.array(export.rhs(0, x, u, export.default_parameters)).reshape(-1)


INPUTS = {"TOut": 273.0, "HSol_azi0_til90": 300.0, "QInt": 200.0, "QHea": 1500.0}


def test_r3c2_equations_match_the_network() -> None:
    building = _single_zone(RCModelType.r3c2)
    p = building.zones[0].parameters
    assert isinstance(p, R3C2Parameters)
    ti, te, t_out, h_sol, q_int, q_hea = 293.0, 290.0, *INPUTS.values()
    expected = [
        (
            (te - ti) / p.indoor_envelope_resistance
            + (t_out - ti) / p.indoor_outdoor_resistance
            + 3.0 * h_sol
            + q_int
            + q_hea
        )
        / p.air_capacitance,
        (
            (ti - te) / p.indoor_envelope_resistance
            + (t_out - te) / p.envelope_outdoor_resistance
            + (building.ground_temperature - te) / p.envelope_ground_resistance
            + 0.5 * h_sol
        )
        / p.envelope_capacitance,
    ]
    assert _rhs(building, [ti, te], INPUTS) == pytest.approx(expected, rel=1e-10)


def test_iso13790_equations_match_the_standard() -> None:
    building = _single_zone(RCModelType.iso13790)
    p = building.zones[0].parameters
    assert isinstance(p, ISO13790Parameters)
    ti, tm, t_out, h_sol, q_int, q_hea = 293.0, 291.0, *INPUTS.values()
    radiant = (1 - p.convective_fraction) * q_int + (3.0 + 0.5) * h_sol
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
    assert _rhs(building, [ti, tm], INPUTS) == pytest.approx(expected, rel=1e-10)


def test_coupling_conserves_energy() -> None:
    """With adiabatic zones, the inter-zone heat flows only redistribute heat."""
    network = convert_network("house", MODELS / "three_zones_mpc.yaml", library=mpc_library(RCModelType.r1c1))
    export = to_casadi(network.model(), "house.building_mpc")
    names = list(export.parameter_names)
    parameters = np.array(export.default_parameters, dtype=float)
    zones = ("space_001", "space_002", "space_003")
    for zone in zones:
        for name in ("Ria", "Rig"):
            parameters[names.index(f"{zone}_{name}")] = 1e12
    xdot = np.array(export.rhs(0, [295.0, 290.0, 285.0], np.zeros(len(export.input_names)), parameters))
    capacities = [parameters[names.index(f"{zone}_Ci")] for zone in zones]
    assert float(np.dot(capacities, xdot.reshape(-1))) == pytest.approx(0, abs=1e-9)
