"""Multilayer zones of the ``FastModel`` library (its default envelope): Buildings MixedAir physics as a CasADi ODE.

The year-long agreement with Buildings is checked by ``python -m validation.mpc``; these tests check the
assembly: the constructions of each zone, the shared walls between zones, the optics fits, the solar
distribution and the CasADi export (rumoca >= 0.10.3).
"""

import math
import warnings
from pathlib import Path

import numpy as np
import pytest
import rumoca
from typer.testing import CliRunner

from trano.data_models.conversion import convert_network
from trano.elements.envelope import BaseInternalElement
from trano.elements.library.library import Library
from trano.elements.space import Space
from trano.main import app
from trano.fast_model.interface import FastModelInterface
from trano.fast_model.components import Instance
from trano.fast_model.components.envelope import (
    INTERNAL_EMISSIVITY,
    Convection,
    _Surface,
    internal_side,
    layer_chain,
    multilayer_model,
    solar_distribution,
    window_optics,
    zone_elements,
)
from trano.fast_model.modelica import network_interface
from trano.topology import Network

MODELS = Path(__file__).parent / "models"
THREE_ZONES = MODELS / "three_zones_mpc.yaml"


def _exports_connected_models() -> bool:
    from tests.test_fast_model_connected import _exports_connected_models as probe

    return probe()


needs_connected_export = pytest.mark.skipif(
    not _exports_connected_models(), reason="the CasADi export of connected models needs rumoca >= 0.10.3"
)


def _network(path: Path = THREE_ZONES) -> Network:
    library = Library.from_configuration("FastModel")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return convert_network(path.stem, path, library=library)


def _spaces(network: Network) -> list[Space]:
    return [node for node in network.graph.nodes if isinstance(node, Space)]


# ------------------------------------------------------------------------------------- the catalogue
def test_array_and_integer_parameters_are_checked() -> None:
    Instance(name="rad", component="Thermal.StarNode", parameters={"n": 2, "G": [1.0, 2.0]})
    with pytest.raises(ValueError, match="Integer"):
        Instance(name="rad", component="Thermal.StarNode", parameters={"n": 2.5, "G": [1.0, 2.0]})
    with pytest.raises(ValueError, match="list of n values"):
        Instance(name="rad", component="Thermal.StarNode", parameters={"n": 2, "G": [1.0, 2.0, 3.0]})


# -------------------------------------------------------------------------------------- the physics
def test_layers_put_half_of_each_cell_on_its_two_nodes() -> None:
    network = _network()
    opaque, _ = zone_elements(_spaces(network)[0], network)
    construction = opaque[0].construction
    chain = layer_chain(construction, area=2.0, cells_per_layer=3)
    expected_capacity = sum(
        layer.material.density * layer.material.specific_heat_capacity * layer.thickness * 2.0
        for layer in construction.layers
    )
    expected_resistance = sum(
        layer.thickness / layer.material.thermal_conductivity / 2.0 for layer in construction.layers
    )
    assert len(chain.capacities) == 3 * len(construction.layers) + 1
    assert sum(chain.capacities) == pytest.approx(expected_capacity)
    assert sum(chain.resistances) == pytest.approx(expected_resistance)


def test_convection_follows_the_buildings_correlations() -> None:
    assert Convection.for_tilt(0.0) == Convection(c_pos=0.76, c_neg=1.51)  # ceiling
    assert Convection.for_tilt(math.pi) == Convection(c_pos=1.51, c_neg=0.76)  # floor
    assert Convection.for_tilt(math.pi / 2) == Convection(c_pos=1.3, c_neg=1.3)


def test_window_optics_reproduce_the_angular_properties() -> None:
    from trano.elements.glazing import INCIDENCE_ANGLES, N_ANGLES

    network = _network()
    _, windows = zone_elements(_spaces(network)[0], network)
    glazing = windows[0].glazing
    optics = window_optics(glazing)
    cosines = np.cos(INCIDENCE_ANGLES)
    fitted = np.polynomial.polynomial.polyval(cosines, [optics[f"tau{i}"] for i in range(5)])
    expected = np.asarray(glazing.properties.optics.transmittance)[:N_ANGLES]
    assert fitted == pytest.approx(expected, abs=0.015)
    assert len(optics["aDif"]) == len(np.asarray(glazing.properties.optics.absorptances))  # type: ignore[arg-type]


def test_solar_distribution_conserves_the_transmitted_radiation() -> None:
    surfaces = [
        _Surface(port="flo", area=20, emissivity=0.9, absorptance=0.6, is_floor=True),
        _Surface(port="wal", area=40, emissivity=0.9, absorptance=0.5),
        _Surface(port="win", area=6, emissivity=0.84, absorptance=0.05, transmittance=0.7),
    ]
    k_dir, k_dif = solar_distribution(surfaces)
    lost = 0.7 * 6 / sum((s.absorptance + s.transmittance) * s.area for s in surfaces)
    assert sum(k_dif) + lost == pytest.approx(1)
    assert k_dir[0] == pytest.approx(0.6)  # direct radiation reaches the floor first
    assert sum(k_dir[1:]) == pytest.approx(0.4 * (0.5 * 40 + 0.05 * 6) / (0.5 * 40 + 0.75 * 6))


# --------------------------------------------------------------------------------------- the zones
def test_internal_walls_are_shared_by_their_two_zones() -> None:
    network = _network()
    internal = [node for node in network.graph.nodes if isinstance(node, BaseInternalElement)]
    assert internal
    for element in internal:
        spaces = [space for space in _spaces(network) if element in network.graph.succ[space]]
        assert sorted(internal_side(element, space, network) for space in spaces) == ["a", "b"]
    model = multilayer_model(network)
    names = {instance.name for instance in model.instances}
    for element in internal:
        layers = len(element.construction.layers)  # type: ignore[union-attr]
        assert {f"{element.name}_T{i}" for i in range(layers + 1)} <= names
    surfaces = [instance for instance in model.instances if instance.name.endswith(("_a_conInt", "_b_conInt"))]
    assert len(surfaces) == 2 * len(internal)
    stars = [instance for instance in model.instances if instance.component == "Thermal.StarNode"]
    assert INTERNAL_EMISSIVITY > 0
    assert len(stars) == len(_spaces(network))


def test_every_exterior_orientation_has_its_irradiance_inputs() -> None:
    network = _network()
    model = multilayer_model(network)
    used = {value for instance in model.instances for value in instance.inputs.values()}
    referenced = {name for value in used for name in value.replace("+", " ").split()}
    top_level = {name for name in referenced if "." not in name}  # the others are outputs of the instances
    assert top_level <= set(model.input_names)
    assert {name.split(".")[0] for name in referenced - top_level} <= {i.name for i in model.instances}


@needs_connected_export
def test_the_export_matches_the_interface_and_is_at_rest_in_equilibrium() -> None:
    """Every node at the outdoor (= sky = ground) temperature, no sun and no gains: nothing moves."""
    network = _network()
    interface = network_interface(network)
    export = rumoca.Session().loads(network.model(), model=interface.model).to_casadi()
    assert interface.state_names == list(export.state_names)
    assert interface.input_names == list(export.input_names)
    assert interface.parameter_names == list(export.parameter_names)
    assert interface.parameter_values == pytest.approx(np.array(export.default_parameters).ravel())

    parameters = dict(zip(interface.parameter_names, interface.parameter_values, strict=True))
    grounds = {value for name, value in parameters.items() if name.endswith("_gro.T")}
    assert len(grounds) == 1
    temperature = grounds.pop()
    inputs = {"TOut": temperature, "TBlaSky": temperature, "winSpe": 3.0, "winDir": 1.0}
    u = [inputs.get(name, 0.5 if name.startswith("cosInc_") else 0.0) for name in interface.input_names]
    x = np.full(len(interface.state_names), temperature)
    xdot = np.array(export.rhs(0, x, u, export.default_parameters)).ravel()
    assert np.abs(xdot).max() < 1e-12

    # heating the first zone warms its air first
    u[interface.input_names.index(f"{interface.zones[0].name}_QHea")] = 1000.0
    xdot = np.array(export.rhs(0, x, u, export.default_parameters)).ravel()
    assert xdot[interface.state_names.index(interface.zones[0].indoor_temperature)] > 0


def test_cli_writes_the_multilayer_model(tmp_path: Path) -> None:
    yaml_path = tmp_path / "house.yaml"
    resources = (MODELS.parent / "resources").resolve()
    yaml_path.write_text(THREE_ZONES.read_text().replace("../tests/resources", str(resources)))
    result = CliRunner().invoke(app, ["create-model", str(yaml_path), "FastModel", "--envelope", "multilayer"])
    assert result.exit_code == 0, result.output
    interface = FastModelInterface.read(tmp_path / "house.mpc.json")
    assert [zone.indoor_temperature for zone in interface.zones] == [f"space_00{i}_air.T" for i in (1, 2, 3)]
    assert "Envelope.SolarDistribution" in (tmp_path / "house.mo").read_text()


@pytest.mark.compile
def test_openmodelica_compiles_the_multilayer_model(tmp_path: Path) -> None:
    """The runnable model (Buildings weather and Perez irradiance feeding the multilayer zones) checks in Docker."""
    from trano.simulate.simulate import SimulationOptions, simulate
    from trano.utils.utils import is_success

    network = _network()
    options = SimulationOptions(check_only=True)
    results = simulate(tmp_path, network, options=options, container_name="compile_multilayer")
    assert is_success(results, options=options), results.output.decode(errors="replace")[-3000:]
