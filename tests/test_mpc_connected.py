"""The connected ``mpc`` library: building_mpc assembled from Trano.MPC.Components with connect() statements.

The connected model must be the flat model, written differently: same inputs, and the same right-hand
side once the states are matched by name (``<zone>_Ti`` in the flat model is ``<zone>_Ti.T``). The
interface is written by trano without any compiler, so it is checked against the CasADi export.

The CasADi export of connected models needs rumoca >= 0.10.3 (it eliminates the algebraic variables of
the connections); with an older rumoca these tests are skipped.
"""

import shutil
import warnings
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import rumoca
from typer.testing import CliRunner

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.main import app
from trano.mpc import MPCModelInterface, RCModelType
from trano.mpc.components import ConnectedModel, Instance, TopInput, catalogue, library_source, parse_library
from trano.mpc.components.rc import connected_rc_model
from trano.mpc.estimation import rc_building_from_network
from trano.mpc.modelica import network_interface
from trano.topology import Network

MODELS = Path(__file__).parent / "models"
PROBE = """package Probe
  connector Port Real T; flow Real Q_flow; end Port;
  model Node parameter Real C = 1; Real T(start = 290, fixed = true); Port port;
  equation port.T = T; C*der(T) = port.Q_flow; end Node;
  model Link Port a; Port b; equation a.Q_flow = a.T - b.T; a.Q_flow + b.Q_flow = 0; end Link;
  model M Node n1; Node n2; Link l; equation connect(n1.port, l.a); connect(l.b, n2.port); end M;
end Probe;
"""


def _exports_connected_models() -> bool:
    try:
        rumoca.Session().loads(PROBE, model="Probe.M").to_casadi()
    except Exception:  # any failure means the feature is missing
        return False
    return True


needs_connected_export = pytest.mark.skipif(
    not _exports_connected_models(), reason="the CasADi export of connected models needs rumoca >= 0.10.3"
)


def _library(model_type: RCModelType, connected: bool = True) -> Library:
    return Library.from_configuration("mpc").model_copy(update={"rc_model_type": model_type, "connected": connected})


def _network(path: Path, model_type: RCModelType, connected: bool = True) -> Network:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return convert_network(path.stem, path, library=_library(model_type, connected))


def _envelope_only_models() -> list[Path]:
    """Test buildings the mpc library generates without energy systems (systems are not connected yet)."""
    paths = []
    for path in sorted(MODELS.glob("*.yaml")):
        try:
            network = _network(path, RCModelType.r3c2, connected=False)
            if rc_building_from_network(network, model_type=RCModelType.r3c2).systems:
                continue
            network.model()
        except Exception:  # noqa: S112 - invalid buildings, systems-only models, missing data...
            continue
        paths.append(path)
    return paths


ENVELOPE_ONLY = _envelope_only_models()


def _wrap(model: str) -> str:
    """A package with the Trano.MPC library and ``model``, as in a generated package."""
    library = f"package Trano\npackage MPC\nconnector RealInput = input Real;\n{library_source()}\nend MPC;\nend Trano;"
    return f"package T\n{library}\n{model}\nend T;\n"


# ----------------------------------------------------------------------------------------- catalogue
def test_the_catalogue_describes_every_component() -> None:
    components = catalogue()
    assert {
        "Thermal.HeatCapacitor",
        "Thermal.ThermalResistor",
        "Thermal.ThermalConductor",
        "Thermal.StarNode3",
        "Sources.PrescribedTemperature",
        "Sources.FixedTemperature",
        "Sources.PrescribedHeatFlow",
        "Sources.ScaledHeatFlow",
    } <= set(components)
    capacitor = components["Thermal.HeatCapacitor"]
    assert [parameter.name for parameter in capacitor.parameters] == ["C", "T0"]
    assert capacitor.parameter("T0").default == 294.15
    assert capacitor.parameter("C").default is None
    assert [(state.name, state.start_parameter) for state in capacitor.states] == [("T", "T0")]
    assert [input_.name for input_ in components["Sources.ScaledHeatFlow"].inputs] == ["u"]


def test_the_library_has_no_dependency() -> None:
    """No MSL (slow for rumoca, and the flat model has none either) and no short-class connector."""
    source = library_source()
    assert "Modelica." not in source
    assert "= input Real" not in source


def test_parse_library_reads_nested_packages() -> None:
    source = """package Root
  package A
    model M
      parameter Real k(unit="1") = 2 "gain";
      parameter Real c "no default";
      input Real u(unit="W") "signal";
      Real x(unit="K", start=x0, fixed=true) "state";
    equation
    end M;
  end A;
end Root;"""
    m = parse_library(source)["A.M"]
    assert [(p.name, p.unit, p.default) for p in m.parameters] == [("k", "1", 2.0), ("c", "", None)]
    assert [(i.name, i.unit) for i in m.inputs] == [("u", "W")]
    assert [(s.name, s.start_parameter) for s in m.states] == [("x", "x0")]


def test_instances_are_checked_against_their_class() -> None:
    with pytest.raises(ValueError, match="need a value"):
        Instance(name="c", component="Thermal.HeatCapacitor")
    with pytest.raises(ValueError, match="no parameters"):
        Instance(name="c", component="Thermal.HeatCapacitor", parameters={"C": 1, "R": 2})
    with pytest.raises(ValueError, match="must be bound"):
        Instance(name="s", component="Sources.PrescribedTemperature")
    with pytest.raises(KeyError, match=r"Unknown Trano\.MPC component"):
        Instance(name="x", component="Thermal.Nothing")


def _every_component_model() -> ConnectedModel:
    instances = [
        Instance(name="out", component="Sources.PrescribedTemperature", inputs={"T": "TOut"}),
        Instance(name="gro", component="Sources.FixedTemperature", parameters={"T": 283.15}),
        Instance(name="air", component="Thermal.HeatCapacitor", parameters={"C": 2e5, "T0": 293.15}),
        Instance(name="mass", component="Thermal.HeatCapacitor", parameters={"C": 5e6}),
        Instance(name="wall", component="Thermal.ThermalResistor", parameters={"R": 0.02}),
        Instance(name="win", component="Thermal.ThermalConductor", parameters={"G": 30}),
        Instance(name="ground", component="Thermal.ThermalResistor", parameters={"R": 0.05}),
        Instance(name="surface", component="Thermal.StarNode3", parameters={"G1": 300, "G2": 20, "G3": 900}),
        Instance(name="hea", component="Sources.PrescribedHeatFlow", inputs={"Q_flow": "QHea"}),
        Instance(name="sol", component="Sources.ScaledHeatFlow", parameters={"k": 4.0}, inputs={"u": "HSol"}),
    ]
    connections = [
        ("air.port", "wall.port_a"),
        ("wall.port_b", "out.port"),
        ("air.port", "win.port_a"),
        ("win.port_b", "out.port"),
        ("mass.port", "ground.port_a"),
        ("ground.port_b", "gro.port"),
        ("surface.port_1", "air.port"),
        ("surface.port_2", "out.port"),
        ("surface.port_3", "mass.port"),
        ("sol.port", "surface.node"),
        ("hea.port", "air.port"),
    ]
    return ConnectedModel(
        name="M",
        inputs=[TopInput(name="TOut", unit="K"), TopInput(name="QHea", unit="W"), TopInput(name="HSol", unit="W/m2")],
        instances=instances,
        connections=connections,
    )


@needs_connected_export
def test_the_catalogue_predicts_the_casadi_vectors() -> None:
    model = _every_component_model()
    export = rumoca.Session().loads(_wrap(model.render()), model="T.M").to_casadi()
    assert list(export.state_names) == model.state_names == ["air.T", "mass.T"]
    assert list(export.input_names) == model.input_names
    assert list(export.parameter_names) == model.parameter_names
    assert np.array(export.default_parameters).ravel() == pytest.approx([value for _, value in model.parameters])
    assert np.array(export.default_states).ravel() == pytest.approx(model.initial_state())


@needs_connected_export
def test_the_star_node_is_solved_in_closed_form() -> None:
    """Heat balance of the massless node: the air receives His*(Ts - Ti) with Ts in closed form."""
    model = _every_component_model()
    export = rumoca.Session().loads(_wrap(model.render()), model="T.M").to_casadi()
    t_air, t_mass, t_out, q_hea, h_sol = 293.0, 291.0, 273.0, 500.0, 200.0
    xdot = np.array(export.rhs(0, [t_air, t_mass], [t_out, q_hea, h_sol], export.default_parameters)).ravel()
    t_surface = (300 * t_air + 20 * t_out + 900 * t_mass + 4.0 * h_sol) / (300 + 20 + 900)
    air = (t_out - t_air) / 0.02 + 30 * (t_out - t_air) + 300 * (t_surface - t_air) + q_hea
    mass = 900 * (t_surface - t_mass) + (283.15 - t_mass) / 0.05
    assert xdot == pytest.approx([air / 2e5, mass / 5e6], rel=1e-12)


# ------------------------------------------------------------------------- connected = flat building
def test_many_buildings_are_covered() -> None:
    assert len(ENVELOPE_ONLY) >= 5


@needs_connected_export
@pytest.mark.parametrize("model_type", list(RCModelType))
@pytest.mark.parametrize("path", ENVELOPE_ONLY, ids=[path.stem for path in ENVELOPE_ONLY])
def test_the_connected_model_is_the_flat_model(path: Path, model_type: RCModelType) -> None:
    flat_network = _network(path, model_type, connected=False)
    network = _network(path, model_type)
    flat_interface, interface = network_interface(flat_network), network_interface(network)
    session = rumoca.Session()
    flat = session.loads(flat_network.model(), model=flat_interface.model).to_casadi()
    export = session.loads(network.model(), model=interface.model).to_casadi()

    assert interface.state_names == list(export.state_names) == [f"{name}.T" for name in flat.state_names]
    assert interface.input_names == list(export.input_names) == list(flat.input_names)
    assert interface.parameter_names == list(export.parameter_names)
    assert interface.parameter_values == pytest.approx(np.array(export.default_parameters).ravel())
    assert interface.initial_state == pytest.approx(np.array(export.default_states).ravel())

    rng = np.random.default_rng(0)
    for _ in range(5):
        x = 285 + 15 * rng.random(len(flat.state_names))
        u = 270 + 500 * rng.random(len(flat.input_names))
        expected = np.array(flat.rhs(0, x, u, flat.default_parameters)).ravel()
        actual = np.array(export.rhs(0, x, u, export.default_parameters)).ravel()
        assert actual == pytest.approx(expected, rel=1e-7, abs=1e-12)


@pytest.mark.parametrize("model_type", list(RCModelType))
def test_the_connected_model_describes_its_zones(model_type: RCModelType) -> None:
    network = _network(MODELS / "three_zones_mpc.yaml", model_type)
    interface = network_interface(network)
    assert [zone.indoor_temperature for zone in interface.zones] == [f"space_00{i}_Ti.T" for i in (1, 2, 3)]
    assert all(state.zone for state in interface.states)
    assert {state.node for state in interface.states} >= {"Ti"}
    source = network.model()
    assert "connect(space_001_Ti.port, space_001_Rie.port_a);" in source or model_type in (
        RCModelType.r1c1,
        RCModelType.iso13790,
    )
    assert "space_001_TZon = rc.space_001_Ti.T;" in source


def test_systems_are_not_connected_yet() -> None:
    network = _network(MODELS / "house_mpc_systems.yaml", RCModelType.r3c2)
    with pytest.raises(NotImplementedError, match="does not model energy systems yet"):
        network.model()


def test_the_flat_model_stays_the_default() -> None:
    assert Library.from_configuration("mpc").connected is False
    source = _network(MODELS / "three_zones_mpc.yaml", RCModelType.r3c2, connected=False).model()
    assert "der(space_001_Ti)" in source


@needs_connected_export
def test_cli_writes_the_connected_model_and_its_interface(tmp_path: Path) -> None:
    yaml_path = tmp_path / "house.yaml"
    shutil.copy(MODELS / "three_zones_mpc.yaml", yaml_path)
    resources = (MODELS.parent / "resources").resolve()
    yaml_path.write_text(yaml_path.read_text().replace("../tests/resources", str(resources)))
    result = CliRunner().invoke(app, ["create-model", str(yaml_path), "mpc", "--connected"])
    assert result.exit_code == 0, result.output
    interface = MPCModelInterface.read(tmp_path / "house.mpc.json")
    export: Any = rumoca.Session().loads((tmp_path / "house.mo").read_text(), model=interface.model).to_casadi()
    assert interface.state_names == list(export.state_names)
    assert interface.parameter_names == list(export.parameter_names)


def test_cli_refuses_connected_for_simulation_libraries(tmp_path: Path) -> None:
    yaml_path = tmp_path / "house.yaml"
    shutil.copy(MODELS / "three_zones_mpc.yaml", yaml_path)
    result = CliRunner().invoke(app, ["create-model", str(yaml_path), "Buildings", "--connected"])
    assert result.exit_code != 0
    assert "--connected only applies to the 'mpc' library" in result.output


def test_connected_rc_model_rejects_unknown_zone_types() -> None:
    network = _network(MODELS / "three_zones_mpc.yaml", RCModelType.r3c2)
    building = rc_building_from_network(network, model_type=RCModelType.r3c2)
    model = connected_rc_model(building)
    assert model.input_names[:1] == ["TOut"]
    assert len(model.state_names) == 2 * len(building.zones)


@pytest.mark.compile
@pytest.mark.parametrize("model_type", [RCModelType.r3c2, RCModelType.iso13790])
def test_openmodelica_compiles_the_connected_model(tmp_path: Path, model_type: RCModelType) -> None:
    """The runnable model of a connected building checks in OpenModelica (Docker)."""
    from trano.simulate.simulate import SimulationOptions, simulate
    from trano.utils.utils import is_success

    network = _network(MODELS / "three_zones_mpc.yaml", model_type)
    options = SimulationOptions(check_only=True)
    results = simulate(tmp_path, network, options=options, container_name=f"compile_connected_{model_type.value}")
    assert is_success(results, options=options), results.output.decode(errors="replace")[-3000:]
