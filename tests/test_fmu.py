"""Hermetic tests of the FMU module: model description parsing, YAML to FMU name resolution and classification.

No Docker, no FMU runtime: the model description is the checked-in fixture
tests/fixtures/fmu/modelDescription.xml, an excerpt of the Buildings FMU of
examples/residential_after_renovation.yaml (its parameters and a few of its other variables).
"""

from pathlib import Path

import pytest
import yaml

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.fmu.buildings import BuildingsEnvelopeResolver
from trano.fmu.calibration import get_at, perturbed_value, set_at
from trano.fmu.export import FmuExportOptions, fmu_script
from trano.fmu.inventory import (
    ResolvedParameter,
    Status,
    Target,
    classify,
    parse_modelica_array,
    resolve_all,
)
from trano.fmu.model_description import parse_model_description, read_model_description
from trano.topology import Network

FIXTURES = Path(__file__).parent / "fixtures" / "fmu"
EXAMPLE = Path(__file__).parents[1] / "examples" / "residential_after_renovation.yaml"
MINIMAL_DESCRIPTION = """<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="2.0" modelName="house.building" guid="{abc}" generationTool="OpenModelica">
  <CoSimulation modelIdentifier="house_building"/>
  <ModelVariables>
    <ScalarVariable name="building.emission1.radiator.Q_flow_nominal" valueReference="1" causality="parameter"
        variability="fixed" initial="exact" description="Nominal heating power">
      <Real start="3900.0" unit="W"/>
    </ScalarVariable>
    <ScalarVariable name="building.emission1.radiator.UA_nominal" valueReference="2" causality="calculatedParameter"
        variability="fixed" initial="calculated">
      <Real unit="W/K"/>
    </ScalarVariable>
    <ScalarVariable name="building.envelope1.zone.nConExt" valueReference="3" causality="parameter"
        variability="constant" initial="exact">
      <Integer start="2"/>
    </ScalarVariable>
    <ScalarVariable name="building.envelope1.zone.useInf" valueReference="4" causality="parameter"
        variability="fixed" initial="exact">
      <Boolean start="true"/>
    </ScalarVariable>
    <ScalarVariable name="building.bus1.dataBus.TZonLiving_001" valueReference="5" causality="local"
        variability="continuous">
      <Real unit="K"/>
    </ScalarVariable>
    <ScalarVariable name="building.envelope1.living_001.datConExtWin[1].A" valueReference="6" causality="parameter"
        variability="tunable" initial="exact">
      <Real start="19.24" unit="m2"/>
    </ScalarVariable>
  </ModelVariables>
</fmiModelDescription>
"""


@pytest.fixture(scope="module")
def example_network() -> Network:
    network = convert_network(EXAMPLE.stem, EXAMPLE, library=Library.from_configuration("Buildings"))
    network.model()
    return network


@pytest.fixture(scope="module")
def example_data() -> dict:
    return yaml.safe_load(EXAMPLE.read_text())


@pytest.fixture(scope="module")
def resolved(example_network: Network, example_data: dict) -> dict[str, ResolvedParameter]:
    return {r.yaml_path: r for r in resolve_all(example_network, example_data, BuildingsEnvelopeResolver())}


def test_parse_model_description_reads_the_scalar_variables() -> None:
    description = parse_model_description(MINIMAL_DESCRIPTION)

    assert description.fmi_version == "2.0"
    assert description.co_simulation and not description.model_exchange
    assert [variable.name for variable in description.parameters()] == [
        "building.emission1.radiator.Q_flow_nominal",
        "building.envelope1.zone.useInf",
        "building.envelope1.living_001.datConExtWin[1].A",
    ]
    power = description.get("building.emission1.radiator.Q_flow_nominal")
    assert power is not None
    assert (power.start_value(), power.unit, power.initial) == (3900.0, "W", "exact")
    boolean = description.get("building.envelope1.zone.useInf")
    assert boolean is not None and boolean.start_value() == 1.0
    assert not description.get("building.emission1.radiator.UA_nominal").settable  # type: ignore[union-attr]
    assert not description.get("building.envelope1.zone.nConExt").settable  # type: ignore[union-attr]
    assert description.matching("building.envelope1.living_001.datConExtWin")[0].variability == "tunable"


def test_read_model_description_from_an_xml_file(tmp_path: Path) -> None:
    path = tmp_path / "modelDescription.xml"
    path.write_text(MINIMAL_DESCRIPTION)

    assert read_model_description(path).model_name == "house.building"
    assert read_model_description(tmp_path).guid == "{abc}"


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("3600*{7, 23}", [25200.0, 82800.0]),
        ("{1,2}", [1.0, 2.0]),
        ("0.05/1000*{0.5,1}", [2.5e-05, 5e-05]),
        ("5000*{2,1}", [10000.0, 5000.0]),
        ("{true}", None),
        ("abc", None),
        ("[35; 70; 30]", None),
    ],
)
def test_parse_modelica_array(text: str, expected: list[float] | None) -> None:
    assert parse_modelica_array(text) == pytest.approx(expected) if expected else parse_modelica_array(text) is None


def test_classify_from_the_model_description() -> None:
    description = parse_model_description(MINIMAL_DESCRIPTION)
    direct = ResolvedParameter(
        yaml_path="spaces[LIVING:001].emissions[0].radiator.parameters.nominal_heating_power_positive_for_heating",
        group="Emission",
        value=3900,
        targets=(Target(path="building.emission1.radiator.Q_flow_nominal", value=3900),),
    )
    derived = direct.model_copy(
        update={
            "targets": (
                *direct.targets,
                Target(path="building.emission1.radiator.UA_nominal", value=100, transformation="UA = Q/dT"),
            )
        }
    )
    structural = ResolvedParameter(
        yaml_path="x", group="Geometry", value=2, targets=(Target(path="building.envelope1.zone.nConExt", value=2),)
    )
    absent = ResolvedParameter(
        yaml_path="y", group="Geometry", value=1, targets=(Target(path="building.gone", value=1),)
    )
    unused = ResolvedParameter(yaml_path="z", group="Ground", value=1, note="IDEAS only")

    assert classify(direct, description).status == Status.calibratable.value
    frozen = classify(derived, description)
    assert frozen.status == Status.frozen.value
    assert "calculatedParameter" in frozen.reason
    assert "constant" in classify(structural, description).reason
    assert classify(absent, description).status == Status.missing.value
    row = classify(unused, description)
    assert (row.status, row.reason) == (Status.not_used.value, "IDEAS only")
    assert classify(direct, description).fmu_names == "building.emission1.radiator.Q_flow_nominal"
    assert classify(direct, description).variability == "fixed"


def test_the_fmu_script_builds_a_co_simulation_fmu_with_cvode(example_network: Network) -> None:
    options = FmuExportOptions()
    script = fmu_script(example_network, "model.mo", options, model="model text")

    assert 'loadModel(Modelica, {"4.1.0"});' in script
    assert 'loadModel(IDEAS, {"3.0.0"});' in script
    assert "AixLib" not in script
    assert 'setCommandLineOptions("--fmiFlags=s:cvode");' in script
    assert (
        'buildModelFMU(residential_after_renovation.building, version="2.0", fmuType="cs", '
        'fileNamePrefix="residential_after_renovation", platforms={"static"}, includeResources=true);'
    ) in script
    assert FmuExportOptions(fmu_type="me").command_line_options == []


class TestResolution:
    """YAML field to Modelica parameter path, on the renovated house."""

    def test_system_parameters_follow_the_schema_mapping(self, resolved: dict[str, ResolvedParameter]) -> None:
        valve = resolved["spaces[LIVING:001].emissions[1].valve.parameters.m_flow_nominal"]
        assert valve.kind == "direct"
        assert valve.paths == ["building.emission1.valve_living_001.m_flow_nominal"]
        assert valve.targets[0].value == 0.093
        assert valve.unit == "kg/s"

        ahu = resolved["systems[5].air_handling_unit.parameters.heat_exchanger_effectiveness"]
        assert ahu.paths == ["building.ventilation1.ahu_001.eps"]
        duct = resolved["spaces[LIVING:001].ventilation_inlets[0].duct.parameters.nominal_mass_flow_rate"]
        assert duct.targets[0] == Target(
            path="building.ventilation1.duct_supply_living_001.m_flow_nominal", value=0.01333
        )

    def test_python_fan_out_is_derived(self, resolved: dict[str, ResolvedParameter]) -> None:
        power = resolved[
            "spaces[LIVING:001].emissions[0].radiator.parameters.nominal_heating_power_positive_for_heating"
        ]
        assert power.kind == "derived"
        assert power.paths == [
            "building.emission1.radiator_living_001.Q_flow_nominal",
            "building.emission1.radiator_living_001.mDry",
            "building.emission1.radiator_living_001.VWat",
        ]
        assert power.targets[1].value == pytest.approx(0.0263 * 3900)
        assert "0.0263" in (power.targets[1].transformation or "")

        n50 = resolved["spaces[LIVING:001].parameters.n50"]
        assert n50.kind == "derived"
        assert n50.targets == (
            Target(
                path="building.envelope1.living_001.ACH",
                value=0.15,
                transformation="ACH = self.n50 / self.n50_to_ach if self.n50 is not None else None",
            ),
        )
        heat_pump = resolved["systems[0].boiler.parameters.nominal_heating_power"]
        assert heat_pump.paths[:3] == [
            "building.production1.generator_001.Q_flow_nominal",
            "building.production1.generator_001.nominal_mass_flow_radiator_loop",
            "building.production1.generator_001.nominal_mass_flow_rate_boiler",
        ]
        assert heat_pump.paths[3:] == [
            "building.production1.generator_001.V_flow[1]",
            "building.production1.generator_001.V_flow[2]",
        ]

    def test_schedules_become_arrays(self, resolved: dict[str, ResolvedParameter]) -> None:
        schedule = resolved["spaces[LIVING:001].emissions[1].valve.control.emission_control.parameters.schedule"]
        assert schedule.group == "Controls"
        assert [(t.path.rsplit(".", 1)[-1], t.value) for t in schedule.targets] == [
            ("schedule[1]", 25200.0),
            ("schedule[2]", 82800.0),
        ]
        occupancy = resolved["spaces[LIVING:001].occupancy.parameters.occupancy"]
        assert occupancy.paths[0] == "building.envelope1.occupancy_1.occupancy[1]"
        assert len(occupancy.targets) == 4
        setpoint = resolved[
            "spaces[LIVING:001].emissions[1].valve.control.emission_control.parameters.temperature_heating_setpoint"
        ]
        assert setpoint.paths == ["building.emission1.emission_control_living_001.THeaSet"]

    def test_materials_fan_out_over_the_constructions_using_them(self, resolved: dict[str, ResolvedParameter]) -> None:
        plaster = resolved["material[PLASTER:001].thermal_conductivity"]
        assert plaster.paths == [
            "building.envelope1.external_wall_001.material[5].k",
            "building.envelope1.party_wall_001.material[2].k",
            "building.envelope1.internal_wall_001.material[1].k",
            "building.envelope1.internal_wall_001.material[3].k",
        ]
        assert all(target.value == 0.5 for target in plaster.targets)
        eps = resolved["material[EPS:001].density"]
        assert eps.paths == ["building.envelope1.external_wall_001.material[1].d"]
        thickness = resolved["constructions[ROOF:001].layers[0].thickness"]
        assert thickness.targets == (Target(path="building.envelope1.roof_001.material[1].x", value=0.22),)

    def test_glazing_fields(self, resolved: dict[str, ResolvedParameter]) -> None:
        assert resolved["glazings[WINDOW:001].u_value_frame"].paths == ["building.envelope1.window_001.UFra"]
        transmittance = resolved["glass_material[GLASS:001].solar_transmittance"]
        assert transmittance.paths == ["building.envelope1.window_001.glass[2].tauSol[1]"]
        low_e = resolved["glass_material[GLASS_LOWE_ROOM_SIDE:001].infrared_absorptivity_room_facing"]
        assert low_e.paths == ["building.envelope1.window_001.glass[1].absIR_b"]
        assert resolved["glass_material[GLASS:001].density"].kind == "not_used"
        argon = resolved["gas[ARGON:001].density"]
        assert argon.kind == "derived"
        assert [t.path.rsplit(".", 1)[-1] for t in argon.targets] == ["MM", "a_mu", "b_mu"] * 2
        assert resolved["gas[AIR:001].thermal_conductivity"].kind == "not_used"
        gap = resolved["glazings[WINDOW:001].layers[1].thickness"]
        assert gap.paths == ["building.envelope1.window_001.gas[1].x"]

    def test_zone_geometry_lands_in_the_construction_arrays(self, resolved: dict[str, ResolvedParameter]) -> None:
        wall = resolved["spaces[LIVING:001].external_boundaries.external_walls[0].surface"]
        assert wall.paths == ["building.envelope1.living_001.datConExtWin[1].A"]
        assert wall.kind == "derived"  # the gross area of the host wall of the window
        window = resolved["spaces[LIVING:001].external_boundaries.windows[0].surface"]
        assert window.paths == [
            "building.envelope1.living_001.datConExtWin[1].hWin",
            "building.envelope1.living_001.datConExtWin[1].wWin",
        ]
        assert window.targets[0].value == pytest.approx(6.5**0.5)
        frame = resolved["spaces[LIVING:001].external_boundaries.windows[0].frame_fraction"]
        assert frame.kind == "direct"
        assert frame.paths == ["building.envelope1.living_001.datConExtWin[1].fFra"]
        # the north wall hosts the window; the west wall and the roof are the opaque walls, in the YAML order
        west = resolved["spaces[BEDROOM_1:001].external_boundaries.external_walls[1].surface"]
        assert west.paths == ["building.envelope1.bedroom_1_001.datConExt[1].A"]
        roof = resolved["spaces[BEDROOM_1:001].external_boundaries.external_walls[2].surface"]
        assert roof.paths == ["building.envelope1.bedroom_1_001.datConExt[2].A"]
        assert roof.kind == "direct"
        floor = resolved["spaces[LIVING:001].external_boundaries.floor_on_grounds[0].surface"]
        assert floor.paths == ["building.envelope1.living_001.datConBou[1].A"]
        party = resolved["spaces[LIVING:001].external_boundaries.adiabatic_walls[0].surface"]
        assert party.paths == ["building.envelope1.living_001.datConBou[2].A"]
        internal = resolved["internal_walls[5].surface"]
        assert internal.paths == [
            "building.envelope1.living_001.surBou[5].A",
            "building.envelope1.kitchen_001.surBou[2].A",
        ]

    def test_ground_fields(self, resolved: dict[str, ResolvedParameter]) -> None:
        ground = resolved["spaces[LIVING:001].external_boundaries.floor_on_grounds[0].ground_temperature"]
        assert ground.targets == (Target(path="building.envelope1.flooronground_0.T", value=286.65),)
        for field in ("perimeter", "average_indoor_temperature"):
            unused = resolved[f"spaces[LIVING:001].external_boundaries.floor_on_grounds[0].{field}"]
            assert unused.kind == "not_used"
            assert "IDEAS" in (unused.note or "")

    def test_pv_angles_are_converted_to_radians(self, resolved: dict[str, ResolvedParameter]) -> None:
        tilt = resolved["solar[0].photovoltaic.parameters.tilt"]
        assert tilt.kind == "derived"
        assert tilt.targets[0].path == "building.solar1.pv_001.til"
        assert tilt.targets[0].value == pytest.approx(35 * 3.141592653589793 / 180, abs=1e-6)
        assert resolved["solar[0].photovoltaic.parameters.area"].paths == ["building.solar1.pv_001.A"]

    def test_every_numeric_field_of_the_example_is_resolved(self, resolved: dict[str, ResolvedParameter]) -> None:
        groups = {r.group for r in resolved.values()}
        assert groups >= {
            "Opaque materials",
            "Layers",
            "Glazing",
            "Space",
            "Ground",
            "Geometry",
            "Emission",
            "Controls",
            "Heat pump",
            "Hydronics",
            "Ventilation",
            "PV",
            "Occupancy",
        }
        assert not [r.yaml_path for r in resolved.values() if r.note and "not found" in r.note]


class TestPerturbation:
    def test_get_and_set_at_a_yaml_path(self, example_data: dict) -> None:
        path = "spaces[LIVING:001].emissions[0].radiator.parameters.nominal_heating_power_positive_for_heating"
        assert get_at(example_data, path) == 3900
        changed = set_at(example_data, path, 4680)
        assert get_at(changed, path) == 4680
        assert get_at(example_data, path) == 3900
        assert get_at(changed, "material[EPS:001].thermal_conductivity") == 0.032
        assert get_at(set_at(example_data, "internal_walls[5].surface", 1.0), "internal_walls[5].surface") == 1.0

    def test_perturbed_values_stay_physical(self) -> None:
        assert perturbed_value("a.thermal_conductivity", 0.032) == pytest.approx(0.0384)
        assert perturbed_value("a.frame_fraction", 0.25) == pytest.approx(0.25 / 1.2)
        assert perturbed_value("a.heat_exchanger_effectiveness", 0.8) == pytest.approx(0.8 / 1.2)
        assert perturbed_value("a.solar_transmittance", [0.83]) == pytest.approx([0.83 / 1.2])
        assert perturbed_value("a.schedule", "3600*{7, 23}") == "{21000, 69000}"
