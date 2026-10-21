"""Compile once, change parameters at run time: the parts that need no Docker.

The init file fixture (tests/data/runtime) is a subset of the ``_init.xml`` OpenModelica 1.26 writes for
examples/residential_after_renovation.yaml with Buildings 13: free, evaluated, bound and alias variables.
"""

import math
from pathlib import Path

import numpy as np
import pytest

from trano.runtime.adapters import get_adapter
from trano.runtime.adapters.buildings import number_of_states
from trano.runtime.checks import (
    IMPACT_THRESHOLD,
    ImpactResult,
    OutputSeries,
    check_readback,
    parameter_filter,
    variable_filter,
)
from trano.runtime.expressions import NotNumericError, evaluate, modelica_array, scalarize
from trano.runtime.init_xml import ParameterKind, read_default_experiment, read_model_variables
from trano.runtime.inventory import (
    InventoryRow,
    Priority,
    Status,
    free_overrides,
    group_of,
    perturbation,
    read_inventory,
    static_row,
    write_inventory,
)
from trano.runtime.model import CompiledModel, RunResult, build_script, verify_run
from trano.runtime.overrides import (
    OverrideIssue,
    OverrideMessage,
    experiment_flags,
    format_value,
    ignored_overrides,
    output_flag,
    parse_output_values,
    read_override_file,
    verify_overrides,
    parse_override_messages,
    write_override_file,
)
from trano.runtime.resolution import (
    NotRuntimeChangeableError,
    ParameterChange,
    Resolution,
    ResolutionKind,
    SpecificationResolver,
    diff,
)
from trano.runtime.specification import (
    YamlField,
    get_value,
    load_description,
    numeric_fields,
    resolve_keys,
    with_values,
)
from trano.simulate.simulate import SimulationOptions

EXAMPLE = Path(__file__).parents[1].joinpath("examples", "residential_after_renovation.yaml")
INIT_XML = Path(__file__).parent.joinpath("data", "runtime", "residential_after_renovation.building_init.xml")
LIBRARIES = ["Buildings"]  # IDEAS gets its adapter later
EPS_K = "material[EPS:001].thermal_conductivity"
LIVING = "building.envelope1.living_001"

# Lines of the log of the executable (OpenModelica 1.26.9) for overrides it does not apply.
LOG = """LOG_STDOUT        | warning | You are overriding variable: stopTime=7200 again with stopTime=86400.
LOG_STDOUT        | warning | It is not possible to override the following quantity: building.envelope1.living_001.V
|                 | |       | It seems to be structural, final, protected or evaluated or has a non-constant binding.
LOG_STDOUT        | warning | simulation_input_xml.c: override variable name not found in model: building.does.not.exist
LOG_SUCCESS       | info    | The simulation finished successfully.
"""


@pytest.fixture(scope="module")
def variables() -> dict:
    return read_model_variables(INIT_XML)


@pytest.fixture(scope="module", params=LIBRARIES)
def resolver(request: pytest.FixtureRequest) -> SpecificationResolver:
    return SpecificationResolver.from_file(EXAMPLE, request.param)


# -- init file ----------------------------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("name", "kind"),
    [
        (f"{LIVING}.ACH", ParameterKind.free),
        (f"{LIVING}.datConExtWin[1].layers.material[1].k", ParameterKind.free),
        ("building.emission1.radiator_living_001.T_a_nominal", ParameterKind.evaluated),
        ("building.envelope1.external_wall_001.material[1].nSta", ParameterKind.evaluated),
        (f"{LIVING}.V", ParameterKind.bound),
        (f"{LIVING}.air.vol.T", ParameterKind.alias),
        ("building.production1.generator_001.nSeg", ParameterKind.free),
    ],
)
def test_the_init_file_tells_how_a_variable_takes_its_value(variables: dict, name: str, kind: ParameterKind) -> None:
    assert variables[name].kind == kind


def test_the_init_file_gives_start_values_units_and_types(variables: dict) -> None:
    ach = variables[f"{LIVING}.ACH"]
    assert (ach.start_value, ach.causality, ach.variability, ach.is_value_changeable) == (
        0.15,
        "parameter",
        "parameter",
        True,
    )
    assert variables["building.envelope1.external_wall_001.material[1].k"].unit == "W/(m.K)"
    assert variables["building.production1.generator_001.useStorageTank"].start_value == 1.0
    assert variables[f"{LIVING}.V"].start_value is None
    assert variables[f"{LIVING}.air.vol.T"].alias_variable == "building.envelope1.heatPortCon[1].T"


def test_the_init_file_is_read_for_the_names_asked_only() -> None:
    assert set(read_model_variables(INIT_XML, [f"{LIVING}.ACH", "nothing"])) == {f"{LIVING}.ACH"}


def test_the_default_experiment_of_the_init_file() -> None:
    experiment = read_default_experiment(INIT_XML)
    assert (experiment["stopTime"], experiment["stepSize"], experiment["outputFormat"]) == ("172800", "900", "mat")


# -- overrides -----------------------------------------------------------------------------------------------
def test_values_are_written_as_the_executable_reads_them() -> None:
    assert [format_value(value) for value in (0.1, 1e-5, 3, True, False)] == ["0.1", "1e-05", "3", "true", "false"]
    assert float(format_value(0.1 + 0.2)) == 0.1 + 0.2
    with pytest.raises(ValueError, match="finite"):
        format_value(math.nan)


def test_the_override_file_holds_one_sorted_assignment_per_line(tmp_path: Path) -> None:
    path = write_override_file(tmp_path / "overrides.txt", {f"{LIVING}.ACH": 0.3, f"{LIVING}.AFlo": 44})
    assert path.read_text() == f"{LIVING}.ACH=0.3\n{LIVING}.AFlo=44\n"


@pytest.mark.parametrize("name", ["", "a b", "a=b", "a,b", "stopTime"])
def test_invalid_override_names_are_refused(tmp_path: Path, name: str) -> None:
    with pytest.raises(ValueError, match="Invalid name"):
        write_override_file(tmp_path / "overrides.txt", {name: 1.0})


def test_experiment_settings_are_flags_of_the_executable() -> None:
    assert experiment_flags({"startTime": 0, "stopTime": 86400.0, "tolerance": 1e-6}) == [
        "-startTime=0",
        "-stopTime=86400.0",
        "-tolerance=1e-06",
    ]
    with pytest.raises(ValueError, match="Unknown experiment settings"):
        experiment_flags({"solver": "dassl"})


def test_overrides_the_executable_does_not_apply_are_found_in_its_log() -> None:
    assert parse_override_messages(LOG) == [
        OverrideMessage("stopTime", OverrideIssue.repeated),
        OverrideMessage(f"{LIVING}.V", OverrideIssue.not_overridable),
        OverrideMessage("building.does.not.exist", OverrideIssue.not_found),
    ]
    assert ignored_overrides(LOG, [f"{LIVING}.V", f"{LIVING}.ACH"]) == [
        OverrideMessage(f"{LIVING}.V", OverrideIssue.not_overridable)
    ]


def test_an_ignored_override_never_passes_as_a_change(tmp_path: Path) -> None:
    run = RunResult(
        "run", tmp_path / "missing.mat", LOG, 0, {f"{LIVING}.V": 200.0}, tuple(parse_override_messages(LOG))
    )
    result = check_readback(run, {f"{LIVING}.V": 200.0}, [], {})
    assert not result.ok
    assert "not_overridable" in result.reason
    assert not run.success  # no result file


# -- build ---------------------------------------------------------------------------------------------------
def test_the_build_script_compiles_with_build_model_and_no_parameter_evaluation_flag() -> None:
    script = build_script("package p end p;", "build/p.mo", "p.building", SimulationOptions(end_time=86400))
    assert 'loadFile("/simulation/build/p.mo");' in script
    assert 'cd("/simulation/build");' in script
    assert 'buildModel(p.building, startTime = 0, stopTime = 86400, tolerance = 0.0001, outputFormat="mat");' in script
    assert "setCommandLineOptions" not in script
    assert "evaluate" not in script.lower()


def test_the_build_script_records_the_flags_given() -> None:
    script = build_script("", "m.mo", "p.building", SimulationOptions(), flags=("-d=aliasConflicts",))
    assert 'setCommandLineOptions("-d=aliasConflicts");' in script


def test_a_compiled_model_is_found_again_from_its_build_directory(tmp_path: Path) -> None:
    compiled = CompiledModel(
        tmp_path, "p.building", options={"end_time": 3600}, specification="/x.yaml", library="Buildings"
    )
    compiled.directory.mkdir()
    compiled.save()
    assert CompiledModel.load(compiled.directory) == compiled
    assert compiled.init_xml.name == "p.building_init.xml"
    assert not compiled.exists()


# -- expressions and specification ---------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("expression", "expected"),
    [
        ("3600*{7, 23}", {"x[1]": 25200.0, "x[2]": 82800.0}),
        ("[35; 70; 30]", {"x[1,1]": 35.0, "x[2,1]": 70.0, "x[3,1]": 30.0}),
        ("1/6/4", {"x": 1 / 6 / 4}),
        ("0.05/1000*{0.5,1}", {"x[1]": 0.05 / 1000 * 0.5, "x[2]": 0.05 / 1000}),
        ("{5000,-1,-1}", {"x[1]": 5000.0, "x[2]": -1.0, "x[3]": -1.0}),
        ("2.6e-5", {"x": 2.6e-5}),
        (21.0, {"x": 21.0}),
        ("true", None),
        ("{true, true}", None),
        ("Buildings.Fluid.Types.EfficiencyCurves.Constant", None),
    ],
)
def test_parameter_values_are_scalarised_as_the_executable_names_them(expression: str, expected: dict | None) -> None:
    assert scalarize("x", expression) == expected


def test_a_boolean_is_not_numeric() -> None:
    with pytest.raises(NotNumericError):
        evaluate(True)
    assert modelica_array([25200.0, 84600.0]) == "{25200, 84600}"


def test_the_numeric_fields_of_the_example() -> None:
    description = load_description(EXAMPLE)
    fields = {field.path: field.value for field in numeric_fields(description)}
    assert fields[EPS_K] == 0.032
    assert fields["glass_material[GLASS:001].solar_transmittance"] == [0.83]
    assert fields["spaces[LIVING:001].occupancy.parameters.occupancy"] == "3600*{7, 8, 17, 23}"
    assert fields["systems[GENERATOR:001].boiler.parameters.nominal_heating_power"] == 3000
    assert fields["spaces[LIVING:001].external_boundaries.windows[0].frame_fraction"] == 0.25
    assert "weather.parameters.path" not in fields
    assert not any(path.endswith((".id", ".variant")) for path in fields)
    assert "spaces[LIVING:001].external_boundaries.external_walls[0].tilt" not in fields  # "wall"
    assert fields["solar[PV:001].photovoltaic.parameters.tilt"] == 35.0
    assert all(get_value(description, path) == value for path, value in fields.items())


def test_paths_select_list_items_by_id_or_index() -> None:
    description = load_description(EXAMPLE)
    assert resolve_keys(description, EPS_K)[:2] == ("material", 10)
    assert resolve_keys(description, "systems[GENERATOR:001].boiler.parameters.tank_volume")[:3] == (
        "systems",
        0,
        "boiler",
    )
    assert resolve_keys(description, "systems[0].boiler.parameters.tank_volume") == resolve_keys(
        description, "systems[GENERATOR:001].boiler.parameters.tank_volume"
    )
    with pytest.raises(KeyError):
        resolve_keys(description, "material[NOPE:001].density")
    changed = with_values(description, {EPS_K: 0.04})
    assert get_value(changed, EPS_K) == 0.04
    assert get_value(description, EPS_K) == 0.032


@pytest.mark.parametrize(
    ("path", "value", "expected"),
    [
        (EPS_K, 0.032, 0.032 * 1.2),
        ("x.heat_exchanger_effectiveness", 0.8, 0.8 * 0.8),
        ("x.solar_transmittance", [0.83], [0.83 * 0.8]),
        ("x.infrared_transmissivity", 0.0, 0.05),
        ("x.temperature_heating_setpoint", 294.15, 296.15),
        ("x.temperature_heating_setback", 289.15, 287.15),
        ("x.ground_temperature", 286.65, 288.65),
        ("x.schedule", "3600*{7, 23}", "{27000, 84600}"),
        ("x.occupancy", "3600*{0, 7, 22, 24}", "{0, 27000, 81000, 86400}"),
        ("x.azimuth", 0.0, 0.2),
    ],
)
def test_perturbations_stay_physical(path: str, value: object, expected: object) -> None:
    perturbed = perturbation(YamlField(path, tuple(path.split(".")), value))  # type: ignore[arg-type]
    assert perturbed == pytest.approx(expected) if not isinstance(expected, str) else perturbed == expected


@pytest.mark.parametrize(
    ("path", "group", "priority"),
    [
        (EPS_K, "Opaque materials", Priority.high),
        ("material[BRICK_INNER:001].density", "Opaque materials", Priority.medium),
        ("constructions[ROOF:001].layers[0].thickness", "Layers", Priority.medium),
        ("glazings[WINDOW:001].u_value_frame", "Glazing", Priority.high),
        ("spaces[LIVING:001].external_boundaries.windows[0].frame_fraction", "Glazing", Priority.medium),
        ("spaces[LIVING:001].parameters.n50", "Space", Priority.high),
        ("spaces[LIVING:001].external_boundaries.floor_on_grounds[0].ground_temperature", "Ground", Priority.high),
        ("spaces[LIVING:001].external_boundaries.external_walls[0].surface", "Geometry", Priority.low),
        (
            "spaces[LIVING:001].emissions[VALVE_LIVING:001].valve.control.emission_control.parameters.schedule",
            "Controls",
            Priority.medium,
        ),
        ("systems[GENERATOR:001].boiler.parameters.tank_volume", "Heat pump", Priority.medium),
        ("systems[PUMP:001].pump.parameters.m_flow_nominal", "Hydronics", Priority.low),
        ("systems[AHU:001].air_handling_unit.parameters.heat_exchanger_effectiveness", "Ventilation", Priority.high),
        ("solar[PV:001].photovoltaic.parameters.azimuth", "PV", Priority.medium),
    ],
)
def test_fields_are_grouped_as_in_the_brief(path: str, group: str, priority: Priority) -> None:
    assert group_of(path) == (group, priority)


# -- resolution (trano conversion, no Docker) ----------------------------------------------------------------
def test_a_material_reaches_every_construction_and_zone_using_it(resolver: SpecificationResolver) -> None:
    """The envelope record and the copy OpenModelica makes in each zone, with what Buildings derives from k."""
    resolution = resolver.resolve(EPS_K, 0.033)  # small enough to keep the discretisation of the layer
    overrides = resolution.overrides
    copies = [
        "building.envelope1.external_wall_001",
        *(f"{LIVING}.datConExtWin[{j}].layers" for j in (1, 2, 3)),
        "building.envelope1.bedroom_1_001.datConExt[1].layers",
    ]
    for record in copies:
        assert overrides[f"{record}.material[1].k"] == 0.033
        assert overrides[f"{record}.material[1].R"] == pytest.approx(0.12 / 0.033)  # what the conduction uses
        assert f"{record}.material[1].piMat" in overrides
        assert f"{record}.R" in overrides  # the construction's total, evaluated in the executable
    assert sum(name.endswith("material[1].k") for name in overrides) == 12  # 1 record and 11 zone copies
    assert not any(name.endswith("nSta") for name in overrides)
    assert resolution.kind == ResolutionKind.derived  # R, piMat are not the YAML value


@pytest.mark.parametrize(("path", "value"), [(EPS_K, 0.0384), ("material[EPS:001].density", 100.0)])
def test_a_material_change_can_alter_the_discretisation(
    resolver: SpecificationResolver, path: str, value: float
) -> None:
    """nSta = ceil(nStaRef * x * sqrt(c * d / k) / 331.4): +20 % on the conductivity of EPS takes it from 2 to 1."""
    resolution = resolver.resolve(path, value)
    assert "building.envelope1.external_wall_001.material[1].nSta" in resolution.runtime_names
    assert resolution.kind == ResolutionKind.derived  # nSta is not the YAML value


def test_the_airtightness_reaches_the_infiltration_through_a_python_transformation(
    resolver: SpecificationResolver,
) -> None:
    resolution = resolver.resolve("spaces[LIVING:001].parameters.n50", 3.6)
    assert resolution.kind == ResolutionKind.derived
    assert resolution.overrides == pytest.approx({f"{LIVING}.ACH": 0.18})
    assert "n50_to_ach" in (resolution.transformation or "")


def test_the_heat_pump_power_fans_out_to_its_flows(resolver: SpecificationResolver) -> None:
    resolution = resolver.resolve("systems[GENERATOR:001].boiler.parameters.nominal_heating_power", 3600.0)
    generator = "building.production1.generator_001"
    assert resolution.overrides == pytest.approx(
        {
            f"{generator}.Q_flow_nominal": 3600.0,
            f"{generator}.nominal_mass_flow_radiator_loop": 1.5 * 3600 / 10 / 4200,
            f"{generator}.nominal_mass_flow_rate_boiler": 1.5 * 3600 / 20 / 4200,
            f"{generator}.V_flow[1]": 1.5 * 3600 / 20 / 4200 / 1000 * 0.5,
            f"{generator}.V_flow[2]": 1.5 * 3600 / 20 / 4200 / 1000,
        }
    )


def test_ideas_only_ground_fields_are_not_used(resolver: SpecificationResolver) -> None:
    for name, value in (("perimeter", 18.0), ("average_indoor_temperature", 295.15)):
        resolution = resolver.resolve(f"spaces[LIVING:001].external_boundaries.floor_on_grounds[0].{name}", value)
        assert resolution.kind == ResolutionKind.not_used
        assert "IDEAS only" in (resolution.not_used_reason or "")


def test_a_window_cannot_turn_away_from_its_wall(resolver: SpecificationResolver) -> None:
    resolution = resolver.resolve("spaces[LIVING:001].external_boundaries.windows[0].azimuth", 0.2)
    assert resolution.kind == ResolutionKind.structural
    assert "No wall found" in (resolution.error or "")


def test_yaml_changes_become_overrides_together(resolver: SpecificationResolver) -> None:
    overrides = resolver.overrides(
        {"spaces[LIVING:001].parameters.n50": 3.6, "solar[PV:001].photovoltaic.parameters.efficiency": 0.22}
    )
    assert overrides == pytest.approx({f"{LIVING}.ACH": 0.18, "building.solar1.pv_001.eta": 0.22})


def test_the_snapshot_matches_the_start_values_of_the_executable(
    resolver: SpecificationResolver, variables: dict
) -> None:
    snapshot = resolver.baseline.snapshot
    common = [name for name in variables if name in snapshot]
    assert len(common) >= 12
    for name in common:
        if variables[name].start_value is not None:
            assert snapshot[name].value == pytest.approx(variables[name].start_value, rel=1e-12), name


def test_the_impact_outputs_of_the_example() -> None:
    from trano.data_models.conversion import convert_network
    from trano.elements.library.library import Library

    network = convert_network("residential_after_renovation", EXAMPLE, library=Library.from_configuration("Buildings"))
    network.model()
    outputs = get_adapter("Buildings").impact_outputs(network)
    assert outputs["T_living_001"] == f"{LIVING}.air.vol.T"
    assert outputs["P_pv_001"] == "building.solar1.pv_001.P"
    assert sum(label.startswith("Q_radiator") for label in outputs) == 6


def test_the_discretisation_of_the_insulation_layer() -> None:
    from trano.elements.construction import Layer, Material

    eps = Material(name="EPS", thermal_conductivity=0.032, density=20, specific_heat_capacity=1450)
    assert number_of_states(Layer(material=eps, thickness=0.12)) == 2  # as in the init file
    assert number_of_states(Layer(material=eps.model_copy(update={"density": 100}), thickness=0.12)) == 3
    assert (
        number_of_states(Layer(material=eps.model_copy(update={"thermal_conductivity": 0.0384}), thickness=0.12)) == 1
    )


def test_unknown_libraries_have_no_adapter_yet() -> None:
    with pytest.raises(NotImplementedError, match="IDEAS"):
        get_adapter("IDEAS")


# -- classification ------------------------------------------------------------------------------------------
def _resolution(names: list[str], kind_value: float = 1.0, yaml_value: float = 1.0) -> Resolution:
    return Resolution(
        path="spaces[LIVING:001].parameters.x",
        value=yaml_value,
        new_value=kind_value,
        changes=tuple(ParameterChange(name, 0.5, kind_value, name) for name in names),
        model_changed=True,
    )


def test_a_field_on_free_parameters_is_a_candidate(variables: dict) -> None:
    row = static_row(_resolution([f"{LIVING}.ACH"]), variables)
    assert (row.status, row.causality, row.is_value_changeable, row.unit) == (
        Status.changeable,
        "parameter",
        "true",
        "1/h",
    )


def test_a_field_on_evaluated_parameters_is_frozen_with_its_mechanism(variables: dict) -> None:
    row = static_row(
        _resolution(["building.solar1.pv_001.til"]), variables, {r"\.pv_[^.]+\.til$": "Evaluate=true in Buildings"}
    )
    assert row.status == Status.frozen
    assert "evaluated at translation" in row.reason
    assert "Evaluate=true in Buildings" in row.reason
    assert row.is_value_changeable == "false"


def test_a_field_partly_on_evaluated_parameters_is_left_to_the_checks(variables: dict) -> None:
    resolution = _resolution(
        ["building.emission1.radiator_living_001.Q_flow_nominal", "building.emission1.radiator_living_001.mDry"],
        kind_value=4680.0,
    )
    row = static_row(resolution, variables)
    assert row.status == Status.changeable
    assert "1 of 2 run-time names cannot be overridden" in row.reason
    assert row.is_value_changeable == "mixed"
    assert free_overrides(resolution, variables) == {"building.emission1.radiator_living_001.Q_flow_nominal": 4680.0}


def test_a_field_on_names_absent_from_the_executable_is_missing(variables: dict) -> None:
    row = static_row(_resolution(["building.nothing.k"]), variables)
    assert row.status == Status.missing
    assert "building.nothing.k" in row.reason


def test_a_field_the_model_ignores_is_not_used(variables: dict) -> None:
    resolution = Resolution("p", 1.0, 2.0, (), model_changed=False, not_used_reason="IDEAS only")
    assert static_row(resolution, variables).status == Status.not_used


def test_appearing_parameters_are_structural() -> None:
    from trano.runtime.adapters import SnapshotEntry

    changes = diff({"a": SnapshotEntry(1.0, "a")}, {"a": SnapshotEntry(1.0, "a"), "b": SnapshotEntry(2.0, "b")})
    resolution = Resolution("p", 1.0, 2.0, changes, model_changed=True)
    assert resolution.kind == ResolutionKind.structural
    assert resolution.structural_names == ("b",)


def test_the_inventory_has_the_columns_of_the_brief(tmp_path: Path) -> None:
    assert InventoryRow.columns() == [
        "yaml_path",
        "modelica_paths",
        "runtime_names",
        "unit",
        "default",
        "causality",
        "variability",
        "is_value_changeable",
        "readback_ok",
        "impact_metric",
        "equivalence_ok",
        "status",
        "reason",
    ]
    rows = [InventoryRow(yaml_path=EPS_K, status=Status.changeable)]
    assert read_inventory(write_inventory(rows, tmp_path / "inventory.csv"))[0].status == "changeable"


# -- checks --------------------------------------------------------------------------------------------------
def test_variable_filters_keep_the_names_and_patterns_given() -> None:
    import re

    pattern = re.compile(variable_filter([f"{LIVING}.datConExtWin[1].A"]))
    assert pattern.match(f"{LIVING}.datConExtWin[1].A")
    assert not pattern.match(f"{LIVING}.datConExtWin[1].AFlo")
    record = re.compile(parameter_filter([f"{LIVING}.datConExtWin[1].layers"], ["out.T"]))
    assert record.match(f"{LIVING}.datConExtWin[1].layers.material[2].k")
    assert record.match("out.T")
    assert not record.match(f"{LIVING}.datConExtWin[2].layers.material[2].k")


def test_the_impact_metric_is_normalised_by_the_range_of_the_output() -> None:
    baseline = OutputSeries({"Q": np.array([0.0, 100.0, 0.0, 100.0]), "T": np.array([293.0, 294.0, 293.0, 294.0])})
    run = OutputSeries({"Q": np.array([0.0, 110.0, 0.0, 110.0]), "T": np.array([293.0, 294.0, 293.0, 294.0])})
    metrics = run.nrmsd(baseline)
    assert metrics == pytest.approx({"Q": math.sqrt(50.0) / 100, "T": 0.0})
    assert run.energy_change(baseline, "Q") == pytest.approx(0.1)
    impact = ImpactResult(metrics, {"Q": 0.1})
    assert impact.has_impact
    assert impact.summary().startswith("max NRMSD 7.07e-02 (Q); energy Q +10.00%")
    assert not ImpactResult({"Q": IMPACT_THRESHOLD / 2}, {}).has_impact


# -- overrides checked against the compiled model ------------------------------------------------------------
FIXTURE_BUILD = CompiledModel(INIT_XML.parents[1], "residential_after_renovation.building", build_directory="runtime")


def test_yaml_changes_on_free_parameters_become_overrides_of_the_executable(resolver: SpecificationResolver) -> None:
    assert resolver.overrides({"spaces[LIVING:001].parameters.n50": 3.6}, FIXTURE_BUILD) == pytest.approx(
        {f"{LIVING}.ACH": 0.18}
    )


def test_yaml_changes_on_evaluated_parameters_are_refused(resolver: SpecificationResolver) -> None:
    with pytest.raises(NotRuntimeChangeableError, match="Evaluate=true") as error:
        resolver.overrides({"solar[PV:001].photovoltaic.parameters.tilt": 40.0}, FIXTURE_BUILD)
    assert error.value.names == ["building.solar1.pv_001.til"]
    assert error.value.free == {}


def test_evaluated_parameters_the_model_does_not_use_are_left_out(
    resolver: SpecificationResolver, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The construction's total R is evaluated but unused (equivalence exact): a material change needs only the rest."""
    resolution = resolver.resolve(EPS_K, 0.033)
    kinds = {
        name: ParameterKind.evaluated if name.endswith((".layers.R", "external_wall_001.R")) else ParameterKind.free
        for name in resolution.runtime_names
    }
    monkeypatch.setattr(resolver, "parameter_kinds", lambda compiled: kinds)
    overrides = resolver.overrides({EPS_K: 0.033}, FIXTURE_BUILD)
    assert "building.envelope1.external_wall_001.R" not in overrides
    assert overrides["building.envelope1.external_wall_001.material[1].R"] == pytest.approx(0.12 / 0.033)
    kinds["building.envelope1.external_wall_001.material[1].nSta"] = ParameterKind.evaluated
    with pytest.raises(NotRuntimeChangeableError):
        resolver.overrides({EPS_K: 0.0384}, FIXTURE_BUILD)  # outside the admissible range: nSta changes


# -- verification of the values a run used -------------------------------------------------------------------
# The line the executable prints with -output=<names> at the end of a run (OpenModelica 1.26.9).
OUTPUT_LOG = (
    "LOG_SUCCESS       | info    | The simulation finished successfully.\n"
    f"time=3600,{LIVING}.ACH=0.2999999999999999889,{LIVING}.datConExtWin[1].layers.material[1].k="
    "0.033000000000000001554,building.envelope1.occupancy_1.gain[2,1]=70,building.solar1.pv_001.til=0.61086499999999999133\n"
)


def test_the_values_used_by_the_simulation_are_read_from_its_log() -> None:
    assert parse_output_values(OUTPUT_LOG) == pytest.approx(
        {
            f"{LIVING}.ACH": 0.3,
            f"{LIVING}.datConExtWin[1].layers.material[1].k": 0.033,
            "building.envelope1.occupancy_1.gain[2,1]": 70.0,
            "building.solar1.pv_001.til": 0.610865,
        }
    )
    assert output_flag(["b", "a[1,2]"]) == "-output=a[1,2],b"


def test_a_run_that_used_every_value_set_is_verified() -> None:
    verification = verify_overrides(
        OUTPUT_LOG, {f"{LIVING}.ACH": 0.3, f"{LIVING}.datConExtWin[1].layers.material[1].k": 0.033}
    )
    assert verification.ok
    assert verification.summary() == "2 overrides applied: the simulation used every value set"


def test_a_value_the_simulation_did_not_use_is_reported() -> None:
    log = LOG + OUTPUT_LOG  # LOG holds the refusal of the volume V
    verification = verify_overrides(log, {"building.solar1.pv_001.til": 0.5, f"{LIVING}.V": 200.0, "x.y": 1.0})
    assert not verification.ok
    assert verification.differs == {"building.solar1.pv_001.til": (0.5, pytest.approx(0.610865))}
    assert [message.name for message in verification.refused] == [f"{LIVING}.V"]
    assert verification.missing == (f"{LIVING}.V", "x.y")
    assert "set 0.5, used 0.610865" in verification.summary()


def test_a_finished_run_is_verified_from_its_files(tmp_path: Path) -> None:
    compiled = CompiledModel(tmp_path, "p.building")
    runs = tmp_path / "runs"
    runs.mkdir()
    write_override_file(runs / "trial.overrides.txt", {f"{LIVING}.ACH": 0.3})
    (runs / "trial.log").write_text(OUTPUT_LOG)
    assert read_override_file(runs / "trial.overrides.txt") == {f"{LIVING}.ACH": 0.3}
    assert verify_run(compiled, "trial").ok
    (runs / "trial.log").write_text("LOG_SUCCESS | info | The simulation finished successfully.\n")
    assert verify_run(compiled, "trial").missing == (f"{LIVING}.ACH",)  # a run made without -output
