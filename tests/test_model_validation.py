"""Validation of the YAML models: each rule has an invalid model and a valid counterpart.

The models are small changes of three models of `tests/models` (hydronic heating, air handling unit and
ideal heaters). An invalid model must be rejected with `InvalidModelError`, naming the offending id; its
valid counterpart must translate.
"""

import copy
import re
import warnings
from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest
import yaml

from trano.data.include import Loader
from trano.data_models.conversion import convert_network
from trano.exceptions import InvalidModelError
from trano.validation.issues import HydronicLayoutWarning, ModelWarning

MODELS = Path(__file__).parent / "models"
Model = dict[str, Any]
Change = Callable[[Model], None]


def load(name: str) -> Model:
    return yaml.load((MODELS / name).read_text(), Loader=Loader)  # noqa: S506


HYDRONIC = load("single_zone_hydronic.yaml")
AHU = load("single_zone_air_handling_unit_simple_vav_control.yaml")
IDEAL = load("three_zones_ideal_heaters.yaml")


def system(model: Model, id_: str) -> dict[str, Any]:
    return next(value for entry in model["systems"] for value in entry.values() if value["id"] == id_)


def emission(model: Model, id_: str) -> dict[str, Any]:
    return next(
        value
        for space in model["spaces"]
        for entry in space.get("emissions", [])
        for value in entry.values()
        if value["id"] == id_
    )


def parameters(get: Callable[[Model], dict[str, Any]], **values: Any) -> Change:  # noqa: ANN401
    def change(model: Model) -> None:
        get(model).setdefault("parameters", {}).update(values)

    return change


def _write(base: Model, change: Change, tmp_path: Path, name: str) -> Path:
    model = copy.deepcopy(base)
    change(model)
    path = tmp_path / f"{name}.yaml"
    path.write_text(yaml.safe_dump(model, sort_keys=False))
    return path


def unchanged(model: Model) -> None:
    """The valid counterpart of a rule that the base model already satisfies."""


def _boiler(model: Model) -> dict[str, Any]:
    return system(model, "BOILER:001")


def _boiler_control(model: Model) -> dict[str, Any]:
    return system(model, "BOILER:001")["control"]["boiler_control"]


def _radiator(model: Model) -> dict[str, Any]:
    return emission(model, "RADIATOR:003")


def _emission_control(model: Model) -> dict[str, Any]:
    return emission(model, "VALVE:003")["control"]["emission_control"]


def _wall(model: Model) -> dict[str, Any]:
    return model["spaces"][0]["external_boundaries"]["external_walls"][0]


def _window(model: Model) -> dict[str, Any]:
    return model["spaces"][0]["external_boundaries"]["windows"][0]


def _remove_pump(model: Model) -> None:
    model["systems"] = [entry for entry in model["systems"] if "pump" not in entry]
    system(model, "THREE_WAY_VALVE:001")["inlets"] = ["SPLIT_VALVE:002", "BOILER:001"]


def _mixing_circuit(model: Model) -> None:
    """Boiler -> three-way valve -> sensor -> pump -> radiator: the pump after the valve."""
    system(model, "THREE_WAY_VALVE:001")["inlets"] = ["SPLIT_VALVE:002", "BOILER:001"]
    system(model, "THREE_WAY_VALVE:001")["outlets"] = ["TEMPERATURE_SENSOR:001"]
    system(model, "PUMP:001")["inlets"] = ["TEMPERATURE_SENSOR:001"]
    system(model, "PUMP:001")["outlets"] = ["RADIATOR:003"]
    system(model, "TEMPERATURE_SENSOR:001")["outlets"] = []


def _set(get: Callable[[Model], dict[str, Any]], key: str, value: Any) -> Change:  # noqa: ANN401
    def change(model: Model) -> None:
        get(model)[key] = value

    return change


def _pop(get: Callable[[Model], dict[str, Any]], key: str) -> Change:
    def change(model: Model) -> None:
        get(model).pop(key)

    return change


def _append_system(entry: dict[str, Any]) -> Change:
    def change(model: Model) -> None:
        model["systems"].append(copy.deepcopy(entry))

    return change


def _emissions(*kinds: str) -> Change:
    """Keep only the emissions of the given kinds in the first space, wiring the return accordingly."""

    def change(model: Model) -> None:
        space = model["spaces"][0]
        space["emissions"] = [entry for entry in space["emissions"] if next(iter(entry)) in kinds]
        last = next(iter(space["emissions"][-1].values()))["id"]
        system(model, "SPLIT_VALVE:002")["inlets"] = [last]
        first = next(iter(space["emissions"][0].values()))["id"]
        system(model, "TEMPERATURE_SENSOR:001")["outlets"] = [first]

    return change


# rule id, base model, invalid change, valid counterpart, id named in the message, message fragment
RULES: list[tuple[str, Model, Change, Change, str, str]] = [
    # C1-C2 incompatible pairings
    (
        "C1 water feeds air",
        HYDRONIC,
        lambda m: (
            m["spaces"][0].update(ventilation_inlets=[{"duct": {"id": "DUCT:009"}}]),
            system(m, "TEMPERATURE_SENSOR:001").update(outlets=["DUCT:009"]),
        ),
        unchanged,
        "TEMPERATURE_SENSOR:001",
        "Water and air components cannot be connected",
    ),
    (
        "C2 water feeds an ideal radiator",
        HYDRONIC,
        _set(_radiator, "variant", "idealbus"),
        unchanged,
        "RADIATOR:003",
        "has no water circuit",
    ),
    # C3-C4 control and variant per class (schema)
    (
        "C3 control of another kind",
        HYDRONIC,
        _set(lambda m: system(m, "PUMP:001"), "control", {"boiler_control": {"id": "CONTROL:9"}}),
        unchanged,
        "systems[1].pump.control.boiler_control",
        "unknown field",
    ),
    (
        "C4 variant of another class",
        HYDRONIC,
        _set(lambda m: system(m, "PUMP:001"), "variant", "idealbus"),
        unchanged,
        "systems[1].pump.variant",
        "must be one of default (got 'idealbus')",
    ),
    (
        "C3 VAV control of another variant",
        AHU,
        lambda m: m["spaces"][0]["ventilation_inlets"][1]["vav"]["control"]["vav_control"].update(variant="default"),
        unchanged,
        "VAV:001",
        "vav_control of variant 'constant'",
    ),
    # C5 open circuits
    (
        "C5 open return",
        HYDRONIC,
        lambda m: (system(m, "SPLIT_VALVE:002").pop("outlets"), system(m, "BOILER:001").pop("inlets")),
        unchanged,
        "BOILER:001",
        "not on a closed water circuit (nothing upstream)",
    ),
    (
        "C5 three-way valve without bypass",
        HYDRONIC,
        _set(lambda m: system(m, "THREE_WAY_VALVE:001"), "inlets", ["PUMP:001"]),
        unchanged,
        "THREE_WAY_VALVE:001",
        "has 2 connections",
    ),
    (
        "C5 open air circuit",
        AHU,
        _pop(lambda m: system(m, "AHU:001"), "inlets"),
        unchanged,
        "AHU:001",
        "not on a closed air circuit",
    ),
    # C6 orphans
    (
        "C6 orphan system",
        HYDRONIC,
        _append_system({"pump": {"id": "PUMP:099", "control": {"collector_control": None}}}),
        unchanged,
        "PUMP:099",
        "not connected to anything",
    ),
    (
        "C6 mpc-only system",
        HYDRONIC,
        _append_system({"battery": {"id": "BATTERY:001"}}),
        unchanged,
        "BATTERY:001",
        "only modelled by the 'mpc' library",
    ),
    # C7-C8 emitters of a space
    (
        "C7 two emitters",
        HYDRONIC,
        lambda m: m["spaces"][0]["emissions"].append(
            {"radiator": {"id": "RADIATOR:777", "variant": "idealbus", "control": {"emission_control": None}}}
        ),
        unchanged,
        "SPACE:001",
        "a space takes one",
    ),
    ("C8 valve without radiator", HYDRONIC, _emissions("valve"), unchanged, "SPACE:001", "no water radiator"),
    ("C8 radiator without valve", HYDRONIC, _emissions("radiator"), unchanged, "RADIATOR:003", "without a valve"),
    # C9-C10 references
    (
        "C9 unknown outlet",
        HYDRONIC,
        _set(lambda m: system(m, "TEMPERATURE_SENSOR:001"), "outlets", ["RADIATOR:404"]),
        unchanged,
        "TEMPERATURE_SENSOR:001",
        "'RADIATOR:404'",
    ),
    (
        "C9 unknown construction",
        HYDRONIC,
        _set(_wall, "construction", "NOPE:001"),
        unchanged,
        "SPACE:001",
        "'NOPE:001', which is not defined",
    ),
    (
        "C9 unknown material",
        HYDRONIC,
        lambda m: m["default"]["constructions"][0]["layers"][0].update(material="NOPE:MAT"),
        unchanged,
        "NOPE:MAT",
        "not defined",
    ),
    (
        "C10 window with a wall construction",
        HYDRONIC,
        _set(_window, "construction", "CAVITYWALL:001"),
        unchanged,
        "SPACE:001",
        "which is a construction; windows take a glazing",
    ),
    # C11-C14 ids and edges
    (
        "C11 duplicate system id",
        HYDRONIC,
        _append_system({"pump": {"id": "PUMP:001", "control": {"collector_control": None}}}),
        unchanged,
        "PUMP:001",
        "the id is used by 2 components",
    ),
    (
        "C11 ids equal once cleaned",
        HYDRONIC,
        _append_system({"temperature_sensor": {"id": "pump_001"}}),
        unchanged,
        "PUMP:001",
        "pump_001",
    ),
    (
        "C12 self-connection",
        HYDRONIC,
        _set(lambda m: system(m, "PUMP:001"), "outlets", ["PUMP:001"]),
        unchanged,
        "PUMP:001",
        "connected to itself",
    ),
    (
        "C13 both directions",
        HYDRONIC,
        _set(lambda m: system(m, "PUMP:001"), "inlets", ["BOILER:001", "THREE_WAY_VALVE:001"]),
        unchanged,
        "PUMP:001",
        "both upstream and downstream of THREE_WAY_VALVE:001",
    ),
    (
        "C14 two sources on one port",
        HYDRONIC,
        _append_system(
            {
                "pump": {
                    "id": "PUMP:002",
                    "inlets": ["BOILER:001"],
                    "outlets": ["THREE_WAY_VALVE:001"],
                    "control": {"collector_control": None},
                }
            }
        ),
        unchanged,
        "THREE_WAY_VALVE:001",
        "has 4 connections",
    ),
    # C15-C17 missing companions
    (
        "C15 valve without control",
        HYDRONIC,
        _pop(lambda m: emission(m, "VALVE:003"), "control"),
        unchanged,
        "VALVE:003",
        "needs a control",
    ),
    (
        "C15 pump without control",
        HYDRONIC,
        _pop(lambda m: system(m, "PUMP:001"), "control"),
        unchanged,
        "PUMP:001",
        "needs a control",
    ),
    (
        "C16 three-way valve without sensor",
        HYDRONIC,
        lambda m: (
            m.update(systems=[e for e in m["systems"] if "temperature_sensor" not in e]),
            system(m, "THREE_WAY_VALVE:001").update(outlets=["RADIATOR:003"]),
        ),
        unchanged,
        "THREE_WAY_VALVE:001",
        "no temperature_sensor follows it",
    ),
    ("C17 no pump", HYDRONIC, _remove_pump, _mixing_circuit, "BOILER:001", "has no pump"),
    # C18 missing space data (schema)
    (
        "C18 space without envelope",
        HYDRONIC,
        _pop(lambda m: m["spaces"][0], "external_boundaries"),
        unchanged,
        "spaces[0].external_boundaries",
        "is required",
    ),
    # C19-C21 parameter values
    (
        "C19 zero boiler power",
        HYDRONIC,
        parameters(_boiler, nominal_heating_power=0),
        parameters(_boiler, nominal_heating_power=1),
        "BOILER:001",
        "must be strictly positive",
    ),
    (
        "C19 zero conductivity",
        HYDRONIC,
        lambda m: m["default"]["material"][0].update(thermal_conductivity=0),
        unchanged,
        "thermal_conductivity = 0",
        "must be strictly positive",
    ),
    ("C19 negative surface", HYDRONIC, _set(_wall, "surface", -20), unchanged, "spaces[0]", "surface: must be >= 0"),
    (
        "C19 emissivity above 1",
        HYDRONIC,
        lambda m: m["default"]["material"][0].update(longwave_emissivity=1.5),
        unchanged,
        "longwave_emissivity",
        "must be <= 1",
    ),
    (
        "C20 temperature in degC",
        HYDRONIC,
        parameters(_boiler_control, tsup_nominal=70),
        parameters(_boiler_control, tsup_nominal=343.15),
        "tsup_nominal",
        "70 degC is 343.15 K",
    ),
    (
        "C21 radiator inlet below outlet",
        HYDRONIC,
        parameters(_radiator, water_inlet_temperature_at_nominal_condition=320.15),
        parameters(_radiator, water_inlet_temperature_at_nominal_condition=343.15),
        "RADIATOR:003",
        "must be above its nominal outlet temperature",
    ),
    (
        "C21 setback above set point",
        HYDRONIC,
        parameters(_emission_control, temperature_heating_setback=300.15),
        parameters(_emission_control, temperature_heating_setback=288.15),
        "EMISSION_CONTROL:001",
        "setback temperature",
    ),
    # P1-5 one key per entry (schema)
    (
        "P1-5 two components in one entry",
        HYDRONIC,
        lambda m: m["systems"].append({"boiler": {"id": "B:9"}, "pump": {"id": "P:9"}}),
        unchanged,
        "systems[5]",
        "exactly one of",
    ),
    (
        "P1-1 unknown parameter",
        HYDRONIC,
        parameters(_radiator, bogus=1),
        unchanged,
        "spaces[0].emissions[0].radiator.parameters.bogus",
        "unknown field",
    ),
]


@pytest.mark.parametrize(
    ("base", "invalid", "offender", "fragment"),
    [rule[1:3] + rule[4:] for rule in RULES],
    ids=[rule[0] for rule in RULES],
)
def test_invalid_model_is_rejected(base: Model, invalid: Change, offender: str, fragment: str, tmp_path: Path) -> None:
    path = _write(base, invalid, tmp_path, "invalid")
    with pytest.raises(InvalidModelError) as error:
        convert_network("invalid", path)
    assert offender in str(error.value)
    assert fragment in str(error.value)


@pytest.mark.parametrize(("base", "valid"), [(rule[1], rule[3]) for rule in RULES], ids=[rule[0] for rule in RULES])
def test_valid_counterpart_translates(base: Model, valid: Change, tmp_path: Path) -> None:
    path = _write(base, valid, tmp_path, "valid")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", ModelWarning)
        assert convert_network("valid", path).model()


def test_every_error_is_reported_at_once(tmp_path: Path) -> None:
    def two_errors(model: Model) -> None:
        system(model, "PUMP:001")["outlets"] = ["PUMP:001"]
        emission(model, "RADIATOR:003").setdefault("parameters", {})["nominal_heating_power_positive_for_heating"] = 0

    with pytest.raises(InvalidModelError) as error:
        convert_network("invalid", _write(HYDRONIC, two_errors, tmp_path, "invalid"))
    assert "PUMP:001: is connected to itself" in str(error.value)
    assert "RADIATOR:003: nominal_heating_power_positive_for_heating = 0" in str(error.value)


def test_ventilation_duct_shared_by_two_spaces_is_accepted() -> None:
    """The same duct as outlet of a space and inlet of the next one: transfer air, not a duplicate id."""
    assert convert_network("transfer", MODELS / "multizone_air_handling_unit_space_connected.yaml").model()


def test_weather_file_not_found_is_a_warning(tmp_path: Path) -> None:
    path = _write(
        HYDRONIC, lambda m: m.update(weather={"parameters": {"path": "/nowhere/weather.mos"}}), tmp_path, "weather"
    )
    with pytest.warns(ModelWarning, match="weather.mos"):
        convert_network("weather", path)


# Phase 3: physical checks backed by simulations (see trano/validation/hydronics.py)
WARNINGS: list[tuple[str, Model, Change, str, str]] = [
    (
        "T2 tank above boiling",
        HYDRONIC,
        parameters(_boiler_control, tsup_nominal=363.15),
        "BOILER:001",
        "above the boiling point of water",
    ),
    (
        "T1 diverting valve set below the supply",
        HYDRONIC,
        _set(
            lambda m: system(m, "THREE_WAY_VALVE:001")["control"],
            "three_way_valve_control",
            {"id": "THREE_WAY_VALVE_CONTROL:001", "parameters": {"supply_temperature_setpoint": 343.15}},
        ),
        "THREE_WAY_VALVE:001",
        "WILL CLOSE and the rooms will go cold",
    ),
    (
        "T4 undersized pump",
        HYDRONIC,
        parameters(lambda m: system(m, "PUMP:001"), m_flow_nominal=0.005),
        "PUMP:001",
        "less than half",
    ),
    (
        "T5 low valve authority",
        HYDRONIC,
        parameters(lambda m: emission(m, "VALVE:003"), dp_valve_nominal=500, dp_fixed_nominal=50000),
        "VALVE:003",
        "authority",
    ),
    (
        "C21 boiler switched off below its set point",
        HYDRONIC,
        parameters(_boiler_control, threshold_to_switch_off_boiler=343.15),
        "BOILER_CONTROL:001",
        "charged below the supply set point",
    ),
    (
        "gas layer in an opaque construction",
        HYDRONIC,
        lambda m: m.setdefault("constructions", []).append(
            {
                "id": "CAVITY:001",
                "layers": [
                    {"material": "BRICK:001", "thickness": 0.09},
                    {"material": "AIR:001", "thickness": 0.03},
                    {"material": "BRICK:001", "thickness": 0.09},
                ],
            }
        ),
        "CAVITY:001",
        "layer 1 is the gas 'AIR:001'.*AIRCAVITY:001",
    ),
]


@pytest.mark.parametrize(
    ("base", "change", "offender", "fragment"), [case[1:] for case in WARNINGS], ids=[case[0] for case in WARNINGS]
)
def test_risky_model_translates_with_a_warning(
    base: Model, change: Change, offender: str, fragment: str, tmp_path: Path
) -> None:
    with pytest.warns(ModelWarning, match=rf"{offender}: .*{fragment}"):
        assert convert_network("risky", _write(base, change, tmp_path, "risky")).model()


def test_supply_set_point_at_boiling_is_rejected(tmp_path: Path) -> None:
    path = _write(HYDRONIC, parameters(_boiler_control, tsup_nominal=373.15), tmp_path, "boiling")
    with pytest.raises(InvalidModelError, match="BOILER:001: its supply set point"):
        convert_network("boiling", path)


def _three_way_valve_set_points(model: str) -> list[str]:
    return re.findall(r"TColSet\w*\s*\(y=([0-9.]+)\)", model.replace("\n", " "))


def test_diverting_valve_set_point_is_raised_above_the_boiler_supply(tmp_path: Path) -> None:
    """Pump upstream of the valve and a boiler at 80 degC (tank top at 95 degC): the 90 degC set point would close
    the valve, as simulated (rooms at 16 degC in three_zones_hydronic.yaml), so it is raised to 100 degC."""
    path = _write(HYDRONIC, parameters(_boiler_control, tsup_nominal=353.15), tmp_path, "diverting")
    with pytest.warns(HydronicLayoutWarning, match="THREE_WAY_VALVE:001: WRONG HYDRONIC LAYOUT.*WORKAROUND APPLIED"):
        model = convert_network("diverting", path).model()
    assert _three_way_valve_set_points(model) == ["373.15"]


def test_valve_set_point_is_kept_when_the_supply_stays_below_it(tmp_path: Path) -> None:
    """The base model (boiler at 70 degC, tank top at 85 degC) keeps the 90 degC set point, but its layout is still
    reported: the valve stays open and controls nothing."""
    with pytest.warns(HydronicLayoutWarning, match="WRONG HYDRONIC LAYOUT.*controls nothing") as found:
        model = convert_network("kept", _write(HYDRONIC, unchanged, tmp_path, "kept")).model()
    assert _three_way_valve_set_points(model) == ["363.15"]
    assert not [warning for warning in found if "WORKAROUND APPLIED" in str(warning.message)]


def test_wrong_hydronic_layout_can_be_made_an_error(tmp_path: Path) -> None:
    with warnings.catch_warnings():
        warnings.simplefilter("error", HydronicLayoutWarning)
        with pytest.raises(HydronicLayoutWarning, match="THREE_WAY_VALVE:001: WRONG HYDRONIC LAYOUT"):
            convert_network("strict", _write(HYDRONIC, unchanged, tmp_path, "strict"))


def test_mixing_valve_keeps_its_set_point(tmp_path: Path) -> None:
    """With the pump after the valve, the valve mixes and holds its set point whatever the supply."""

    def mixing_at_95(model: Model) -> None:
        _mixing_circuit(model)
        _boiler_control(model).setdefault("parameters", {})["tsup_nominal"] = 368.15

    with warnings.catch_warnings():
        warnings.simplefilter("error", HydronicLayoutWarning)
        model = convert_network("mixing", _write(HYDRONIC, mixing_at_95, tmp_path, "mixing")).model()
    assert _three_way_valve_set_points(model) == ["363.15"]


def test_given_valve_set_point_is_used(tmp_path: Path) -> None:
    change = _set(
        lambda m: system(m, "THREE_WAY_VALVE:001")["control"],
        "three_way_valve_control",
        {"id": "THREE_WAY_VALVE_CONTROL:001", "parameters": {"supply_temperature_setpoint": 373.15}},
    )
    model = convert_network("given", _write(HYDRONIC, change, tmp_path, "given")).model()
    assert _three_way_valve_set_points(model) == ["373.15"]


TUTORIALS = Path(__file__).parent / "tutorials"


def test_mixing_circuit_tutorial_translates_without_warning() -> None:
    """The pump after each three-way valve: the layout the warning points to translates without any warning."""
    with warnings.catch_warnings():
        warnings.simplefilter("error", ModelWarning)
        model = convert_network("mixing", TUTORIALS / "three_zones_hydronic_mixing.yaml").model()
    connections = re.sub(r"\s", "", model)
    for circuit in ("001", "002"):
        assert f"connect(three_way_valve_{circuit}.port_2,temperature_sensor_{circuit}.port_a)" in connections
        assert f"connect(temperature_sensor_{circuit}.port_b,pump_{circuit}.port_a)" in connections
    assert _three_way_valve_set_points(model) == ["343.15", "343.15"]


@pytest.mark.parametrize("tutorial", ["three_zones_hydronic_heating.yaml", "multizones_with_pv.yaml"])
def test_tutorials_with_the_pump_before_the_valves_warn(tutorial: str) -> None:
    with pytest.warns(HydronicLayoutWarning, match="WRONG HYDRONIC LAYOUT"):
        convert_network("diverting", TUTORIALS / tutorial)


def _heat_pump(model: Model) -> None:
    _boiler(model)["variant"] = "air_water_heat_pump"


def _radiator_rating(model: str) -> tuple[str, str]:
    radiator = re.search(r"radiator_003\((.*?)\)", model.replace(" ", "").replace("\n", ""))
    assert radiator is not None
    found = dict(re.findall(r"(T_a_nominal|T_b_nominal)=([0-9.]+)", radiator.group(1)))
    return found["T_a_nominal"], found["T_b_nominal"]


def _radiator_warnings(found: list[warnings.WarningMessage]) -> list[str]:
    return [str(warning.message) for warning in found if "RADIATOR:003" in str(warning.message)]


def test_default_radiator_on_a_heat_pump_is_reported_and_kept(tmp_path: Path) -> None:
    """The default rating (80/60 degC, for a boiler) on a heat pump at 50 degC: the radiator delivers under half of
    its 2500 W. The model is left as written; the warning gives the rating for the heat pump."""
    with warnings.catch_warnings(record=True) as found:
        warnings.simplefilter("always")
        model = convert_network("heat_pump", _write(HYDRONIC, _heat_pump, tmp_path, "heat_pump")).model()
    [message] = _radiator_warnings(found)
    assert re.search(
        r"RADIATOR DOES NOT FIT THE HEAT PUMP: .*353.15/333.15 K \(the default, for a boiler\).*"
        r"BOILER:001 supplies at most 323.15 K.*\(4\d% of its nominal 2500 W\).*"
        r"water_inlet_temperature_at_nominal_condition: 323.15, water_outlet_temperature_at_nominal_condition: 313.15",
        message,
    )
    assert "VALVE:003" not in message  # its 0.06 kg/s already passes the 0.0598 kg/s of 2500 W over 10 K
    assert _radiator_rating(model) == ("353.15", "333.15")


def test_valve_too_small_for_the_heat_pump_rating_is_reported(tmp_path: Path) -> None:
    def larger_radiator(model: Model) -> None:
        _heat_pump(model)
        parameters(_radiator, nominal_heating_power_positive_for_heating=5000)(model)

    with pytest.warns(ModelWarning, match=r"RADIATOR:003: .*m_flow_nominal: 0.1195 on VALVE:003 for 5000 W \(now 0.06"):
        convert_network("larger", _write(HYDRONIC, larger_radiator, tmp_path, "larger"))


def test_recommendation_follows_the_heat_pump_set_point(tmp_path: Path) -> None:
    def low_temperature(model: Model) -> None:
        _heat_pump(model)
        parameters(_boiler, temperature_supply_setpoint_heat_pump=308.15)(model)

    with pytest.warns(ModelWarning, match=r"RADIATOR:003: .*at most 308.15 K.*nominal_condition: 298.15"):
        convert_network("low", _write(HYDRONIC, low_temperature, tmp_path, "low"))


def test_given_rating_above_the_heat_pump_is_reported_as_given(tmp_path: Path) -> None:
    def old_radiator(model: Model) -> None:
        _heat_pump(model)
        parameters(
            _radiator,
            water_inlet_temperature_at_nominal_condition=348.15,
            water_outlet_temperature_at_nominal_condition=338.15,
        )(model)

    with pytest.warns(ModelWarning, match=r"RADIATOR:003: .*348.15/338.15 K \(as given\)"):
        model = convert_network("old", _write(HYDRONIC, old_radiator, tmp_path, "old")).model()
    assert _radiator_rating(model) == ("348.15", "338.15")


def test_radiator_rated_for_the_heat_pump_is_not_reported(tmp_path: Path) -> None:
    def rated(model: Model) -> None:
        _heat_pump(model)
        parameters(
            _radiator,
            water_inlet_temperature_at_nominal_condition=323.15,
            water_outlet_temperature_at_nominal_condition=313.15,
        )(model)

    with warnings.catch_warnings(record=True) as found:
        warnings.simplefilter("always")
        convert_network("rated", _write(HYDRONIC, rated, tmp_path, "rated"))
    assert _radiator_warnings(found) == []


def test_radiator_fed_by_a_boiler_is_not_reported(tmp_path: Path) -> None:
    with warnings.catch_warnings(record=True) as found:
        warnings.simplefilter("always")
        convert_network("boiler", _write(HYDRONIC, unchanged, tmp_path, "boiler"))
    assert _radiator_warnings(found) == []
