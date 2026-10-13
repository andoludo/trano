"""Opt-in performance options: declarations rewritten per library, defaults leaving the model untouched."""

import re
from pathlib import Path

import pytest

from tests.golden import remove_trano_package
from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.performance import PerformanceOptions, apply_performance_options
from trano.performance.declarations import ACTUATOR_FILTER, MOVER_FILTER, SENSOR_DYNAMICS, apply_rule
from validation.bestest.harness import normalized_model

MODELS = Path(__file__).parent.joinpath("models")
FAST = PerformanceOptions(ideal_actuators=True, steady_state_sensors=True)


@pytest.mark.parametrize(
    ("declaration", "expected"),
    [
        (
            "Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage val(m_flow_nominal=1);",
            "Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage val(use_strokeTime=false, m_flow_nominal=1);",
        ),
        (
            "IDEAS.Fluid.Actuators.Valves.TwoWayEqualPercentage val(m_flow_nominal=1);",
            "IDEAS.Fluid.Actuators.Valves.TwoWayEqualPercentage val(use_inputFilter=false, m_flow_nominal=1);",
        ),
        (
            "AixLib.Fluid.Actuators.Dampers.Exponential dam;",
            "AixLib.Fluid.Actuators.Dampers.Exponential dam(use_strokeTime=false);",
        ),
        (
            "Buildings.Fluid.Actuators.Valves.ThreeWayLinear val[2]();",
            "Buildings.Fluid.Actuators.Valves.ThreeWayLinear val[2](each use_strokeTime=false);",
        ),
        (
            'Buildings.Fluid.Actuators.Valves.\n    TwoWayLinear val "Split class path";',
            'Buildings.Fluid.Actuators.Valves.\n    TwoWayLinear val(use_strokeTime=false) "Split class path";',
        ),
    ],
)
def test_actuator_filter_is_removed_with_the_parameter_of_each_library(declaration: str, expected: str) -> None:
    assert apply_rule(declaration, ACTUATOR_FILTER) == (expected, 1)


def test_mover_filter_covers_preconfigured_movers() -> None:
    text = "Buildings.Fluid.Movers.Preconfigured.SpeedControlled_y pum(redeclare package Medium = M);"
    assert apply_rule(text, MOVER_FILTER) == (
        "Buildings.Fluid.Movers.Preconfigured.SpeedControlled_y pum(use_riseTime=false, redeclare package Medium = M);",
        1,
    )


@pytest.mark.parametrize(
    "text",
    [
        # a value written by a template or the user wins
        "Buildings.Fluid.Sensors.TemperatureTwoPort sen(tau=10, m_flow_nominal=1);",
        "Buildings.Fluid.Sensors.TemperatureTwoPort sen(final tau=10);",
        # not declarations
        "extends Buildings.Fluid.Sensors.TemperatureTwoPort(m_flow_nominal=1);",
        "replaceable model M = Buildings.Fluid.Sensors.TemperatureTwoPort constrainedby "
        "Buildings.Fluid.Sensors.TemperatureTwoPort annotation(choicesAllMatching=true);",
        'Real x "see Buildings.Fluid.Sensors.TemperatureTwoPort sen for details";',
        "// Buildings.Fluid.Sensors.TemperatureTwoPort sen;",
        # flow sensors are steady-state already, and another library is not touched
        "Buildings.Fluid.Sensors.MassFlowRate sen;",
        "Modelica.Fluid.Sensors.TemperatureTwoPort sen;",
    ],
)
def test_declarations_left_alone(text: str) -> None:
    assert apply_rule(text, SENSOR_DYNAMICS) == (text, 0)


def test_nested_modifier_of_the_same_name_does_not_count_as_set() -> None:
    text = "Buildings.Fluid.Sensors.TemperatureTwoPort sen(per(tau=3));"
    assert apply_rule(text, SENSOR_DYNAMICS) == (
        "Buildings.Fluid.Sensors.TemperatureTwoPort sen(tau=0, per(tau=3));",
        1,
    )


def test_default_options_change_nothing() -> None:
    text = "Buildings.Fluid.Actuators.Valves.TwoWayLinear val;"
    assert apply_performance_options(text, PerformanceOptions()) == text


def test_options_are_strict() -> None:
    with pytest.raises(ValueError, match="Extra inputs"):
        PerformanceOptions(fast=True)  # type: ignore[call-arg]


def _yaml_with_performance(tmp_path: Path, source: str, performance: str) -> Path:
    path = tmp_path.joinpath(source)
    path.write_text(MODELS.joinpath(source).read_text() + f"\nperformance:\n{performance}")
    return path


@pytest.mark.parametrize("library", ["Buildings", "IDEAS"])
def test_yaml_options_rewrite_every_hydronic_declaration(tmp_path: Path, library: str) -> None:
    path = _yaml_with_performance(
        tmp_path, "single_zone_hydronic.yaml", "  ideal_actuators: true\n  steady_state_sensors: true\n"
    )
    network = convert_network("single_zone_hydronic", path, library=Library.from_configuration(library))
    assert network.performance == FAST
    model = network.model()
    valve = "use_inputFilter" if library == "IDEAS" else "use_strokeTime"
    assert re.search(rf"TwoWayEqualPercentage\s+valve_\w+\(\s*{valve}=false", model)
    assert re.search(r"SpeedControlled_y pumBoi\(use_riseTime=false", model)  # in the embedded Trano package
    assert re.search(r"TemperatureTwoPort temperature_sensor1\(tau=0", model)


def test_default_model_is_the_model_without_options() -> None:
    path = MODELS.joinpath("single_zone_hydronic.yaml")
    with_defaults = convert_network("single_zone_hydronic", path, performance=PerformanceOptions()).model()
    assert normalized_model(with_defaults) == normalized_model(convert_network("single_zone_hydronic", path).model())
    assert "use_strokeTime=false" not in with_defaults


def test_argument_replaces_the_yaml_options(tmp_path: Path) -> None:
    path = _yaml_with_performance(tmp_path, "single_zone_hydronic.yaml", "  ideal_actuators: true\n")
    network = convert_network("single_zone_hydronic", path, performance=PerformanceOptions())
    assert network.performance == PerformanceOptions()


def test_boiler_control_hysteresis_is_rendered_only_when_set(tmp_path: Path) -> None:
    default = remove_trano_package(
        convert_network("three_zones_hydronic", MODELS.joinpath("three_zones_hydronic.yaml")).model()
    )
    assert "dTHys" not in default
    source = MODELS.joinpath("three_zones_hydronic.yaml").read_text()
    path = tmp_path.joinpath("three_zones_hydronic.yaml")
    path.write_text(
        source.replace(
            "        boiler_control:\n",
            (
                "        boiler_control:\n          id: BOILER_CONTROL:001\n"
                "          parameters:\n            temperature_hysteresis: 1.0\n"
            ),
            1,
        )
    )
    model = convert_network("three_zones_hydronic", path).model()
    assert re.search(r"BoilerControl\w*\s+\w+\([^)]*dTHys=1\.0", remove_trano_package(model))
    assert "Greater lesThr(h=dTHys)" in model
