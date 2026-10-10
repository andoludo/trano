"""The schema maps every parameter to the name each library takes, and leaves the rest optional."""

import logging

import pytest

from trano.elements.library.parameters import (
    LIBRARIES,
    PARAMETERS,
    LibraryMapping,
    ParameterSpec,
    library_parameters,
    param_from_config,
    parameter_specs,
)

SpaceParameter = param_from_config("Space")
OccupancyParameters = param_from_config("Occupancy")
PumpParameters = param_from_config("Pump")
BoilerParameters = param_from_config("Boiler")
assert SpaceParameter and OccupancyParameters and PumpParameters and BoilerParameters


def test_a_mapping_is_a_name_null_or_options() -> None:
    assert LibraryMapping.parse("hZone") == LibraryMapping(name="hZone")
    assert LibraryMapping.parse(None) == LibraryMapping()
    assert LibraryMapping.parse({"name": "ACH", "variants": ["infiltration"]}).variants == ("infiltration",)
    with pytest.raises(TypeError):
        LibraryMapping.parse(3)


def test_a_spec_rejects_an_unknown_library() -> None:
    with pytest.raises(ValueError, match="unknown libraries"):
        ParameterSpec.from_attribute("x", {"range": "float", "libraries": {"modelica": "x"}})


@pytest.mark.parametrize("model", sorted(set(PARAMETERS.values()), key=lambda model: model.__name__))
def test_no_two_parameters_share_a_modelica_name_in_a_library(model) -> None:  # noqa: ANN001
    for library in LIBRARIES:
        names = [
            spec.mapping(library).name
            for spec in parameter_specs(model).values()
            if spec.render and spec.mapping(library).name is not None
        ]
        assert len(names) == len(set(names)), (model.__name__, library, names)


def test_the_space_renders_each_library_s_names() -> None:
    parameters = SpaceParameter(floor_area=48, average_room_height=2.7)

    assert library_parameters(parameters, "Buildings") == {
        "AFlo": 48.0,
        "hRoo": 2.7,
        "linearizeRadiation": "true",
        "m_flow_nominal": 0.01,
        "mSenFac": 1.0,
        "T_start": 294.15,
    }
    assert library_parameters(parameters, "IDEAS") == {
        "hZone": 2.7,
        "mSenFac": 1.0,
        "T_start": 294.15,
        "V": pytest.approx(129.6),
    }
    assert library_parameters(parameters, "reduced_order") == {"AZone": 48.0, "VAir": pytest.approx(129.6)}
    assert library_parameters(parameters, "iso_13790") == {"AFlo": 48.0, "VRoo": pytest.approx(129.6)}


def test_an_unset_parameter_is_the_library_default() -> None:
    assert "ACH" not in library_parameters(SpaceParameter(), "Buildings", "infiltration")
    assert "ventilationSchedule" not in library_parameters(SpaceParameter(), "Buildings", "infiltration")


def test_a_variant_bound_parameter_only_renders_for_its_variant(caplog: pytest.LogCaptureFixture) -> None:
    parameters = SpaceParameter(ach=0.5)

    assert library_parameters(parameters, "Buildings", "infiltration")["ACH"] == 0.5
    with caplog.at_level(logging.WARNING):
        rendered = library_parameters(parameters, "Buildings", "default")
    assert "ACH" not in rendered
    assert "ach is not taken by the Buildings library for the default variant" in caplog.text


def test_a_parameter_taken_another_way_is_neither_rendered_nor_warned(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.WARNING):
        rendered = library_parameters(SpaceParameter(ach=0.5, floor_area=30), "IDEAS", "infiltration")
    assert "n50" not in rendered and "ACH" not in rendered and "AFlo" not in rendered
    assert caplog.text == ""


def test_a_when_given_parameter_renders_only_when_set_away_from_its_default() -> None:
    assert "m_flow_nominal" not in library_parameters(SpaceParameter(), "IDEAS")
    assert "m_flow_nominal" not in library_parameters(SpaceParameter(nominal_mass_flow_rate=0.01), "IDEAS")
    assert library_parameters(SpaceParameter(nominal_mass_flow_rate=0.2), "IDEAS")["m_flow_nominal"] == 0.2


def test_a_parameter_the_library_does_not_take_is_dropped_with_a_warning(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.WARNING):
        rendered = library_parameters(SpaceParameter(average_room_height=3.0), "iso_13790")
    assert "hRoo" not in rendered
    assert "average_room_height is not taken by the iso_13790 library" in caplog.text


def test_trano_only_inputs_are_never_rendered() -> None:
    rendered = library_parameters(BoilerParameters(dt_boi_nominal=15), "Buildings")

    assert "dTBoi_nominal" not in rendered
    assert rendered["nominal_mass_flow_rate_boiler"] == pytest.approx(1.5 * 20000 / 15 / 4200)


def test_a_deprecated_parameter_warns_when_given(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.WARNING):
        PumpParameters(constant_input_set_point=1.0)
        OccupancyParameters()
    assert "constant_input_set_point is deprecated" in caplog.text
    assert "ach is deprecated" not in caplog.text
    assert "constInput" not in library_parameters(PumpParameters(constant_input_set_point=1.0), "Buildings")


def test_the_occupancy_floor_area_belongs_to_the_co2_variant() -> None:
    parameters = OccupancyParameters(floor_area=30)

    assert library_parameters(parameters, "Buildings", "co2")["AFlo"] == 30.0
    assert "AFlo" not in library_parameters(parameters, "Buildings", "default")


def test_unknown_names_pass_through_as_modelica_modifiers() -> None:
    assert library_parameters(SpaceParameter(hIntFixed=4.0), "Buildings")["hIntFixed"] == 4.0


def test_mpc_only_classes_take_no_modelica_library() -> None:
    battery = param_from_config("Battery")
    assert battery is not None
    assert library_parameters(battery(), "Buildings") == {}
    assert library_parameters(battery(), "mpc")["capacity"] == 10.0
