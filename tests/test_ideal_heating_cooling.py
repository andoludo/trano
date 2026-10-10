"""The ideal heating and cooling element drives a zone towards scheduled set points without a control element."""

import re

import pytest

from tests.fixtures.simple_space_1 import simple_space_1_fixture
from tests.golden import remove_trano_package
from trano.elements import param_from_config
from trano.elements.library.library import Library
from trano.elements.system import IdealHeatingCooling
from trano.topology import Network

Parameters = param_from_config("IdealHeatingCooling")
SETBACK = "[0, 283.15; 25200, 283.15; 28800, 293.15; 82800, 293.15; 82800, 283.15; 86400, 283.15]"


def generated_model(library: str, **parameters: str | float) -> str:
    network = Network(name=f"{library}_ideal", library=Library.from_configuration(library))
    space = simple_space_1_fixture()
    space.emissions = [IdealHeatingCooling(name="hvac", parameters=Parameters(**parameters))]
    network.add_boiler_plate_spaces([space])
    return remove_trano_package(network.model())


def declaration(model: str) -> str:
    start = model.index("IdealHeatingSystem.IdealHeatingCooling")
    return re.sub(r"\s+", " ", model[start : model.index("annotation", start)])


def test_defaults_are_a_constant_dual_set_point_without_radiative_share() -> None:
    hvac = declaration(generated_model("Buildings"))

    assert "TSetHea=[0, 293.15]" in hvac and "TSetCoo=[0, 300.15]" in hvac
    assert "QHea_flow_max=1000000.0" in hvac and "QCoo_flow_max=1000000.0" in hvac
    assert "k=0.1, Ti=300.0" in hvac and "frad=0.0" in hvac


def test_schedules_and_capacities_come_from_the_parameters() -> None:
    hvac = declaration(
        generated_model("Buildings", heating_setpoint_schedule=SETBACK, maximum_cooling_power=0, radiative_fraction=0.4)
    )

    assert f"TSetHea={SETBACK}" in hvac
    assert "QCoo_flow_max=0.0" in hvac and "frad=0.4" in hvac


@pytest.mark.parametrize("library", ["Buildings", "IDEAS"])
def test_both_heat_ports_reach_the_zone(library: str) -> None:
    model = generated_model(library)

    assert re.search(r"connect\(hvac\.heatPortCon,\s*heatPortCon\[1\]\)", model)
    assert re.search(r"connect\(hvac\.heatPortRad,\s*heatPortRad\[1\]\)", model)
    assert "Control" not in declaration(model)


def test_the_model_is_part_of_the_trano_package() -> None:
    network = Network(name="package", library=Library.from_configuration("Buildings"))
    space = simple_space_1_fixture()
    space.emissions = [IdealHeatingCooling(name="hvac")]
    network.add_boiler_plate_spaces([space])
    model = network.model()

    assert "model IdealHeatingCooling" in model
    assert "Modelica.Blocks.Continuous.LimPID conHea" in model
    assert "extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic" in model
