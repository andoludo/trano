import tempfile
from pathlib import Path

import pytest

from trano.data_models.conversion import convert_network
from trano.exceptions import IncompatiblePortsError, WrongSystemFlowError
from trano.elements.library.library import Library
from trano.simulate.simulate import SimulationOptions, simulate
from trano.utils.utils import is_success


def get_path(file_name: str) -> Path:
    return Path(__file__).parent.joinpath("models", file_name)


@pytest.mark.parametrize("library_name", ["IDEAS", "Buildings"])
@pytest.mark.simulate
def test_three_zones_hydronic(schema: Path, library_name: str) -> None:
    house = get_path("three_zones_hydronic.yaml")
    network = convert_network(
        "three_zones_hydronic", house, library=Library.from_configuration(library_name)
    )
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


@pytest.mark.simulate
def test_single_zone_hydronic(schema: Path) -> None:
    house = get_path("single_zone_hydronic.yaml")
    network = convert_network("single_zone_hydronic", house)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


@pytest.mark.simulate
def test_single_zone_hydronic_weather(schema: Path) -> None:
    house = get_path("single_zone_hydronic_weather.yaml")
    network = convert_network("single_zone_hydronic_weather", house)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


@pytest.mark.simulate
def test_single_zone_air_handling_unit_simple_vav_control(schema: Path) -> None:
    house = get_path("single_zone_air_handling_unit_simple_vav_control.yaml")
    network = convert_network("single_zone_air_handling_unit_simple_vav_control", house)

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        options = SimulationOptions(
            end_time=3600, check_only=True
        )  # TODO: why simulation fails
        results = simulate(
            Path(project_path),
            network,
            options=options,
        )
        assert is_success(results, options=options)


@pytest.mark.simulate
def test_single_zone_air_handling_unit_complex_vav(schema: Path) -> None:
    house = get_path("single_zone_air_handling_unit_complex_vav.yaml")
    network = convert_network("single_zone_air_handling_unit_complex_vav", house)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_single_zone_air_handling_unit_wrong_flow(schema: Path) -> None:
    house = get_path("single_zone_air_handling_unit_wrong_flow.yaml")
    network = convert_network("single_zone_air_handling_unit_wrong_flow", house)
    with pytest.raises(WrongSystemFlowError):
        network.model()


@pytest.mark.simulate
def test_single_zone_air_handling_unit_without_vav_with_duct(schema: Path) -> None:
    house = get_path("single_zone_air_handling_unit_without_vav_with_duct.yaml")
    # TODO: remove ducts here
    network = convert_network(
        "single_zone_air_handling_unit_without_vav_with_duct", house
    )

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        options = SimulationOptions(end_time=3600, check_only=True)
        results = simulate(
            Path(project_path),
            network,
            options=options,  # TODO: investigate why simulation fails
        )
        assert is_success(results, options=options)


@pytest.mark.parametrize(
    "file_name",
    [
        "single_zone_hydronic_unidentified_paramer",
        "single_zone_hydronic_unknown_id",
        "single_zone_hydronic_unknown_system",
        "single_zone_air_handling_unit_without_vav",
        "single_zone_air_handling_unit",
        "hello_world_missing_space_parameters",
    ],
)
def test_unexpected_configuration(schema: Path, file_name: str) -> None:
    house = get_path(f"{file_name}.yaml")
    with pytest.raises((ValueError, KeyError, IncompatiblePortsError)):
        network = convert_network(file_name, house)
        network.model()


@pytest.mark.parametrize(
    "file_name",
    [
        "single_zone_hydronic_wrong_flow",
        "single_zone_hydronic_random_id",
    ],
)
def test_unexpected_configuration_should_fail_but_pass_(
    schema: Path, file_name: str
) -> None:
    # TODO: this is to be checked
    house = get_path(f"{file_name}.yaml")
    network = convert_network(file_name, house)
    network.model()


# COntailer tests


def test_print():
    tt = '\nreplaceable package Medium = IDEAS.Media.Air(extraPropertiesNames={"CO2"})\nconstrainedby Modelica.Media.Interfaces.PartialMedium\n"Medium in the component"\nannotation (choicesAllMatching = true);\n  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[#heatPortCon#] heatPortCon\n    "Nodes for convective heat gains"\n    annotation (Placement(transformation(extent={{90,40},{110,60}}),\n        iconTransformation(extent={{90,40},{110,60}})));\n          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[#heatPortCon1#]  heatPortCon1\n"Nodes for convective heat gains"\nannotation (Placement(transformation(extent={{90,40},{110,60}}),\n    iconTransformation(extent={{-4,98},{6,108}})));\n  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[#heatPortRad#] heatPortRad\n    "Nodes for radiative heat gains"\n    annotation (Placement(transformation(extent={{90,-62},{110,-42}}),\n        iconTransformation(extent={{90,-62},{110,-42}})));\n              three_zones_hydronic_containers.Trano.Controls.BaseClasses.DataBus\n                                                 dataBus annotation (Placement(\n    transformation(extent={{-120,52},{-80,92}}),  iconTransformation(extent\n      ={{-228,58},{-208,78}})));\n        Modelica.Fluid.Interfaces.FluidPorts_b[#ports_b#] ports_b(redeclare package Medium =\n    Medium) annotation (Placement(\n        transformation(extent={{-108,-38},{-88,42}}), iconTransformation(extent\n          ={{-108,-38},{-88,42}})));\n  annotation (\n    Icon(\n      coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),\n        graphics={Rectangle(\n          extent={{-60,100},{60,-100}},\n          lineColor={255,128,0},\n          fillColor={215,215,215},\n          fillPattern=FillPattern.Forward)}));\n             '
    a = 12
