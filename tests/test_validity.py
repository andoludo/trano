import tempfile
from pathlib import Path

import pytest

from tests.conftest import clean_model, _read
from trano.data_models.conversion import convert_network
from trano.elements.containers import containers_factory
from trano.exceptions import IncompatiblePortsError, WrongSystemFlowError
from trano.library.library import Library
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


@pytest.mark.parametrize("library_name", ["IDEAS", "Buildings"])
@pytest.mark.run(order=22)
def test_three_zones_hydronic_template(library_name: str) -> None:
    house = get_path("three_zones_hydronic.yaml")
    network = convert_network(
        "three_zones_hydronic", house, library=Library.from_configuration(library_name)
    )
    assert clean_model(network.model(), f"{network.name}_{library_name}_yaml") == set(
        _read(f"{network.name}_{library_name}_yaml")
    )


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


@pytest.mark.run(order=23)
def test_single_zone_hydronic_template(schema: Path) -> None:
    house = get_path("single_zone_hydronic.yaml")
    network = convert_network("single_zone_hydronic", house)
    assert clean_model(network.model(), f"{network.name}_yaml") == set(
        _read(f"{network.name}_yaml")
    )


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


@pytest.mark.run(order=24)
def test_single_zone_hydronic_weather_template(schema: Path) -> None:
    house = get_path("single_zone_hydronic_weather.yaml")
    network = convert_network("single_zone_hydronic_weather", house)
    assert clean_model(network.model(), f"{network.name}_yaml") == set(
        _read(f"{network.name}_yaml")
    )


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


@pytest.mark.run(order=25)
def test_single_zone_air_handling_unit_simple_vav_control_template(
    schema: Path,
) -> None:
    house = get_path("single_zone_air_handling_unit_simple_vav_control.yaml")
    network = convert_network("single_zone_air_handling_unit_simple_vav_control", house)
    assert clean_model(network.model(), f"{network.name}_yaml") == set(
        _read(f"{network.name}_yaml")
    )


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


@pytest.mark.run(order=26)
def test_single_zone_air_handling_unit_complex_vav_template(schema: Path) -> None:
    house = get_path("single_zone_air_handling_unit_complex_vav.yaml")
    network = convert_network("single_zone_air_handling_unit_complex_vav", house)
    assert clean_model(network.model(), f"{network.name}_yaml") == set(
        _read(f"{network.name}_yaml")
    )


@pytest.mark.run(order=27)
def test_two_zones_template(schema: Path) -> None:
    house = get_path("two_zones.yaml")
    network = convert_network("two_zones", house)
    assert clean_model(network.model(), f"{network.name}_yaml") == set(
        _read(f"{network.name}_yaml")
    )


@pytest.mark.run(order=28)
def test_two_zones_ideas_template(schema: Path) -> None:
    house = get_path("two_zones_ideas.yaml")
    network = convert_network("two_zones_ideas", house)
    assert clean_model(network.model(), f"{network.name}_yaml") == set(
        _read(f"{network.name}_yaml")
    )


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


@pytest.mark.run(order=29)
def test_single_zone_air_handling_unit_without_vav_with_duct_template(
    schema: Path,
) -> None:
    house = get_path("single_zone_air_handling_unit_without_vav_with_duct.yaml")
    # TODO: remove ducts here
    network = convert_network(
        "single_zone_air_handling_unit_without_vav_with_duct", house
    )
    assert clean_model(network.model(), f"{network.name}_yaml") == set(
        _read(f"{network.name}_yaml")
    )


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


@pytest.mark.parametrize("library_name", ["IDEAS"])
def test_three_zones_hydronic_with_containers(schema: Path, library_name: str) -> None:
    house = get_path("three_zones_hydronic_containers.yaml")
    network = convert_network(
        "three_zones_hydronic_containers",
        house,
        library=Library.from_configuration(library_name),
    )
    model_ = network.model(include_container=True)
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_containers():
    import json
    path_ = Path("/home/aan/Documents/trano/trano/elements/config/containers.json")
    contaienrs = containers_factory()
    path_.write_text(json.dumps(contaienrs.model_dump(exclude_unset=True, exclude_defaults=True, exclude_none=True), indent=4))