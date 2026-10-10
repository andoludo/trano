"""Weather files are copied into the simulation folder; library resources are referenced as they are."""

from pathlib import Path

import pytest

from tests.fixtures.simple_space_1 import simple_space_1_fixture
from trano.elements import Weather, param_from_config
from trano.elements.library.library import Library
from trano.topology import Network

WeatherParameters = param_from_config("Weather")
RESOURCE = 'Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/USA_CO_Denver.Intl.AP.725650_TMY3.mos")'


def network_with(path: str) -> tuple[Network, Weather]:
    network = Network(name="weather", library=Library.from_configuration("Buildings"))
    weather = Weather(parameters=WeatherParameters(path=path))
    network.add_boiler_plate_spaces([simple_space_1_fixture()], weather=weather)
    return network, weather


def test_modelica_resources_stay_untouched(tmp_path: Path) -> None:
    network, weather = network_with(RESOURCE)

    network.set_weather_path_to_container_path(tmp_path)

    assert weather.parameters.path == RESOURCE  # type: ignore[union-attr]
    assert list(tmp_path.iterdir()) == []


def test_local_files_are_copied_next_to_the_model(tmp_path: Path) -> None:
    weather_file = tmp_path.joinpath("site.mos")
    weather_file.write_text("#1\n")
    project = tmp_path.joinpath("project")
    project.mkdir()
    network, weather = network_with(str(weather_file))

    network.set_weather_path_to_container_path(project)

    assert weather.parameters.path == '"/simulation/site.mos"'  # type: ignore[union-attr]
    assert project.joinpath("site.mos").read_text() == "#1\n"


def test_missing_weather_file_is_reported(tmp_path: Path) -> None:
    network, _ = network_with(str(tmp_path.joinpath("nowhere-to-be-found.mos")))

    with pytest.raises(FileNotFoundError):
        network.set_weather_path_to_container_path(tmp_path)
