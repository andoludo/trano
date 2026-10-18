"""The simulation runtimes: OpenModelica in a sibling container (docker) or on this machine (local)."""

import shutil
from pathlib import Path, PurePosixPath
from types import SimpleNamespace

import docker  # type: ignore[import-untyped]
import pytest

from tests.fixtures.fake_omc import FakeOmc
from tests.fixtures.simple_space_1 import simple_space_1_fixture
from trano.data_models.conversion import convert_network
from trano.elements import Weather, param_from_config
from trano.elements.library.library import Library
from trano.exceptions import OpenModelicaNotFoundError
from trano.simulate.result import SimulationErrorKind, SimulationResult
from trano.simulate.simulate import (
    CONTAINER_MODEL_DIR,
    RUNTIME_VARIABLE,
    SimulationOptions,
    SimulationRuntime,
    container,
    create_mos_file,
    model_directory,
    resolve_runtime,
    simulate,
    unique_container_name,
)
from trano.topology import Network
from trano.utils.utils import is_success

TUTORIALS = Path(__file__).parent / "tutorials"
MODEL = "package given AixLib.ThermalZones end given;"
# The script of the docker runtime, as trano 0.52.0 wrote it: existing users must get exactly this.
DOCKER_SCRIPT = """
    getVersion();
    loadModel(Modelica, {"4.1.0"});
    loadModel(IDEAS, {"3.0.0"});
    loadModel(AixLib, {"3.0.1", "default"});
    loadFile("/simulation/MODEL.mo");
    checkModel(given.building);
    simulate(given.building, startTime = 0, stopTime = 3600, tolerance = 0.0001);
    getErrorString();
    """


def given_network() -> Network:
    return Network(name="given", library=Library.from_configuration("Buildings"))


def mos_script(project_path: Path, **arguments: object) -> str:
    with create_mos_file(
        given_network(), SimulationOptions(end_time=3600), project_path, model=MODEL, **arguments
    ) as mos:  # type: ignore[arg-type]
        model_file = next(path.name for path in project_path.iterdir() if path.suffix == ".mo")
        return project_path.joinpath(mos).read_text().replace(model_file, "MODEL.mo")


def test_the_docker_script_is_unchanged(tmp_path: Path) -> None:
    assert mos_script(tmp_path) == DOCKER_SCRIPT


def test_the_local_script_loads_the_model_from_the_project(tmp_path: Path) -> None:
    project_path = model_directory(SimulationRuntime.local, tmp_path)

    script = mos_script(tmp_path, model_dir=project_path)

    assert f'loadFile("{tmp_path.resolve().as_posix()}/MODEL.mo");' in script
    assert "/simulation" not in script


def test_the_runtime_defaults_to_docker_and_follows_the_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(RUNTIME_VARIABLE, raising=False)
    assert resolve_runtime() is SimulationRuntime.docker
    monkeypatch.setenv(RUNTIME_VARIABLE, "LOCAL")
    assert resolve_runtime() is SimulationRuntime.local
    assert resolve_runtime("docker") is SimulationRuntime.docker, "an explicit runtime wins over the environment"
    monkeypatch.setenv(RUNTIME_VARIABLE, "podman")
    with pytest.raises(ValueError, match=RUNTIME_VARIABLE):
        resolve_runtime()


def test_model_directory_as_seen_by_omc(tmp_path: Path) -> None:
    assert model_directory(SimulationRuntime.docker, tmp_path) == PurePosixPath("/simulation")
    assert model_directory(SimulationRuntime.local, tmp_path) == tmp_path.resolve()


def network_with_weather(path: Path) -> tuple[Network, Weather]:
    network = given_network()
    weather = Weather(parameters=param_from_config("Weather")(path=str(path)))
    network.add_boiler_plate_spaces([simple_space_1_fixture()], weather=weather)
    return network, weather


@pytest.mark.parametrize("runtime", list(SimulationRuntime))
def test_local_weather_files_are_copied_and_rewritten(tmp_path: Path, runtime: SimulationRuntime) -> None:
    weather_file = tmp_path / "site.mos"
    weather_file.write_text("#1\n")
    project = tmp_path / "project"
    project.mkdir()
    network, weather = network_with_weather(weather_file)

    network.set_weather_path(project, model_directory(runtime, project))

    expected = "/simulation" if runtime is SimulationRuntime.docker else project.resolve().as_posix()
    assert weather.parameters.path == f'"{expected}/site.mos"'  # type: ignore[union-attr]
    assert project.joinpath("site.mos").read_text() == "#1\n"


def test_a_weather_file_already_in_the_project_is_kept(tmp_path: Path) -> None:
    weather_file = tmp_path / "site.mos"
    weather_file.write_text("#1\n")
    network, weather = network_with_weather(weather_file)

    network.set_weather_path(tmp_path, tmp_path)

    assert weather.parameters.path == f'"{tmp_path.as_posix()}/site.mos"'  # type: ignore[union-attr]


def test_the_container_path_helper_is_deprecated(tmp_path: Path) -> None:
    weather_file = tmp_path / "site.mos"
    weather_file.write_text("#1\n")
    project = tmp_path / "project"
    project.mkdir()
    network, weather = network_with_weather(weather_file)

    with pytest.warns(DeprecationWarning, match="set_weather_path"):
        network.set_weather_path_to_container_path(project)

    assert weather.parameters.path == f'"{CONTAINER_MODEL_DIR}/site.mos"'  # type: ignore[union-attr]


def test_every_run_gets_its_own_container() -> None:
    names = {unique_container_name() for _ in range(100)}
    assert len(names) == 100
    assert all(name.startswith("trano-") for name in names)


class Containers:
    """The containers of a docker client that records the names it is asked to run, and runs none."""

    def __init__(self) -> None:
        self.names: list[str] = []

    def get(self, name: str) -> None:
        raise docker.errors.NotFound(name)

    def run(self, *_: object, name: str, **__: object) -> None:
        self.names.append(name)
        raise RuntimeError("stop here")


def test_the_container_is_named_uniquely_by_default(tmp_path: Path) -> None:
    client = SimpleNamespace(containers=Containers())

    for _ in range(2):
        with pytest.raises(RuntimeError, match="stop here"), container(client, tmp_path):
            pass

    first, second = client.containers.names
    assert first != second and first.startswith("trano-")


def test_the_local_runtime_needs_omc(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("PATH", str(tmp_path))
    with pytest.raises(OpenModelicaNotFoundError, match=RUNTIME_VARIABLE):
        simulate(tmp_path, given_network(), model=MODEL, runtime=SimulationRuntime.local)


def test_the_local_runtime_runs_omc_in_the_results_directory(tmp_path: Path, fake_omc: FakeOmc) -> None:
    phases: list[str] = []

    result = simulate(tmp_path, given_network(), model=MODEL, runtime="local", on_phase=phases.append)

    assert isinstance(result, SimulationResult)
    assert result.success and is_success(result)
    assert result.result_path == tmp_path.resolve() / "results" / "given.building_res.mat"
    assert result.result_path.exists()
    assert phases == ["compiling", "simulating"]
    record = fake_omc.record
    assert record["cwd"] == str(tmp_path.resolve() / "results")
    script = str(record["script"])
    assert f'loadFile("{tmp_path.resolve().as_posix()}/' in script
    assert "configure.mos" not in script
    assert not (tmp_path / "configure.mos").exists(), "the local runtime does not install libraries"


def test_a_failed_local_run_reports_why(tmp_path: Path, fake_omc: FakeOmc) -> None:
    fake_omc.replay("translation_error.log")

    result = simulate(tmp_path, given_network(), model=MODEL, runtime="local")

    assert isinstance(result, SimulationResult)
    assert not result.success and result.result_path is None
    assert result.error_kind is SimulationErrorKind.translation
    assert result.error_message == "Variable undefinedThing not found in scope building."
    assert "Failed to build model" in result.log


def test_a_local_check_reports_the_errors_of_the_check(tmp_path: Path, fake_omc: FakeOmc) -> None:
    simulate(tmp_path, given_network(), SimulationOptions(check_only=True), model=MODEL, runtime="local")

    script = str(fake_omc.record["script"])
    assert "simulate(" not in script
    assert script.rstrip().endswith("checkModel(given.building);\n    getErrorString();")


def test_a_local_run_is_stopped_after_its_timeout(tmp_path: Path, fake_omc: FakeOmc) -> None:
    fake_omc.replay("success.log", sleep_s=30)

    result = simulate(tmp_path, given_network(), model=MODEL, runtime="local", timeout_s=0.5)

    assert isinstance(result, SimulationResult)
    assert result.error_kind is SimulationErrorKind.timeout
    assert result.duration_s < 10


@pytest.mark.simulation
@pytest.mark.skipif(shutil.which("omc") is None, reason="OpenModelica's omc is not on the PATH")
def test_first_model_simulates_one_day_with_the_local_runtime(tmp_path: Path) -> None:
    network = convert_network("first_model", TUTORIALS / "first_model.yaml")

    result = simulate(tmp_path, network, SimulationOptions(end_time=24 * 3600), runtime=SimulationRuntime.local)

    assert isinstance(result, SimulationResult)
    assert result.success, result.error_message
    assert result.result_path == tmp_path.resolve() / "results" / "first_model.building_res.mat"
    assert result.result_path.exists()
