import platform
import shutil
import subprocess
import tempfile
from contextlib import contextmanager
from pathlib import Path
from typing import Generator, Optional

import docker  # type: ignore
from jinja2 import Environment
from pydantic import BaseModel, Field

from trano.exceptions import DockerNotInstalledError, DockerClientError
from trano.topology import Network


def check_docker_installed() -> None:
    try:
        subprocess.run(
            ["docker", "--version"],  # noqa: S603, S607
            check=True,
        )
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        raise DockerNotInstalledError(
            "Docker is not installed on the system. Simulation cannot be run."
        ) from e


def client() -> docker.DockerClient:
    check_docker_installed()
    system = platform.system()

    if system in ("Linux", "Darwin"):
        base_url = "unix:///var/run/docker.sock"
    elif system == "Windows":
        base_url = "npipe:////./pipe/docker_engine"
    else:
        raise NotImplementedError(f"Unsupported platform: {system}")
    try:
        client = docker.DockerClient(base_url=base_url)
    except Exception as e:
        raise DockerClientError(
            f"Docker client cannot be initialized with base url: '{base_url}'. "
            f"Simulation cannot be run on your {system} system."
        ) from e
    return client


class SimulationOptions(BaseModel):
    start_time: int = Field(default=0)
    end_time: int = Field(default=2 * 3600 * 24 * 7)
    check_only: bool = Field(default=False)
    tolerance: float = Field(default=1e-4)


class SimulationLibraryOptions(SimulationOptions):
    library_name: str = Field(default="Buildings")


def simulate(
    project_path: Path,
    model_network: Network,
    options: Optional[SimulationOptions] = None,
) -> docker.models.containers.ExecResult:
    client_ = client()
    options = options or SimulationOptions()
    with container(client_, project_path) as container_, create_mos_file(
        model_network, options, project_path
    ) as mos_file_name:
        results = container_.exec_run(cmd=f"omc /simulation/{mos_file_name}")
    return results


def stop_container(client: docker.DockerClient, container_name: str) -> None:
    try:
        container = client.containers.get(container_name)
        if container.attrs["State"]["Status"] == "running":
            container.stop()
        container.remove()
    except docker.errors.NotFound:
        pass


@contextmanager
def container(
    client: docker.DockerClient, project_path: Path
) -> Generator[docker.models.containers.Container, None, None]:
    container_name = "openmodelica"
    stop_container(client, container_name)
    container = client.containers.run(
        "openmodelica/openmodelica:v1.24.4-ompython",
        command="tail -f /dev/null",
        volumes=[
            f"{project_path}:/simulation",
            f"{project_path}/results:/results",
        ],
        detach=True,
        name=container_name,
    )
    configuration_path = Path(__file__).parent / "configure.mos"
    shutil.copy(configuration_path, project_path / "configure.mos")
    container.exec_run(cmd="chmod -R 777 /results")
    container.exec_run(cmd="chmod -R 777 /simulation")
    container.exec_run(cmd="omc /simulation/configure.mos")
    yield container
    container.exec_run(
        cmd='find / -name "*_res.mat" -exec cp {} /results \;'  # noqa: W605
    )
    container.stop()
    container.remove()


@contextmanager
def create_mos_file(
    network: Network, options: SimulationOptions, project_path: Path
) -> Generator[str, None, None]:
    # TODO: do we want this here?
    network.set_weather_path_to_container_path(project_path)
    model = network.model()
    with tempfile.NamedTemporaryFile(
        mode="w", dir=project_path, suffix=".mo"
    ) as temp_model_file, tempfile.NamedTemporaryFile(
        mode="w", dir=project_path, suffix=".mos"
    ) as temp_mos_file:
        Path(temp_model_file.name).write_text(model)
        environment = Environment(autoescape=True)
        if options.check_only:
            template = environment.from_string(
                """
    getVersion();
    loadFile("/simulation/{{model_file}}");
    checkModel({{model_name}}.building);
    """
            )
        else:
            template = environment.from_string(
                f"""
    getVersion();
    loadFile("/simulation/{{{{model_file}}}}");
    checkModel({{{{model_name}}}}.building);
    simulate({{{{model_name}}}}.building,startTime = {options.start_time},
    stopTime = {options.end_time},
    tolerance = {options.tolerance});
    """
            )
        mos_file = template.render(
            model_file=Path(temp_model_file.name).name, model_name=network.name
        )
        Path(temp_mos_file.name).write_text(mos_file)
        yield Path(temp_mos_file.name).name
