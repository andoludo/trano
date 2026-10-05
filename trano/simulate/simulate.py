import platform
import subprocess
import tempfile
from contextlib import contextmanager
from pathlib import Path
from collections.abc import Generator

import docker  # type: ignore
from pydantic import BaseModel, Field

from trano.exceptions import DockerNotInstalledError, DockerClientError
from trano.elements.jinja import STRING_ENVIRONMENT
from trano.topology import Network


def check_docker_installed() -> None:
    try:
        subprocess.run(
            ["docker", "--version"],  # noqa: S607
            check=True,
        )
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        raise DockerNotInstalledError("Docker is not installed on the system. Simulation cannot be run.") from e


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


class ModelicaEnvironment(BaseModel):
    """Versions of the OpenModelica image and of the Modelica libraries installed in it.

    Buildings 13 is built against Modelica 4.1 while IDEAS and AixLib still declare
    Modelica 4.0, so both Modelica Standard Library versions are installed side by side.

    IDEAS stays at 3.0.0: IDEAS 4.0.0 cannot be parsed by OpenModelica 1.25 and triggers
    internal compiler errors in OpenModelica 1.26 and 1.27 (expandable weather bus of
    SlabOnGround and of the zone lighting control, `coeffsCp` on surface arrays), which
    IDEAS' own examples reproduce.
    """

    openmodelica_image: str = Field(default="openmodelica/openmodelica:v1.26.9-ompython")
    modelica: list[str] = Field(default=["4.0.0+maint.om", "4.1.0+maint.om"])
    buildings: str = Field(default="13.0.0")
    ideas: str = Field(default="3.0.0")
    aixlib: str = Field(default="3.0.1")

    @property
    def modelica_version(self) -> str:
        """Modelica Standard Library version to load before a model: the newest one.

        Every generated model refers to Buildings (through the embedded Trano package) and
        possibly IDEAS or AixLib, whose `uses` annotations ask for different versions. Only
        one can be loaded, and the newer one declares itself compatible with the older.
        """
        return max(self.modelica, key=_version_key).split("+")[0]

    def configure_script(self) -> str:
        """Content of the OpenModelica script installing the libraries.

        A library already available on the MODELICAPATH (for instance baked into a custom
        image) is not installed again: that keeps the package manager, and the network, out
        of the loop and makes the container start in seconds instead of minutes.
        """
        lines = ["getVersion();"]
        lines += [
            _install_unless_available(
                "Modelica", version, ("ModelicaServices", "Modelica", "Complex"), exact_match=True
            )
            for version in self.modelica
        ]
        lines += [
            _install_unless_available("Buildings", self.buildings),
            _install_unless_available("IDEAS", self.ideas),
            _install_unless_available("AixLib", self.aixlib),
        ]
        return "\n".join(lines) + "\n"


def _version_key(version: str) -> tuple[int, ...]:
    return tuple(int(part) for part in version.split("+", maxsplit=1)[0].split("."))


def _install_unless_available(
    library: str, version: str, packages: tuple[str, ...] | None = None, exact_match: bool = False
) -> str:
    """OpenModelica statements installing `packages` unless `library` has `version` installed.

    The library name is checked before its versions: asking the versions of a library that
    is not installed aborts the script.
    """
    suffix = ", exactMatch=true" if exact_match else ""
    installs = " ".join(f'installPackage({package}, "{version}"{suffix});' for package in packages or (library,))
    library_present = f'max({{l == "{library}" for l in cat(1, {{""}}, getAvailableLibraries())}})'
    version_present = f'max({{v == "{version}" for v in cat(1, {{""}}, getAvailableLibraryVersions({library}))}})'
    return f"if {library_present} then\n  if not {version_present} then {installs} end if;\nelse\n  {installs}\nend if;"


MODELICA_ENVIRONMENT = ModelicaEnvironment()


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
    options: SimulationOptions | None = None,
) -> docker.models.containers.ExecResult:
    client_ = client()
    options = options or SimulationOptions()
    with (
        container(client_, project_path) as container_,
        create_mos_file(model_network, options, project_path) as mos_file_name,
    ):
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
    client: docker.DockerClient,
    project_path: Path,
    environment: ModelicaEnvironment = MODELICA_ENVIRONMENT,
) -> Generator[docker.models.containers.Container, None, None]:
    container_name = "openmodelica"
    stop_container(client, container_name)
    container = client.containers.run(
        environment.openmodelica_image,
        command="tail -f /dev/null",
        volumes=[
            f"{project_path}:/simulation",
            f"{project_path}/results:/results",
        ],
        detach=True,
        name=container_name,
    )
    (project_path / "configure.mos").write_text(environment.configure_script())
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
    network: Network,
    options: SimulationOptions,
    project_path: Path,
    environment: ModelicaEnvironment = MODELICA_ENVIRONMENT,
) -> Generator[str, None, None]:
    # TODO: do we want this here?
    network.set_weather_path_to_container_path(project_path)
    model = network.model()
    with (
        tempfile.NamedTemporaryFile(mode="w", dir=project_path, suffix=".mo") as temp_model_file,
        tempfile.NamedTemporaryFile(mode="w", dir=project_path, suffix=".mos") as temp_mos_file,
    ):
        Path(temp_model_file.name).write_text(model)
        simulation = (
            ""
            if options.check_only
            else f"""
    simulate({{{{model_name}}}}.building,startTime = {options.start_time},
    stopTime = {options.end_time},
    tolerance = {options.tolerance});"""
        )
        template = STRING_ENVIRONMENT.from_string(
            f"""
    getVersion();
    loadModel(Modelica, {{"{environment.modelica_version}"}});
    loadFile("/simulation/{{{{model_file}}}}");
    checkModel({{{{model_name}}}}.building);{simulation}
    """
        )
        mos_file = template.render(model_file=Path(temp_model_file.name).name, model_name=network.name)
        Path(temp_mos_file.name).write_text(mos_file)
        yield Path(temp_mos_file.name).name
