import contextlib
import math
import os
import platform
import shutil
import signal
import subprocess
import tempfile
import threading
import time
from collections.abc import Callable, Generator
from contextlib import contextmanager
from enum import StrEnum
from http import HTTPStatus
from pathlib import Path, PurePath, PurePosixPath
from uuid import uuid4

import docker  # type: ignore
from pydantic import BaseModel, Field

from trano.exceptions import DockerClientError, DockerNotInstalledError, OpenModelicaNotFoundError
from trano.elements.jinja import STRING_ENVIRONMENT
from trano.simulate.result import SimulationErrorKind, SimulationResult, truncate_log
from trano.topology import Network

RUNTIME_VARIABLE = "TRANO_SIMULATION_RUNTIME"
CONTAINER_MODEL_DIR = PurePosixPath("/simulation")  # where the project directory is mounted in the container
TIMEOUT_EXIT_CODE = 124  # exit code of coreutils' ``timeout`` when it stops the command


class SimulationRuntime(StrEnum):
    """Where OpenModelica runs: in a sibling container of its image, or ``omc`` on the PATH of this machine."""

    docker = "docker"
    local = "local"

    @classmethod
    def from_environment(cls) -> "SimulationRuntime":
        """The runtime named by the environment variable ``TRANO_SIMULATION_RUNTIME``, else ``docker``."""
        value = os.environ.get(RUNTIME_VARIABLE, "").strip().lower()
        if not value:
            return cls.docker
        try:
            return cls(value)
        except ValueError as error:
            raise ValueError(
                f"{RUNTIME_VARIABLE}={value!r} is not a simulation runtime; use one of {', '.join(cls)}."
            ) from error


class SimulationPhase(StrEnum):
    """The steps reported to the ``on_phase`` callback, in this order."""

    generating = "generating"  # YAML to network and Modelica text
    compiling = "compiling"  # before OpenModelica starts
    simulating = "simulating"  # OpenModelica started
    done = "done"


PhaseCallback = Callable[[str], None]


def resolve_runtime(runtime: SimulationRuntime | str | None = None) -> SimulationRuntime:
    return SimulationRuntime.from_environment() if runtime is None else SimulationRuntime(runtime)


def model_directory(runtime: SimulationRuntime, project_path: Path) -> PurePath:
    """The project directory as ``omc`` sees it: mounted in the container, or the directory itself."""
    return CONTAINER_MODEL_DIR if runtime is SimulationRuntime.docker else project_path.resolve()


def notify(on_phase: PhaseCallback | None, phase: SimulationPhase) -> None:
    if on_phase is not None:
        on_phase(phase)


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


LIBRARIES_VARIABLE = "TRANO_MODELICA_LIBRARIES"
CONTAINER_LIBRARIES = "/root/.openmodelica/libraries"  # where OpenModelica's package manager installs
CONTAINER_CACHE = "/root/.openmodelica/cache"  # where it keeps the downloaded archives
EXTRACT_SDF = """
import sys
import zipfile
from pathlib import Path

cache, libraries, version = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
archive, target = cache / f"SDF-Modelica-{version}.zip", libraries / f"SDF {version}"
if archive.exists() and not (target / "package.mo").exists():
    with zipfile.ZipFile(archive) as files:
        for name in files.namelist():
            if name.startswith("SDF/") and not name.endswith("/"):
                path = target / name.removeprefix("SDF/")
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(files.read(name))
"""
"""Install SDF from the archive the package manager downloaded: OpenModelica 1.26.9 fails to extract
it (the archive has no directory entries), which aborts the installation of AixLib, its dependent.
Arguments: the archive cache, the library directory and the SDF version."""


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
    libraries_path: Path | None = Field(
        default_factory=lambda: Path(path) if (path := os.environ.get(LIBRARIES_VARIABLE)) else None,
        description="Host directory of installed Modelica libraries mounted into the container; "
        f"from the environment variable {LIBRARIES_VARIABLE}. Without it the libraries are installed "
        "by OpenModelica's package manager at every container start, which needs network access.",
    )
    modelica: list[str] = Field(default=["4.0.0+maint.om", "4.1.0+maint.om"])
    buildings: str = Field(default="13.0.0")
    ideas: str = Field(default="3.0.0")
    aixlib: str = Field(default="3.0.1")
    sdf: str = Field(default="0.4.5", description="Version of SDF, the dependency of AixLib installed by hand.")

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
    number_of_intervals: int | None = Field(
        default=None, gt=0, description="Output points over the simulation; OpenModelica's default when unset."
    )
    variable_filter: str | None = Field(
        default=None, description="Regular expression of the variables to store (OpenModelica's variableFilter)."
    )

    @property
    def simulate_arguments(self) -> str:
        """The keyword arguments of OpenModelica's ``simulate`` command."""
        arguments = [f"startTime = {self.start_time}", f"stopTime = {self.end_time}", f"tolerance = {self.tolerance}"]
        if self.number_of_intervals is not None:
            arguments.append(f"numberOfIntervals = {self.number_of_intervals}")
        if self.variable_filter is not None:
            arguments.append(f'variableFilter = "{self.variable_filter}"')
        return ", ".join(arguments)


class SimulationLibraryOptions(SimulationOptions):
    library_name: str = Field(default="Buildings")


def simulate(  # noqa: PLR0913
    project_path: Path,
    model_network: Network,
    options: SimulationOptions | None = None,
    container_name: str | None = None,
    model: str | None = None,
    runtime: SimulationRuntime | str | None = None,
    timeout_s: float | None = None,
    on_phase: PhaseCallback | None = None,
) -> docker.models.containers.ExecResult | SimulationResult:
    """Simulate the network with OpenModelica; results land in ``project_path/results``.

    ``runtime`` defaults to the environment variable ``TRANO_SIMULATION_RUNTIME``, else ``docker``:

    * ``docker`` runs ``omc`` in a container of the OpenModelica image and returns docker's ``ExecResult``
      (unchanged for existing callers). Each call gets its own container, named ``container_name``
      (``trano-<random>`` by default).
    * ``local`` runs the ``omc`` found on the PATH and returns a ``SimulationResult``. The Modelica
      libraries must be installed already (``python -m trano.simulate.install_libraries``).

    Two runs must not share a ``project_path``. ``model`` is the Modelica text of the network when it was
    rendered already; a network renders its model once. ``timeout_s`` stops OpenModelica after that long.
    """
    options = options or SimulationOptions()
    if resolve_runtime(runtime) is SimulationRuntime.local:
        return simulate_local(project_path, model_network, options, model=model, timeout_s=timeout_s, on_phase=on_phase)
    return simulate_docker(
        project_path,
        model_network,
        options,
        container_name=container_name,
        model=model,
        timeout_s=timeout_s,
        on_phase=on_phase,
    )


def simulate_docker(  # noqa: PLR0913
    project_path: Path,
    model_network: Network,
    options: SimulationOptions,
    container_name: str | None = None,
    model: str | None = None,
    timeout_s: float | None = None,
    on_phase: PhaseCallback | None = None,
) -> docker.models.containers.ExecResult:
    """Simulate in a sibling container of the OpenModelica image, through the Docker socket."""
    client_ = client()
    notify(on_phase, SimulationPhase.compiling)
    with (
        container(client_, project_path, container_name=container_name) as container_,
        create_mos_file(model_network, options, project_path, model=model) as mos_file_name,
    ):
        command = f"omc {CONTAINER_MODEL_DIR}/{mos_file_name}"
        if timeout_s is not None:
            command = f"timeout {math.ceil(timeout_s)} {command}"
        notify(on_phase, SimulationPhase.simulating)
        results = container_.exec_run(cmd=command)
    return results


def simulate_local(  # noqa: PLR0913
    project_path: Path,
    model_network: Network,
    options: SimulationOptions,
    model: str | None = None,
    timeout_s: float | None = None,
    on_phase: PhaseCallback | None = None,
    environment: ModelicaEnvironment | None = None,
) -> SimulationResult:
    """Simulate with the ``omc`` of this machine, the libraries being installed already.

    OpenModelica writes ``<model>.building_res.mat`` (and its build files) into its working directory,
    ``project_path/results``.

    Raises:
        OpenModelicaNotFoundError: when ``omc`` is not on the PATH.
    """
    omc = shutil.which("omc")
    if omc is None:
        raise OpenModelicaNotFoundError(
            f"OpenModelica's omc is not on the PATH, which the local simulation runtime needs. Install "
            f"OpenModelica or set the environment variable {RUNTIME_VARIABLE}=docker to simulate in a container."
        )
    started = time.perf_counter()
    project_path = project_path.resolve()
    results_path = project_path / "results"
    results_path.mkdir(parents=True, exist_ok=True)
    with create_mos_file(
        model_network,
        options,
        project_path,
        environment=environment or MODELICA_ENVIRONMENT,
        model=model,
        model_dir=project_path,
        report_check_errors=True,
    ) as mos_file_name:
        notify(on_phase, SimulationPhase.compiling)
        log, timed_out = run_omc(
            [omc, str(project_path / mos_file_name)],
            cwd=results_path,
            timeout_s=timeout_s,
            on_start=lambda: notify(on_phase, SimulationPhase.simulating),
        )
    duration_s = time.perf_counter() - started
    if timed_out:
        return SimulationResult.failure(
            SimulationErrorKind.timeout, f"OpenModelica did not finish within {timeout_s} s.", log, duration_s
        )
    return SimulationResult.from_log(
        log,
        check_only=options.check_only,
        result_path=results_path / f"{model_network.name}.building_res.mat",
        duration_s=duration_s,
    )


def run_omc(
    command: list[str], cwd: Path, timeout_s: float | None = None, on_start: Callable[[], None] | None = None
) -> tuple[str, bool]:
    """OpenModelica's output (stdout and stderr, truncated) and whether it was stopped after ``timeout_s``.

    The output is read line by line: ``on_start`` is called once, with the first line.
    """
    timed_out = threading.Event()
    lines: list[str] = []
    with subprocess.Popen(  # noqa: S603
        command,
        cwd=cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        errors="replace",
        start_new_session=os.name == "posix",  # its own process group, with the simulation executable it starts
    ) as process:

        def stop() -> None:
            timed_out.set()
            _kill(process)

        timer = threading.Timer(timeout_s, stop) if timeout_s is not None else None
        if timer is not None:
            timer.start()
        try:
            for line in process.stdout or ():
                if not lines and on_start is not None:
                    on_start()
                lines.append(line)
            process.wait()
        finally:
            if timer is not None:
                timer.cancel()
    return truncate_log("".join(lines)), timed_out.is_set()


def _kill(process: subprocess.Popen[str]) -> None:
    """Kill ``omc`` and, on POSIX, the simulation executable it started, which holds its output open."""
    if os.name == "posix":
        with contextlib.suppress(ProcessLookupError):
            os.killpg(process.pid, signal.SIGKILL)
    else:
        process.kill()


def run_simulation(  # noqa: PLR0913
    project_path: Path,
    model_network: Network,
    options: SimulationOptions | None = None,
    runtime: SimulationRuntime | str | None = None,
    model: str | None = None,
    container_name: str | None = None,
    timeout_s: float | None = None,
    on_phase: PhaseCallback | None = None,
) -> SimulationResult:
    """``simulate`` with a ``SimulationResult`` for both runtimes."""
    options = options or SimulationOptions()
    if resolve_runtime(runtime) is SimulationRuntime.local:
        return simulate_local(project_path, model_network, options, model=model, timeout_s=timeout_s, on_phase=on_phase)
    started = time.perf_counter()
    outcome = simulate_docker(
        project_path,
        model_network,
        options,
        container_name=container_name,
        model=model,
        timeout_s=timeout_s,
        on_phase=on_phase,
    )
    output = outcome.output
    log = output.decode(errors="replace") if isinstance(output, bytes) else str(output or "")
    duration_s = time.perf_counter() - started
    if timeout_s is not None and outcome.exit_code == TIMEOUT_EXIT_CODE:
        return SimulationResult.failure(
            SimulationErrorKind.timeout, f"OpenModelica did not finish within {timeout_s} s.", log, duration_s
        )
    return SimulationResult.from_log(
        log,
        check_only=options.check_only,
        result_path=project_path / "results" / f"{model_network.name}.building_res.mat",
        duration_s=duration_s,
    )


def unique_container_name() -> str:
    return f"trano-{uuid4().hex[:8]}"


def stop_container(client: docker.DockerClient, container_name: str, attempts: int = 10) -> None:
    """Stop and remove a leftover container of that name, waiting for a removal already in progress."""
    for attempt in range(attempts):
        try:
            container = client.containers.get(container_name)
            if container.attrs["State"]["Status"] == "running":
                container.stop()
            container.remove()
            return
        except docker.errors.NotFound:
            return
        except docker.errors.APIError as error:
            if error.status_code != HTTPStatus.CONFLICT or attempt == attempts - 1:
                raise
            time.sleep(1)


@contextmanager
def container(
    client: docker.DockerClient,
    project_path: Path,
    environment: ModelicaEnvironment = MODELICA_ENVIRONMENT,
    container_name: str | None = None,
) -> Generator[docker.models.containers.Container, None, None]:
    """A container of the OpenModelica image with the project mounted and the libraries installed.

    Without ``container_name`` it gets a unique ``trano-<random>`` name, so that runs can be parallel.
    """
    container_name = container_name or unique_container_name()
    stop_container(client, container_name)
    volumes = [f"{project_path}:{CONTAINER_MODEL_DIR}", f"{project_path}/results:/results"]
    if environment.libraries_path is not None:
        volumes.append(f"{environment.libraries_path.resolve()}:{CONTAINER_LIBRARIES}")
    container = client.containers.run(
        environment.openmodelica_image,
        command="tail -f /dev/null",
        volumes=volumes,
        detach=True,
        name=container_name,
    )
    try:
        (project_path / "configure.mos").write_text(environment.configure_script())
        (project_path / "extract_sdf.py").write_text(EXTRACT_SDF)
        container.exec_run(cmd="chmod -R 777 /results")
        container.exec_run(cmd="chmod -R 777 /simulation")
        container.exec_run(cmd="omc /simulation/configure.mos")
        # AixLib fails to install on its SDF dependency: install SDF by hand, then configure again,
        # which installs what is still missing and only checks the rest.
        container.exec_run(
            cmd=f"python3 /simulation/extract_sdf.py {CONTAINER_CACHE} {CONTAINER_LIBRARIES} {environment.sdf}"
        )
        container.exec_run(cmd="omc /simulation/configure.mos")
        yield container
        container.exec_run(
            cmd='find / -name "*_res.mat" -exec cp {} /results \;'  # noqa: W605
        )
    finally:
        # also after a failed run: every run has its own container, which nothing else removes
        container.stop()
        container.remove()


def _load_aixlib(model: str, environment: ModelicaEnvironment) -> str:
    """The statement loading AixLib for the models using it (reduced-order and ISO 13790 zones).

    The `uses` annotation of the generated package does not list AixLib, so OpenModelica does not
    load it with the model. The configured version is preferred; the package manager may have
    installed another compatible one (AixLib is not installed with an exact match), which
    "default" (the newest installed) then loads.
    """
    if "AixLib." not in model:
        return ""
    return f"""
    loadModel(AixLib, {{"{environment.aixlib}", "default"}});"""


def load_statements(model: str, model_file: str, environment: ModelicaEnvironment = MODELICA_ENVIRONMENT) -> str:
    """The OpenModelica statements loading the libraries and the model file ``/simulation/<model_file>``.

    ``model`` is the Modelica text of the file: it tells whether AixLib has to be loaded too.
    """
    return f"""
    getVersion();
    loadModel(Modelica, {{"{environment.modelica_version}"}});
    loadModel(IDEAS, {{"{environment.ideas}"}});{_load_aixlib(model, environment)}
    loadFile("/simulation/{model_file}");"""


@contextmanager
def create_mos_file(  # noqa: PLR0913
    network: Network,
    options: SimulationOptions,
    project_path: Path,
    environment: ModelicaEnvironment = MODELICA_ENVIRONMENT,
    model: str | None = None,
    model_dir: PurePath = CONTAINER_MODEL_DIR,
    report_check_errors: bool = False,
) -> Generator[str, None, None]:
    """Write the model and the OpenModelica script simulating it into ``project_path``; yield the script name.

    ``model_dir`` is ``project_path`` as ``omc`` sees it: mounted in the container (docker runtime) or
    the directory itself (local runtime). ``report_check_errors`` prints why the check of a ``check_only``
    run failed.
    """
    if model is None:
        # TODO: do we want this here?
        network.set_weather_path(project_path, model_dir)
        model = network.model()
    with (
        tempfile.NamedTemporaryFile(mode="w", dir=project_path, suffix=".mo") as temp_model_file,
        tempfile.NamedTemporaryFile(mode="w", dir=project_path, suffix=".mos") as temp_mos_file,
    ):
        Path(temp_model_file.name).write_text(model)
        check_errors = "\n    getErrorString();" if options.check_only and report_check_errors else ""
        simulation = (
            check_errors
            if options.check_only
            else f"""
    simulate({{{{model_name}}}}.building, {options.simulate_arguments});
    getErrorString();"""
        )
        template = STRING_ENVIRONMENT.from_string(
            f"""{load_statements(model, "{{model_file}}", environment)};
    getVersion();
    loadModel(Modelica, {{"{environment.modelica_version}"}});
    loadModel(IDEAS, {{"{environment.ideas}"}});{_load_aixlib(model, environment)}
    loadFile("{model_dir.as_posix()}/{{{{model_file}}}}");
    checkModel({{{{model_name}}}}.building);{simulation}
    """
        )
        mos_file = template.render(model_file=Path(temp_model_file.name).name, model_name=network.name)
        Path(temp_mos_file.name).write_text(mos_file)
        yield Path(temp_mos_file.name).name
