"""Compile a trano model once with OpenModelica, then run the executable many times with new parameters.

:func:`build_model` translates and compiles the model with ``buildModel`` in the OpenModelica container of
:mod:`trano_simulate.simulate` and keeps the build directory: the executable, ``<Model>_init.xml`` (the
start values and attributes of every variable) and ``<Model>_info.json``. :class:`ModelRunner` starts one
container on that directory and runs the executable as often as needed with parameter overrides
(``-overrideFile``) and experiment settings (``-startTime=``, ``-stopTime=``): no recompilation.

The executable is the native one OpenModelica built (linux64 in the image), so it runs in the same image,
where the libraries it reads resources from at run time (the weather files) are installed too.

Running the executable directly rather than through OMPython's ``ModelicaSystem`` keeps the I/O in one
place (the container of trano's simulations), needs no OMC session per run, and exposes the
executable's log, where the overrides it ignores are reported.
"""

import json
import logging
from collections.abc import Mapping, Sequence
from contextlib import ExitStack
from dataclasses import asdict, dataclass, field
from pathlib import Path
from types import TracebackType
from typing import Self

import docker  # type: ignore
from buildingspy.io.outputfile import Reader  # type: ignore

from trano.runtime.overrides import (
    IgnoredOverrideError,
    OverrideMessage,
    OverrideValue,
    experiment_flags,
    ignored_overrides,
    parse_override_messages,
    write_override_file,
)
from trano_simulate.simulate import (
    CONTAINER_MODEL_DIR,
    MODELICA_ENVIRONMENT,
    ModelicaEnvironment,
    SimulationOptions,
    client,
    container,
    load_statements,
)
from trano.topology import Network

logger = logging.getLogger(__name__)

OPENMODELICA_FLAGS: tuple[str, ...] = ()
"""Command line options of omc for the translation (``setCommandLineOptions``); none by default, as in
trano's regular simulations. Flags such as ``-d=evaluateAllParameters`` or ``--evaluateProtectedParameters``
would evaluate parameters at translation and remove them from what can be changed at run time."""
BUILD_DIRECTORY = "build"
RUNS_DIRECTORY = "runs"
RESULT_FORMAT = "mat"
CONTAINER_PROJECT = CONTAINER_MODEL_DIR.as_posix()  # where trano's simulation container mounts the project
BUILD_CONTAINER = "trano_runtime_build"
RUN_CONTAINER = "trano_runtime_run"
METADATA_FILE = "runtime.json"
SUCCESS_MESSAGE = "The simulation finished successfully"


@dataclass(frozen=True)
class CompiledModel:
    """A compiled model: its build directory in a project and the name of its top-level class."""

    project_path: Path
    model_name: str
    build_directory: str = BUILD_DIRECTORY
    flags: tuple[str, ...] = OPENMODELICA_FLAGS
    options: dict[str, float | int | str | None] = field(default_factory=dict)
    specification: str | None = None  # the YAML file the model was generated from
    library: str | None = None

    @property
    def directory(self) -> Path:
        return self.project_path / self.build_directory

    @property
    def container_directory(self) -> str:
        return f"{CONTAINER_PROJECT}/{self.build_directory}"

    @property
    def executable(self) -> Path:
        return self.directory / self.model_name

    @property
    def init_xml(self) -> Path:
        return self.directory / f"{self.model_name}_init.xml"

    @property
    def info_json(self) -> Path:
        return self.directory / f"{self.model_name}_info.json"

    def exists(self) -> bool:
        return self.executable.exists() and self.init_xml.exists()

    def save(self) -> Path:
        metadata = asdict(self) | {"project_path": str(self.project_path), "flags": list(self.flags)}
        path = self.directory / METADATA_FILE
        path.write_text(json.dumps(metadata, indent=2))
        return path

    @classmethod
    def load(cls, directory: Path) -> "CompiledModel":
        """The compiled model of a build directory (written by :func:`build_model`)."""
        directory = directory.resolve()
        metadata = json.loads((directory / METADATA_FILE).read_text())
        return cls(
            project_path=directory.parent,
            model_name=metadata["model_name"],
            build_directory=directory.name,
            flags=tuple(metadata.get("flags", ())),
            options=metadata.get("options", {}),
            specification=metadata.get("specification"),
            library=metadata.get("library"),
        )


class BuildError(RuntimeError):
    """OpenModelica did not produce the executable."""


def build_script(  # noqa: PLR0913
    model: str,
    model_file: str,
    model_name: str,
    options: SimulationOptions,
    build_directory: str = BUILD_DIRECTORY,
    flags: tuple[str, ...] = OPENMODELICA_FLAGS,
    environment: ModelicaEnvironment = MODELICA_ENVIRONMENT,
) -> str:
    """The OpenModelica script compiling ``model_name`` into ``/simulation/<build_directory>``.

    ``buildModel`` takes the keyword arguments of ``simulate``: they become the default experiment
    (and variable filter) of the init file, which the runs can override.
    """
    flag_statement = f'\n    setCommandLineOptions("{" ".join(flags)}"); getErrorString();' if flags else ""
    return f"""{flag_statement}{load_statements(model, model_file, environment)}
    getErrorString();
    cd("{CONTAINER_PROJECT}/{build_directory}");
    buildModel({model_name}, {options.simulate_arguments}, outputFormat="{RESULT_FORMAT}");
    getErrorString();
    """


def build_model(  # noqa: PLR0913
    project_path: Path,
    network: Network,
    options: SimulationOptions | None = None,
    build_directory: str = BUILD_DIRECTORY,
    container_name: str = BUILD_CONTAINER,
    model: str | None = None,
    flags: tuple[str, ...] = OPENMODELICA_FLAGS,
    specification: Path | None = None,
) -> CompiledModel:
    """Compile the network once in ``project_path/<build_directory>``, kept for repeated runs.

    ``model`` is the Modelica text of the network when it was rendered already. The build log is
    written to ``<build_directory>/build.log``. ``specification`` (the YAML file of the network) is
    recorded so that runs can be given YAML changes.
    """
    project_path = project_path.resolve()
    options = options or SimulationOptions()
    directory = project_path / build_directory
    directory.mkdir(parents=True, exist_ok=True)
    project_path.joinpath("results").mkdir(exist_ok=True)
    if model is None:
        network.set_weather_path(project_path, CONTAINER_MODEL_DIR)
        model = network.model()
    model_file = f"{build_directory}/{network.name}.mo"
    project_path.joinpath(model_file).write_text(model)
    compiled = CompiledModel(
        project_path=project_path,
        model_name=f"{network.name}.building",
        build_directory=build_directory,
        flags=flags,
        options=options.model_dump(exclude={"check_only"}),
        specification=str(specification.resolve()) if specification is not None else None,
        library=network.library.name,
    )
    script = directory / "build.mos"
    script.write_text(build_script(model, model_file, compiled.model_name, options, build_directory, flags))
    with container(client(), project_path, container_name=container_name) as container_:
        result = container_.exec_run(cmd=f"omc {CONTAINER_PROJECT}/{build_directory}/build.mos")
    log = result.output.decode(errors="replace")
    directory.joinpath("build.log").write_text(log)
    if not compiled.exists():
        raise BuildError(f"OpenModelica did not build {compiled.model_name}:\n{log[-3000:]}")
    compiled.save()
    return compiled


@dataclass(frozen=True)
class RunResult:
    """One run of the executable."""

    label: str
    result_path: Path
    log: str
    exit_code: int
    overrides: Mapping[str, OverrideValue]
    messages: tuple[OverrideMessage, ...]

    @property
    def success(self) -> bool:
        return self.exit_code == 0 and SUCCESS_MESSAGE in self.log and self.result_path.exists()

    @property
    def ignored(self) -> tuple[OverrideMessage, ...]:
        """The requested overrides the executable reported as not applied."""
        return tuple(ignored_overrides(self.log, self.overrides))

    def reader(self) -> Reader:
        return Reader(str(self.result_path), "dymola")


class RunError(RuntimeError):
    """The executable failed."""


class ModelRunner:
    """Runs a compiled model repeatedly in one container.

    >>> with ModelRunner(compiled) as runner:  # doctest: +SKIP
    ...     baseline = runner.run(label="baseline", stop_time=14 * 86400)
    ...     result = runner.run({"building.envelope1.living_001.ACH": 0.3}, label="ach", stop_time=14 * 86400)
    """

    def __init__(self, compiled: CompiledModel, container_name: str = RUN_CONTAINER) -> None:
        if not compiled.exists():
            raise FileNotFoundError(f"No executable of {compiled.model_name} in {compiled.directory}")
        self.compiled = compiled
        self.container_name = container_name
        self._stack = ExitStack()
        self._container: docker.models.containers.Container | None = None

    def __enter__(self) -> Self:
        self.compiled.project_path.joinpath(RUNS_DIRECTORY).mkdir(exist_ok=True)
        self._container = self._stack.enter_context(
            container(client(), self.compiled.project_path, container_name=self.container_name)
        )
        return self

    def __exit__(
        self, exc_type: type[BaseException] | None, exc: BaseException | None, traceback: TracebackType | None
    ) -> None:
        self._container = None
        self._stack.close()

    def run(  # noqa: PLR0913
        self,
        overrides: Mapping[str, OverrideValue] | None = None,
        *,
        label: str = "run",
        start_time: float | None = None,
        stop_time: float | None = None,
        step_size: float | None = None,
        tolerance: float | None = None,
        variable_filter: str | None = None,
        flags: Sequence[str] = (),
        strict: bool = False,
        check: bool = True,
    ) -> RunResult:
        """Run the executable with the parameter overrides and experiment settings given.

        The overrides, the log and the result (``runs/<label>_result.mat``; the name keeps it out of the
        ``*_res.mat`` files trano's container gathers) are kept in the project. ``flags`` are further options
        of the executable (``-ignoreHideResult`` to store the variables marked HideResult). ``strict`` raises
        :class:`IgnoredOverrideError` when the executable reports an override as not applied; ``check``
        raises :class:`RunError` when the simulation fails.
        """
        if self._container is None:
            raise RuntimeError("ModelRunner.run must be called inside its context (with ModelRunner(...) as runner)")
        overrides = dict(overrides or {})
        runs = self.compiled.project_path / RUNS_DIRECTORY
        relative = f"../{RUNS_DIRECTORY}"
        command = [f"./{self.compiled.model_name}", f"-r={relative}/{label}_result.{RESULT_FORMAT}"]
        if overrides:
            write_override_file(runs / f"{label}.overrides.txt", overrides)
            command.append(f"-overrideFile={relative}/{label}.overrides.txt")
        settings: dict[str, OverrideValue] = {
            key: value
            for key, value in (
                ("startTime", start_time),
                ("stopTime", stop_time),
                ("stepSize", step_size),
                ("tolerance", tolerance),
                ("variableFilter", variable_filter),
            )
            if value is not None
        }
        if settings:
            command.extend(experiment_flags(settings))
        command.extend(flags)
        result_path = runs / f"{label}_result.{RESULT_FORMAT}"
        result_path.unlink(missing_ok=True)
        execution = self._container.exec_run(cmd=command, workdir=self.compiled.container_directory)
        log = execution.output.decode(errors="replace")
        runs.joinpath(f"{label}.log").write_text(log)
        run = RunResult(
            label=label,
            result_path=result_path,
            log=log,
            exit_code=execution.exit_code,
            overrides=overrides,
            messages=tuple(parse_override_messages(log)),
        )
        if strict and run.ignored:
            raise IgnoredOverrideError(run.ignored)
        if check and not run.success:
            raise RunError(f"Run {label} of {self.compiled.model_name} failed:\n{log[-3000:]}")
        return run


def run_model(  # noqa: PLR0913
    compiled: CompiledModel,
    overrides: Mapping[str, OverrideValue] | None = None,
    *,
    label: str = "run",
    start_time: float | None = None,
    stop_time: float | None = None,
    strict: bool = False,
) -> RunResult:
    """One run in its own container; use :class:`ModelRunner` for many."""
    with ModelRunner(compiled) as runner:
        return runner.run(overrides, label=label, start_time=start_time, stop_time=stop_time, strict=strict)


__all__ = [
    "BUILD_DIRECTORY",
    "OPENMODELICA_FLAGS",
    "RESULT_FORMAT",
    "RUNS_DIRECTORY",
    "BuildError",
    "CompiledModel",
    "ModelRunner",
    "RunError",
    "RunResult",
    "build_model",
    "build_script",
    "run_model",
]
