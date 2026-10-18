"""One-call Python API: a YAML model in, a typed ``SimulationResult`` out.

    from trano import simulate_yaml
    from trano.simulate.simulate import SimulationOptions

    result = simulate_yaml(Path("house.yaml"), options=SimulationOptions(end_time=86400), runtime="local")
    result.success, result.result_path, result.error_kind, result.error_message

Model errors and simulation failures are returned, not raised: ``error_kind`` tells which step failed
and ``error_message`` says why in one line.
"""

import re
import tempfile
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import docker  # type: ignore
import yaml
from pydantic import ValidationError

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.exceptions import (
    DockerClientError,
    DockerNotInstalledError,
    InvalidModelError,
    OpenModelicaNotFoundError,
    UnknownLibraryError,
)
from trano.simulate.result import SimulationErrorKind, SimulationResult
from trano.simulate.simulate import (
    PhaseCallback,
    SimulationOptions,
    SimulationPhase,
    SimulationRuntime,
    model_directory,
    notify,
    resolve_runtime,
    run_simulation,
)

ModelInput = Path | str | Mapping[str, Any]
DEFAULT_MODEL_NAME = "model"
MODEL_ERRORS = (InvalidModelError, ValidationError, UnknownLibraryError, FileNotFoundError)
RUNTIME_ERRORS = (OpenModelicaNotFoundError, DockerNotInstalledError, DockerClientError, docker.errors.DockerException)


def simulate_yaml(  # noqa: PLR0913
    model: ModelInput,
    *,
    library: str = "Buildings",
    options: SimulationOptions | None = None,
    project_path: Path | None = None,
    runtime: SimulationRuntime | str | None = None,
    on_phase: PhaseCallback | None = None,
    name: str | None = None,
    timeout_s: float | None = None,
) -> SimulationResult:
    """Validate, translate and simulate a model.

    Args:
        model: path of a YAML (or JSON) model, or the model already parsed into a dict, which is written
            to ``project_path/<name>.yaml`` first.
        library: library the model is generated with.
        options: simulation period, tolerance, outputs; ``check_only`` stops after OpenModelica's check.
        project_path: directory of the run, which must not be shared with another run; a new temporary
            directory, left for the caller to remove, when None. The result file lands in its ``results``.
        runtime: ``docker`` or ``local``; the environment variable ``TRANO_SIMULATION_RUNTIME`` when None,
            else ``docker``.
        on_phase: called with ``"generating"``, ``"compiling"``, ``"simulating"`` and ``"done"``.
        name: name of the Modelica package; the file name of the model, or ``model`` for a dict, when None.
        timeout_s: OpenModelica is stopped after this long and the result is a ``timeout`` error.
    """
    started = time.perf_counter()
    options = options or SimulationOptions()
    project_path = _project_path(project_path)
    try:
        result = _simulate(model, library, options, project_path, runtime, on_phase, name, timeout_s)
    finally:
        notify(on_phase, SimulationPhase.done)
    return result.model_copy(update={"duration_s": time.perf_counter() - started})


def check_yaml(  # noqa: PLR0913
    model: ModelInput,
    *,
    library: str = "Buildings",
    runtime: SimulationRuntime | str | None = None,
    project_path: Path | None = None,
    name: str | None = None,
    timeout_s: float | None = None,
) -> SimulationResult:
    """Validate and translate a model, and check it with OpenModelica, without simulating it."""
    return simulate_yaml(
        model,
        library=library,
        options=SimulationOptions(check_only=True),
        project_path=project_path,
        runtime=runtime,
        name=name,
        timeout_s=timeout_s,
    )


def _simulate(  # noqa: PLR0913
    model: ModelInput,
    library: str,
    options: SimulationOptions,
    project_path: Path,
    runtime: SimulationRuntime | str | None,
    on_phase: PhaseCallback | None,
    name: str | None,
    timeout_s: float | None,
) -> SimulationResult:
    notify(on_phase, SimulationPhase.generating)
    try:
        runtime_ = resolve_runtime(runtime)
    except ValueError as error:
        return SimulationResult.failure(SimulationErrorKind.runtime, str(error))
    try:
        model_path, model_name = _model_file(model, project_path, name)
        network = convert_network(model_name, model_path, library=Library.from_configuration(library))
        network.set_weather_path(project_path, model_directory(runtime_, project_path))
        modelica_model = network.model()
    except MODEL_ERRORS as error:
        return SimulationResult.failure(SimulationErrorKind.validation, _one_line(error), log=str(error))
    try:
        return run_simulation(
            project_path,
            network,
            options,
            runtime=runtime_,
            model=modelica_model,
            timeout_s=timeout_s,
            on_phase=on_phase,
        )
    except RUNTIME_ERRORS as error:
        return SimulationResult.failure(SimulationErrorKind.runtime, _one_line(error), log=str(error))


def _project_path(project_path: Path | None) -> Path:
    if project_path is None:
        return Path(tempfile.mkdtemp(prefix="trano-"))
    project_path.mkdir(parents=True, exist_ok=True)
    return project_path.resolve()


def _model_file(model: ModelInput, project_path: Path, name: str | None) -> tuple[Path, str]:
    """The file of the model, writing a parsed model into the project, and the name of its package."""
    if isinstance(model, Mapping):
        model_name = modelica_name(name or DEFAULT_MODEL_NAME)
        model_path = project_path / f"{model_name}.yaml"
        model_path.write_text(yaml.safe_dump(dict(model), sort_keys=False))
        return model_path, model_name
    model_path = Path(model).resolve()
    if not model_path.exists():
        raise FileNotFoundError(f"Model file {model_path} not found.")
    return model_path, modelica_name(name or model_path.stem)


def modelica_name(name: str) -> str:
    """``name`` as a Modelica identifier: letters, digits and underscores, not starting with a digit."""
    identifier = re.sub(r"\W", "_", name)
    return f"_{identifier}" if not identifier or identifier[0].isdigit() else identifier


def _one_line(error: Exception) -> str:
    errors = getattr(error, "errors", None)
    if isinstance(errors, tuple) and errors:
        return "; ".join(errors)
    return " ".join(str(error).split()) or type(error).__name__
