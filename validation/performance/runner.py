"""Translate and simulate the benchmark cases in OpenModelica, timing each step and collecting solver statistics.

The model is built with ``buildModel`` and its executable run on its own, so that the translation (front
end, back end, code generation and C compilation) and the simulation are timed separately. All cases of a
run share one container: the timings are only comparable within a run on the same machine, and the cases
run one after the other.
"""

from __future__ import annotations

import logging
import re
import shlex
import time
from collections.abc import Iterator
from dataclasses import dataclass
from contextlib import contextmanager, nullcontext
from pathlib import Path

import docker  # type: ignore[import-untyped]
from pydantic import BaseModel, Field

from trano.simulate.simulate import MODELICA_ENVIRONMENT, CONTAINER_LIBRARIES, SimulationOptions, client, container
from validation.bestest.harness import model_hash
from validation.performance.cases import BenchmarkCase

logger = logging.getLogger(__name__)

CONTAINER_ROOT = "/simulation"
DAY = 24 * 3600
RESULT_FILE = "model_res.mat"
URI_LIBRARIES = ("Buildings", "IDEAS", "AixLib", "Modelica")


class Settings(BaseModel):
    """Fixed simulation settings of a benchmark run."""

    days: int = Field(default=7, gt=0)
    tolerance: float = 1e-6
    method: str = "dassl"
    output_interval: int = Field(default=900, gt=0, description="[s]")

    @property
    def stop_time(self) -> int:
        return self.days * DAY

    @property
    def number_of_intervals(self) -> int:
        return self.stop_time // self.output_interval

    def simulation_options(self) -> SimulationOptions:
        return SimulationOptions(
            start_time=0,
            end_time=self.stop_time,
            tolerance=self.tolerance,
            number_of_intervals=self.number_of_intervals,
        )


class TranslationStatistics(BaseModel):
    states: int | None = None
    linear_systems: list[int] = Field(default_factory=list, description="iteration variables of each torn system")
    nonlinear_systems: list[int] = Field(default_factory=list, description="iteration variables of each torn system")


class SolverStatistics(BaseModel):
    steps: int | None = None
    rhs_evaluations: int | None = None
    jacobian_evaluations: int | None = None
    error_test_failures: int | None = None
    convergence_test_failures: int | None = None
    state_events: int | None = None
    time_events: int | None = None
    jacobian_time: float | None = Field(default=None, description="[s]")


class CaseRun(BaseModel):
    case_id: str
    model_hash: str
    success: bool
    translation_time: float = Field(description="[s] wall clock of buildModel, the libraries loading included")
    simulation_time: float | None = Field(default=None, description="[s] wall clock of the executable")
    translation: TranslationStatistics = Field(default_factory=TranslationStatistics)
    solver: SolverStatistics = Field(default_factory=SolverStatistics)
    result_file: str | None = None


def parse_translation(log: str) -> TranslationStatistics:
    """Statistics of the simulation system from the ``-d=backenddaeinfo`` output of ``buildModel``."""
    simulation = (
        log.rsplit("Strong component statistics for simulation", maxsplit=1)[-1] if "for simulation" in log else ""
    )
    states = re.findall(r"Number of states: *(\d+)", log)

    def iteration_variables(kind: str) -> list[int]:
        found = re.search(rf"\* {kind} torn systems[^:]*: \d+ systems?\s*\{{([^}}]*)\}}", simulation)
        sizes = re.findall(r"\((\d+),", found.group(1)) if found else []
        return sorted((int(size) for size in sizes), reverse=True)

    return TranslationStatistics(
        states=int(states[-1]) if states else None,
        linear_systems=iteration_variables("Linear"),
        nonlinear_systems=iteration_variables("Non-linear"),
    )


_SOLVER_PATTERNS = {
    "steps": r"(\d+) steps taken",
    "rhs_evaluations": r"(\d+) calls of functionODE\n",
    "jacobian_evaluations": r"(\d+) evaluations of jacobian\n",
    "error_test_failures": r"(\d+) error test failures",
    "convergence_test_failures": r"(\d+) convergence test failures",
    "state_events": r"(\d+) state events",
    "time_events": r"(\d+) time events",
}


def parse_solver(log: str) -> SolverStatistics:
    """Statistics of the integrator from the ``-lv=LOG_STATS`` output of the simulation."""
    values: dict[str, float | int] = {}
    for key, pattern in _SOLVER_PATTERNS.items():
        if found := re.search(pattern, log):
            values[key] = int(found.group(1))
    if found := re.search(r"([\d.]+(?:e[+-]?\d+)?)s time of jacobian evaluation", log):
        values["jacobian_time"] = float(found.group(1))
    return SolverStatistics.model_validate(values)


def build_script(case: BenchmarkCase, model_name: str, settings: Settings, aixlib: bool) -> str:
    environment = MODELICA_ENVIRONMENT
    libraries = [("Modelica", environment.modelica_version), ("Buildings", environment.buildings)]
    libraries.append(("IDEAS", environment.ideas))
    if aixlib:
        libraries.append(("AixLib", environment.aixlib))
    loads = "\n".join(
        f'loadModel({name}, {{"{version}", "default"}}); getErrorString();' for name, version in libraries
    )
    return f"""{loads}
loadFile("model.mo"); getErrorString();
setCommandLineOptions("-d=backenddaeinfo"); getErrorString();
buildModel({model_name}.building, startTime=0, stopTime={settings.stop_time}, tolerance={settings.tolerance},
           method="{settings.method}", numberOfIntervals={settings.number_of_intervals}, fileNamePrefix="model");
getErrorString();
"""


def link_libraries_script() -> str:
    """Shell commands resolving ``modelica://<Library>/...`` URIs for an executable run in its directory.

    With Modelica 4.1, ``loadResource`` is no longer evaluated at translation: the simulation executable
    resolves the URI against its working directory, where these links stand for the installed libraries.
    """
    links = [
        f'for d in {CONTAINER_LIBRARIES}/{library}\\ *; do [ -d "$d" ] && ln -sfn "$d" modelica:/{library}; done'
        for library in URI_LIBRARIES
    ]
    commands = ["mkdir -p modelica:", *links]
    return "; ".join(commands)


def _exec(container_: docker.models.containers.Container, command: str, workdir: str) -> tuple[float, str, int]:
    started = time.monotonic()
    outcome = container_.exec_run(["bash", "-c", command], workdir=workdir)
    output = outcome.output.decode(errors="replace") if isinstance(outcome.output, bytes) else str(outcome.output)
    return time.monotonic() - started, output, outcome.exit_code


@contextmanager
def benchmark_container(project: Path, name: str = "trano-performance") -> Iterator[docker.models.containers.Container]:
    project.mkdir(parents=True, exist_ok=True)
    project.joinpath("results").mkdir(exist_ok=True)
    with container(client(), project, container_name=name) as container_:
        yield container_


@dataclass(frozen=True)
class RenderedCase:
    """A case with its model rendered once: the text that is hashed, written and simulated."""

    case: BenchmarkCase
    model_name: str
    model: str
    model_hash: str

    @classmethod
    def render(cls, case: BenchmarkCase, settings: Settings) -> RenderedCase:
        network = case.network()
        model = network.model()
        return cls(case, network.name, model, model_hash(model, case.library, settings.simulation_options()))


def run_case(
    container_: docker.models.containers.Container, rendered: RenderedCase, project: Path, settings: Settings
) -> CaseRun:
    """Translate and simulate one case in the running container; its files land in ``project/<case id>``."""
    case = rendered.case
    directory = project.joinpath(case.id)
    directory.mkdir(parents=True, exist_ok=True)
    directory.joinpath("model.mo").write_text(rendered.model)
    directory.joinpath("build.mos").write_text(
        build_script(case, rendered.model_name, settings, aixlib="AixLib." in rendered.model)
    )
    workdir = f"{CONTAINER_ROOT}/{case.id}"
    _exec(container_, "rm -f model model_res.mat", workdir)
    translation_time, build_log, _ = _exec(container_, "omc build.mos", workdir)
    directory.joinpath("build.log").write_text(build_log)
    run = CaseRun(
        case_id=case.id,
        model_hash=rendered.model_hash,
        success=False,
        translation_time=translation_time,
        translation=parse_translation(build_log),
    )
    if not directory.joinpath("model").exists():
        logger.error("Case %s did not translate; see %s", case.id, directory / "build.log")
        return run
    _exec(container_, link_libraries_script(), workdir)
    simulation_time, simulation_log, exit_code = _exec(
        container_, f"./model -lv=LOG_STATS -s={shlex.quote(settings.method)}", workdir
    )
    directory.joinpath("simulation.log").write_text(simulation_log)
    success = exit_code == 0 and "The simulation finished successfully" in simulation_log
    if not success:
        logger.error("Case %s did not simulate; see %s", case.id, directory / "simulation.log")
    return run.model_copy(
        update={
            "success": success,
            "simulation_time": simulation_time,
            "solver": parse_solver(simulation_log),
            "result_file": str(directory.joinpath(RESULT_FILE)) if success else None,
        }
    )


def _stored_run(project: Path, rendered: RenderedCase, reuse: bool) -> CaseRun | None:
    """The stored successful run of the case: when its model did not change since, or any with ``reuse``."""
    path = project.joinpath(rendered.case.id, "run.json")
    if not path.exists():
        return None
    run = CaseRun.model_validate_json(path.read_text())
    if not (run.success and run.result_file and Path(run.result_file).exists()):
        return None
    return run if reuse or run.model_hash == rendered.model_hash else None


def run_cases(
    cases: list[BenchmarkCase], project: Path, settings: Settings, force: bool = False, reuse: bool = False
) -> list[CaseRun]:
    """Run the cases one after the other in one container.

    A case is not run again when its stored run has the same model (unless ``force``), or any stored run with
    ``reuse`` (to render a report again: some models name their controls in an order that changes between
    processes, which changes their hash, not their behaviour). Nor is it run when another case of the batch
    renders the same model (a profile that does not apply to it): it takes that run.
    """
    runs: dict[str, CaseRun] = {}
    by_hash: dict[str, CaseRun] = {}
    pending: list[RenderedCase] = []
    for case in cases:
        rendered = RenderedCase.render(case, settings)
        if not force and (stored := _stored_run(project, rendered, reuse)) is not None:
            logger.info("%s: unchanged, reusing the stored run", case.id)
            runs[case.id] = by_hash[rendered.model_hash] = stored
        else:
            pending.append(rendered)
    container_context = benchmark_container(project) if pending else nullcontext(None)
    with container_context as container_:
        for rendered in pending:
            case = rendered.case
            if (same := by_hash.get(rendered.model_hash)) is not None:
                logger.info("%s: same model as %s, sharing its run", case.id, same.case_id)
                runs[case.id] = same.model_copy(update={"case_id": case.id})
                continue
            logger.info("%s: translating and simulating", case.id)
            run = run_case(container_, rendered, project, settings)
            project.joinpath(case.id, "run.json").write_text(run.model_dump_json(indent=2))
            runs[case.id] = by_hash[rendered.model_hash] = run
            logger.info(
                "%s: translation %.1f s, simulation %s s",
                case.id,
                run.translation_time,
                f"{run.simulation_time:.1f}" if run.simulation_time is not None else "-",
            )
    return [runs[case.id] for case in cases]
