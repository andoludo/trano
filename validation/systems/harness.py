"""Simulate a case with a library and extract its KPIs, caching by the generated model.

The cache key is the Modelica model trano generates (plus the library and the simulation options), as in the
BESTEST harness: a change of trano that alters the model invalidates the cache, nothing else does. The runtime is
the one of ``trano_simulate`` (the environment variable ``TRANO_SIMULATION_RUNTIME``: ``docker`` by default, ``local``
for the ``omc`` of this machine).
"""

from __future__ import annotations

import logging
import multiprocessing
import signal
import time
from pathlib import Path
from typing import Any

from pydantic import BaseModel, Field

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.topology import Network
from trano.utils.utils import is_success
from trano_simulate.simulate import CONTAINER_MODEL_DIR, SimulationOptions, model_directory, resolve_runtime, simulate
from validation.bestest.harness import model_hash, reset_element_names
from validation.systems.cases import CASES, Case
from validation.systems.kpi import Kpis, extract_kpis

logger = logging.getLogger(__name__)

CACHE_ROOT = Path(__file__).resolve().parents[2].joinpath(".cache", "systems")
RESULTS_ROOT = Path(__file__).resolve().parent.joinpath("results")
TOLERANCE = 1e-6
LOG_LINES = 40
# Every variable is stored: OpenModelica keeps one name per set of alias variables, and with a variable filter
# the name it keeps may be the one the filter drops (the data bus member rather than the sensor, or the reverse).


class CaseResult(BaseModel):
    case: str
    library: str
    kpis: Kpis
    model_hash: str
    wall_time: float = Field(description="[s] of the simulation, 0 when read from the cache")
    traces: dict[str, list[float]] = Field(default_factory=dict, description="Hourly means of a few signals")


def cache_directory(case_id: str, library: str) -> Path:
    return CACHE_ROOT.joinpath(library, case_id)


def simulation_options(case: Case) -> SimulationOptions:
    return SimulationOptions(
        start_time=case.start_time,
        end_time=case.end_time,
        tolerance=TOLERANCE,
        number_of_intervals=(case.end_time - case.start_time) // case.output_interval,
    )


def network_for(case: Case, library: str) -> Network:
    reset_element_names()
    return convert_network(case.id, case.path, library=Library.from_configuration(library))


def _cached(directory: Path, expected_hash: str) -> CaseResult | None:
    path = directory.joinpath("result.json")
    if not path.exists():
        return None
    result = CaseResult.model_validate_json(path.read_text())
    return result.model_copy(update={"wall_time": 0.0}) if result.model_hash == expected_hash else None


def run_case(case_id: str, library: str = "Buildings", force: bool = False, refresh_kpis: bool = False) -> CaseResult:
    """Simulate the case with the library (unless cached) and return its KPIs; ``refresh_kpis`` reads the KPIs again
    from the simulated result file (after a change of ``kpi.py``), ``force`` simulates again."""
    case = CASES[case_id]
    if not case.supports(library):
        raise ValueError(f"Case {case_id} does not run with {library}; its libraries are {case.libraries}")
    options = simulation_options(case)
    directory = cache_directory(case_id, library)
    directory.mkdir(parents=True, exist_ok=True)
    network = network_for(case, library)
    runtime = resolve_runtime()
    network.set_weather_path(directory, model_directory(runtime, directory))
    model = network.model()
    expected_hash = model_hash(model, library, options)
    if not force and not refresh_kpis and (cached := _cached(directory, expected_hash)) is not None:
        logger.info("Case %s with %s read from the cache", case_id, library)
        return cached
    result_file = directory.joinpath("results", f"{case_id}.building_res.mat")
    simulated = directory.joinpath("simulated.txt")  # the hash of the model the result file was simulated from
    wall_time = 0.0
    if force or not (result_file.exists() and simulated.exists() and simulated.read_text() == expected_hash):
        directory.joinpath("model.mo").write_text(model)
        started = time.monotonic()
        outcome = simulate(
            directory, network, options=options, container_name=f"openmodelica-systems-{case_id}", model=model
        )
        wall_time = time.monotonic() - started
        output = outcome.output.decode(errors="replace") if isinstance(outcome.output, bytes) else str(outcome.output)
        directory.joinpath("omc.log").write_text(output)
        if not is_success(outcome, options=options):
            tail = "\n".join(output.splitlines()[-LOG_LINES:])
            raise RuntimeError(f"Case {case_id} with {library} did not simulate; see {directory / 'omc.log'}:\n{tail}")
        simulated.write_text(expected_hash)
    else:
        logger.info("Case %s with %s: KPIs read again from the simulated result file", case_id, library)
    kpis, traces = extract_kpis(case, library, network, result_file)
    result = CaseResult(
        case=case_id, library=library, kpis=kpis, model_hash=expected_hash, wall_time=wall_time, traces=traces
    )
    directory.joinpath("result.json").write_text(result.model_dump_json(indent=2))
    return result


def run_cases(
    case_ids: list[str], library: str = "Buildings", force: bool = False, workers: int = 1, refresh_kpis: bool = False
) -> dict[str, Any]:
    """Run several cases, ``workers`` at a time, each in its own process (the model generation keeps state at
    module level, so two networks built in parallel threads corrupt each other). Returns the results of the cases
    that ran and the error message of the others."""
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    arguments = [(case_id, library, force, refresh_kpis) for case_id in case_ids if CASES[case_id].supports(library)]
    with multiprocessing.get_context("spawn").Pool(workers) as pool:
        outcomes = pool.starmap(_run_case_or_error, arguments)
    results: dict[str, Any] = {}
    for (case_id, *_), outcome in zip(arguments, outcomes, strict=True):
        results[case_id] = outcome
        if not isinstance(outcome, CaseResult):
            logger.error("Case %s with %s failed: %s", case_id, library, outcome)
    return results


def _run_case_or_error(*arguments: Any) -> CaseResult | str:  # noqa: ANN401
    try:
        return run_case(*arguments)
    except Exception as error:
        return f"{type(error).__name__}: {error}"


def _raise_keyboard_interrupt(signum: int, frame: object) -> None:
    raise KeyboardInterrupt


def cached_results(library: str) -> dict[str, CaseResult]:
    """Every cached result of a library, whatever its model hash."""
    results = {}
    for path in sorted(CACHE_ROOT.joinpath(library).glob("*/result.json")):
        results[path.parent.name] = CaseResult.model_validate_json(path.read_text())
    return results


def committed_results(library: str) -> dict[str, CaseResult]:
    """The results committed to the repository (``results/<library>/<case>.json``), so that the documentation renders
    without the cache of the runs."""
    return {
        path.stem: CaseResult.model_validate_json(path.read_text())
        for path in sorted(RESULTS_ROOT.joinpath(library).glob("*.json"))
    }


def load_results(library: str) -> dict[str, CaseResult]:
    """The cached results of a library, completed by its committed results."""
    return {**committed_results(library), **cached_results(library)}


def commit_results(library: str) -> list[Path]:
    """Copy the cached results of a library into ``results/<library>``; returns the files written."""
    directory = RESULTS_ROOT.joinpath(library)
    directory.mkdir(parents=True, exist_ok=True)
    written = []
    for case_id, result in cached_results(library).items():
        path = directory.joinpath(f"{case_id}.json")
        path.write_text(result.model_copy(update={"wall_time": 0.0}).model_dump_json(indent=2) + "\n")
        written.append(path)
    return written


__all__ = ["CONTAINER_MODEL_DIR", "CaseResult", "run_case", "run_cases"]
