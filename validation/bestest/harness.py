"""Run a case with a library and extract its KPIs, caching by the generated model.

The cache key is the Modelica model trano generates (plus the library and the simulation options):
any change of trano that alters the model invalidates the cache, nothing else does.
"""

from __future__ import annotations

import hashlib
import json
import logging
import re
import time
import multiprocessing
import signal
from pathlib import Path
from typing import Any

from pydantic import BaseModel

from trano.data_models.conversion import convert_network
from trano.elements.base import BaseElement
from trano.elements.library.library import Library
from trano.elements.space import Space
from trano.elements.system import IdealHeatingCooling
from trano.simulate.simulate import SimulationOptions, simulate
from trano.topology import Network
from trano.utils.utils import is_success
from validation.bestest.cases import CASES, Case, building_description, check_support, render_case
from validation.bestest.kpi import KpiResults, Signals, extract_kpis, find_variable

logger = logging.getLogger(__name__)

CACHE_ROOT = Path(__file__).resolve().parents[2].joinpath(".cache", "bestest")
SECONDS_PER_YEAR = 365 * 24 * 3600
TOLERANCE = 1e-6
HOURS_PER_YEAR = 365 * 24  # one output point per hour: the KPIs are hourly statistics
LIBRARIES = ("Buildings", "IDEAS", "iso_13790", "reduced_order")
# Result variable holding the zone air temperature, per library.
TEMPERATURE_VARIABLE = {
    "Buildings": "heaPorAir.T",
    "IDEAS": "TAir",
    "iso_13790": "TAir",
    "reduced_order": "TAir",
}


class CaseResult(BaseModel):
    kpis: KpiResults
    model_hash: str
    wall_time: float  # [s] of the simulation, 0 when read from the cache


def network_for(case: Case, library: str) -> Network:
    """The trano network of a case; the YAML is written next to the cache so it can be inspected."""
    directory = cache_directory(case.id, library)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory.joinpath(f"case_{case.id}.yaml")
    path.write_text(render_case(case))
    reset_element_names()
    return convert_network(f"case_{case.id}", path, library=Library.from_configuration(library))


def cache_directory(case_id: str, library: str) -> Path:
    return CACHE_ROOT.joinpath(library, case_id)


def normalized_model(model: str) -> str:
    """The statements of the model, sorted, without annotations and whitespace.

    Layout coordinates differ between renders and the order of the data records (materials,
    constructions) between processes: neither changes the simulation, so neither changes the hash.
    """
    without_annotations = re.sub(r"annotation\s*\(.*?\)\s*;", "", model, flags=re.DOTALL)  # they hold no semicolon
    pieces = re.split(r";|\b(?=(?:record|package|model|end|equation)\b)", without_annotations)
    statements = {re.sub(r"\s+", "", piece) for piece in pieces} - {""}
    return ";".join(sorted(statements))


def model_hash(model: str, library: str, options: SimulationOptions) -> str:
    digest = hashlib.sha256()
    for part in (library, options.model_dump_json(), normalized_model(model)):
        digest.update(part.encode())
        digest.update(b"\0")
    return digest.hexdigest()


def reset_element_names() -> None:
    """Restart the numbering of auto-named elements so that the same case renders the same names."""
    classes = [BaseElement]
    while classes:
        cls = classes.pop()
        cls.name_counter = 0
        classes += cls.__subclasses__()


def _cached(directory: Path, expected_hash: str) -> CaseResult | None:
    path = directory.joinpath("result.json")
    if not path.exists():
        return None
    result = CaseResult.model_validate_json(path.read_text())
    return result.model_copy(update={"wall_time": 0.0}) if result.model_hash == expected_hash else None


def signals_for(network: Network, library: str, result_file: Path, case: Case) -> Signals:
    """Result variables of the case: the temperature of the free-floating zone, the loads of the ideal system.

    With a sun-space (case 960) the standard reports the temperatures of the sun-space.
    """
    from buildingspy.io.outputfile import Reader  # type: ignore

    reader = Reader(str(result_file), "dymola")
    zone_name = "sunspace_001" if case.sunspace else "zone_001"
    zone = next(node for node in network.graph.nodes if isinstance(node, Space) and node.name == zone_name)
    hvac = next((node for node in network.graph.nodes if isinstance(node, IdealHeatingCooling)), None)
    return Signals(
        temperature=find_variable(reader, f"{zone.name}.{TEMPERATURE_VARIABLE[library]}"),
        heating_power=None if hvac is None else find_variable(reader, f"{hvac.name}.QHea_flow"),
        cooling_power=None if hvac is None else find_variable(reader, f"{hvac.name}.QCoo_flow"),
        heating_energy=None if hvac is None else find_variable(reader, f"{hvac.name}.EHea"),
        cooling_energy=None if hvac is None else find_variable(reader, f"{hvac.name}.ECoo"),
    )


def run_case(
    case_id: str,
    library: str = "Buildings",
    force: bool = False,
    container_name: str = "openmodelica",
    end_time: int = SECONDS_PER_YEAR,
) -> CaseResult:
    """Simulate the case (unless cached) and return its KPIs."""
    case = CASES[case_id]
    check_support(case, library)
    options = SimulationOptions(
        start_time=0, end_time=end_time, tolerance=TOLERANCE, number_of_intervals=end_time // 3600
    )
    directory = cache_directory(case_id, library)
    network = network_for(case, library)
    network.set_weather_path_to_container_path(directory)
    model = network.model()  # rendered once: this very text is hashed, kept and simulated
    expected_hash = model_hash(model, library, options)
    if not force and (cached := _cached(directory, expected_hash)) is not None:
        logger.info("Case %s with %s read from the cache", case_id, library)
        return cached
    directory.joinpath("model.mo").write_text(model)
    started = time.monotonic()
    outcome = simulate(directory, network, options=options, container_name=container_name, model=model)
    wall_time = time.monotonic() - started
    output = outcome.output.decode(errors="replace") if isinstance(outcome.output, bytes) else str(outcome.output)
    directory.joinpath("omc.log").write_text(output)
    if not is_success(outcome, options=options):
        raise RuntimeError(f"Case {case_id} with {library} did not simulate; see {directory / 'omc.log'}")
    result_file = directory.joinpath("results", f"case_{case_id}.building_res.mat")
    signals = signals_for(network, library, result_file, case)
    kpis = extract_kpis(result_file, case_id, library, signals, case.trace_days)
    result = CaseResult(kpis=kpis, model_hash=expected_hash, wall_time=wall_time)
    directory.joinpath("result.json").write_text(result.model_dump_json(indent=2))
    return result


def run_cases(
    case_ids: list[str],
    library: str = "Buildings",
    force: bool = False,
    workers: int = 1,
    end_time: int = SECONDS_PER_YEAR,
) -> dict[str, CaseResult]:
    """Run several cases, ``workers`` of them at a time in separate containers.

    Each case runs in its own process: the model generation keeps state at module and class level
    (element name counters, zone numbering), so two networks built in parallel threads corrupt each
    other's models.
    """
    # A worker pool rather than an executor: leaving the block terminates the workers, so a stopped batch
    # does not leave simulations running in the background (a SIGTERM is turned into that exit).
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    arguments = [(case_id, library, force, f"openmodelica-bestest-{case_id}", end_time) for case_id in case_ids]
    with multiprocessing.get_context("spawn").Pool(workers) as pool:
        outcomes = pool.starmap(_run_case_or_error, arguments)
    results = {}
    for case_id, outcome in zip(case_ids, outcomes, strict=True):
        if isinstance(outcome, CaseResult):
            results[case_id] = outcome
        else:
            logger.error("Case %s with %s failed: %s", case_id, library, outcome)
    return results


def _run_case_or_error(*arguments: Any) -> CaseResult | str:  # noqa: ANN401
    """The result of a case, or the error that kept it from simulating (the other cases go on)."""
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


def describe(case_id: str) -> str:
    return json.dumps(building_description(CASES[case_id]), indent=2)
