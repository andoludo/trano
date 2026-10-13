"""Replay a Buildings simulation through the ``mpc`` model of the same YAML and compare the zones.

1. The Buildings model is simulated (cached by the generated model, as for BESTEST).
2. The heat its emitters delivered to each zone (ideal heating and cooling, radiators) is written as the
   ``<zone>_QHea`` columns of an external data file: the runnable ``mpc`` model replays such columns as
   its heating inputs.
3. The runnable ``mpc`` model is simulated with that file. Weather, solar irradiance and occupancy are
   computed with the same Buildings blocks as in the Buildings model, so both models see the same inputs
   and only the zone physics differ.
4. The zone air temperatures are compared hour by hour.
"""

from __future__ import annotations

import contextlib
import logging
import time
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import pandas as pd
from pydantic import BaseModel, Field

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.elements.space import Space
from trano.mpc import RCModelType
from trano.mpc.estimation import sanitize_name
from trano.simulate.simulate import SimulationOptions, simulate
from trano.utils.utils import is_success
from validation.bestest.harness import model_hash, reset_element_names
from validation.bestest.kpi import find_variable
from validation.mpc.cases import CASES, ROOT, MPCCase

if TYPE_CHECKING:
    from buildingspy.io.outputfile import Reader  # type: ignore

    from trano.topology import Network

logger = logging.getLogger(__name__)

CACHE_ROOT = ROOT / ".cache" / "mpc_validation"
TOLERANCE = 1e-6
INTERVAL = 900  # [s] output interval of both simulations and of the replayed heat
REFERENCE = "Buildings"


class ZoneComparison(BaseModel):
    zone: str
    rmse: float = Field(description="Root mean square error of the zone air temperature [K]")
    bias: float = Field(description="Mean error, mpc minus Buildings [K]")
    max_abs: float = Field(description="Largest absolute error [K]")
    mean_reference: float = Field(description="Mean Buildings temperature [degC]")
    heat_kwh: float = Field(description="Heat replayed into the zone [kWh]")


class CaseComparison(BaseModel):
    case: str
    model: str = Field(description="mpc zone model, e.g. R3C2, connected")
    zones: list[ZoneComparison]
    wall_time: float

    @property
    def worst_rmse(self) -> float:
        return max(zone.rmse for zone in self.zones)


def case_directory(case: MPCCase, variant: str) -> Path:
    return CACHE_ROOT / case.id / variant


def _network(case: MPCCase, library: Library, directory: Path) -> Network:
    directory.mkdir(parents=True, exist_ok=True)
    reset_element_names()
    network = convert_network(case.path.stem, case.path, library=library)
    network.set_weather_path_to_container_path(directory)
    return network


def _options(case: MPCCase) -> SimulationOptions:
    return SimulationOptions(
        start_time=case.start_time,
        end_time=case.end_time,
        tolerance=TOLERANCE,
        number_of_intervals=(case.end_time - case.start_time) // INTERVAL,
    )


def _simulate(network: Network, directory: Path, options: SimulationOptions, name: str) -> Path:
    """Simulate unless the cached result was produced by the same model; return the result file."""
    model = network.model()
    expected = model_hash(model, name, options)
    result = directory / "results" / f"{network.name}.building_res.mat"
    hash_file = directory / "model.hash"
    if result.exists() and hash_file.exists() and hash_file.read_text() == expected:
        logger.info("%s read from the cache", directory)
        return result
    directory.joinpath("model.mo").write_text(model)
    outcome = simulate(directory, network, options=options, container_name=f"mpc-validation-{name}", model=model)
    output = outcome.output.decode(errors="replace") if isinstance(outcome.output, bytes) else str(outcome.output)
    directory.joinpath("omc.log").write_text(output)
    if not is_success(outcome, options=options):
        raise RuntimeError(f"{network.name} did not simulate; see {directory / 'omc.log'}")
    hash_file.write_text(expected)
    return result


def _reader(path: Path) -> Reader:
    from buildingspy.io.outputfile import Reader

    return Reader(str(path), "dymola")


def _on_grid(reader: Reader, variable: str, grid: np.ndarray) -> np.ndarray:
    time_, values = reader.values(variable)
    # results hold repeated time stamps at events: keep the last value of each
    _, last = np.unique(time_[::-1], return_index=True)
    keep = len(time_) - 1 - last
    return np.asarray(np.interp(grid, time_[keep], values[keep]))


def _zones(network: Network) -> list[Space]:
    return [node for node in network.graph.nodes if isinstance(node, Space)]


def _emitter_heat(reader: Reader, space: Space, grid: np.ndarray) -> np.ndarray:
    """Heat delivered to the zone by its emitters [W]: mean over each interval, from the integral of the power.

    Radiators (hydronic or ideal) expose the heat flow rates into their convective and radiative ports, so the
    heat they deliver is minus their sum; the ideal heating and cooling system reports its heating and cooling.
    """
    total = np.zeros(len(grid))
    for emission in space.emissions:
        try:
            convective = find_variable(reader, f"{emission.name}.heatPortCon.Q_flow")
            radiative = find_variable(reader, f"{emission.name}.heatPortRad.Q_flow")
        except ValueError:
            pass
        else:
            total -= _interval_mean(reader, convective, grid) + _interval_mean(reader, radiative, grid)
            continue
        try:
            heating = find_variable(reader, f"{emission.name}.QHea_flow")
        except ValueError:
            continue  # valves and other non-emitting elements of the emission chain
        total += _interval_mean(reader, heating, grid)
        with contextlib.suppress(ValueError):  # heating only
            total -= _interval_mean(reader, find_variable(reader, f"{emission.name}.QCoo_flow"), grid)
    return total


def _interval_mean(reader: Reader, variable: str, grid: np.ndarray) -> np.ndarray:
    """Mean of a power over [t_k, t_k+1) (held constant over the interval when replayed)."""
    time_, values = reader.values(variable)
    order = np.argsort(time_, kind="stable")
    time_, values = time_[order], values[order]
    energy = np.concatenate([[0.0], np.cumsum(np.diff(time_) * (values[1:] + values[:-1]) / 2)])
    at_grid = np.interp(grid, time_, energy)
    mean = np.diff(at_grid) / np.diff(grid)
    return np.append(mean, mean[-1])


def reference(case: MPCCase) -> tuple[Network, Path]:
    directory = case_directory(case, REFERENCE)
    network = _network(case, Library.from_configuration(REFERENCE), directory)
    return network, _simulate(network, directory, _options(case), f"{case.id}-{REFERENCE}")


def _write_heat(path: Path, grid: np.ndarray, heat: dict[str, np.ndarray]) -> Path:
    index = pd.Timestamp("2025-01-01") + pd.to_timedelta(grid, unit="s")
    pd.DataFrame({f"{zone}_QHea": values for zone, values in heat.items()}, index=index).to_csv(path)
    return path


def compare(
    case_id: str, model_type: RCModelType = RCModelType.r3c2, connected: bool = True, envelope: str = "rc"
) -> CaseComparison:
    """Replay the Buildings heat of the case through the mpc model and compare the zone temperatures."""
    case = CASES[case_id]
    started = time.monotonic()
    buildings_network, buildings_result = reference(case)
    reader = _reader(buildings_result)
    grid = np.arange(case.start_time, case.end_time + 1, INTERVAL, dtype=float)
    spaces = _zones(buildings_network)
    heat = {sanitize_name(space.name): _emitter_heat(reader, space, grid) for space in spaces}
    reference_temperature = {
        sanitize_name(space.name): _on_grid(reader, find_variable(reader, f"{space.name}.heaPorAir.T"), grid)
        for space in spaces
    }

    multilayer = envelope == "multilayer"
    variant = "mpc-Multilayer" if multilayer else f"mpc-{model_type.value}{'-connected' if connected else ''}"
    directory = case_directory(case, variant)
    library = Library.from_configuration("mpc").model_copy(
        update={"rc_model_type": model_type, "connected": connected or multilayer, "envelope": envelope}
    )
    network = _network(case, library, directory)
    network.external_data = _write_heat(directory / "heat.csv", grid - case.start_time, heat)
    result = _simulate(network, directory, _options(case), f"{case.id}-{variant}")
    mpc_reader = _reader(result)

    zones = []
    for zone, expected in reference_temperature.items():
        actual = _on_grid(mpc_reader, f"{zone}_TZon", grid)
        error = actual - expected
        zones.append(
            ZoneComparison(
                zone=zone,
                rmse=float(np.sqrt(np.mean(error**2))),
                bias=float(np.mean(error)),
                max_abs=float(np.max(np.abs(error))),
                mean_reference=float(np.mean(expected) - 273.15),
                heat_kwh=float(np.sum(heat[zone][:-1]) * INTERVAL / 3.6e6),
            )
        )
    return CaseComparison(
        case=case_id,
        model="Multilayer" if multilayer else f"{model_type.value}{', connected' if connected else ''}",
        zones=zones,
        wall_time=time.monotonic() - started,
    )
