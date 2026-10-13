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
import re
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
HYDRONIC_REFERENCE_INTERVAL = 60  # [s] output interval of the Buildings reference of a hydronic case
HYDRONIC_REPLAY_INTERVAL = 300  # [s] interval of the replayed controls of a hydronic case
# the variables of a Buildings reference the comparison reads
REFERENCE_VARIABLES = (
    r".*(heaPorAir\\.T|heatPortCon\\.Q_flow|heatPortRad\\.Q_flow|QHea_flow|QCoo_flow|\\.m_flow|\\.y|"
    r"TSupply\\.T|TReturn\\.T|supTemp\\.T|retTemp\\.T|ySet|yMea|QCon_flow|TableData2D\\.P|_sensor_[0-9]+\\.T)"
)
REFERENCE = "Buildings"


class ZoneComparison(BaseModel):
    zone: str
    rmse: float = Field(description="Root mean square error of the zone air temperature [K]")
    bias: float = Field(description="Mean error, mpc minus Buildings [K]")
    max_abs: float = Field(description="Largest absolute error [K]")
    mean_reference: float = Field(description="Mean Buildings temperature [degC]")
    heat_kwh: float = Field(description="Heat of the emitters of the zone in Buildings [kWh]")
    emitter_heat_kwh: float | None = Field(None, description="Heat of the radiators of the zone, mpc [kWh]")
    emitter_cv_rmse: float | None = Field(None, description="Hourly CV(RMSE) of the radiator heat [-]")
    emitter_nmbe: float | None = Field(None, description="Normalized mean bias of the radiator heat [-]")


class CircuitComparison(BaseModel):
    """Water temperatures of a hydronic circuit, mpc against Buildings."""

    variable: str
    rmse: float = Field(description="Root mean square error [K]")
    bias: float = Field(description="Mean error, mpc minus Buildings [K]")
    mean_reference: float = Field(description="Mean Buildings temperature [degC]")


class CaseComparison(BaseModel):
    case: str
    model: str = Field(description="mpc zone model, e.g. R3C2, connected")
    zones: list[ZoneComparison]
    circuits: list[CircuitComparison] = Field(default_factory=list)
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


def _options(case: MPCCase, reference: bool = False) -> SimulationOptions:
    fine = reference and case.hydronic
    return SimulationOptions(
        start_time=case.start_time,
        end_time=case.end_time,
        tolerance=TOLERANCE,
        number_of_intervals=(case.end_time - case.start_time) // (HYDRONIC_REFERENCE_INTERVAL if fine else INTERVAL),
        variable_filter=REFERENCE_VARIABLES if fine else None,
    )


_MPC_PACKAGE = re.compile(r"^package MPC\b.*?^end MPC;", flags=re.DOTALL | re.MULTILINE)


def _hashed_text(model: str, network: Network) -> str:
    """The model text the cache depends on: every generated package embeds ``Trano.MPC``, which a model of another
    library never uses, so that the reference simulations survive changes of the mpc components."""
    return model if network.library.name == "mpc" else _MPC_PACKAGE.sub("", model)


def _simulate(network: Network, directory: Path, options: SimulationOptions, name: str) -> Path:
    """Simulate unless the cached result was produced by the same model; return the result file."""
    model = network.model()
    expected = model_hash(_hashed_text(model, network), name, options)
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
    return network, _simulate(network, directory, _options(case, reference=True), f"{case.id}-{REFERENCE}")


def _write_data(path: Path, grid: np.ndarray, columns: dict[str, np.ndarray]) -> Path:
    index = pd.Timestamp("2025-01-01") + pd.to_timedelta(grid, unit="s")
    pd.DataFrame(columns, index=index).to_csv(path)
    return path


def _hydronic_replay(reader: Reader, network: Network, grid: np.ndarray) -> dict[str, np.ndarray]:
    """The hydronic controls of the mpc model, as Buildings ran them: branch flows, three-way valve positions and
    boiler part load ratios or compressor speeds, as means over each interval (they move faster than it)."""
    from trano.elements.system import Boiler, ThreeWayValve, Valve
    from trano.mpc.components.hydronics import hydronic_graph

    columns = {}
    for element in hydronic_graph(network).nodes:
        name = sanitize_name(element.name)
        if isinstance(element, Valve):
            columns[f"{name}_m_flow"] = _interval_mean(reader, find_variable(reader, f"{element.name}.m_flow"), grid)
        elif isinstance(element, ThreeWayValve):
            columns[f"{name}_y"] = _interval_mean(reader, find_variable(reader, f"{element.name}.y"), grid)
        elif isinstance(element, Boiler):
            columns[f"{name}_y"] = _interval_mean(reader, find_variable(reader, _production_control(element)), grid)
    return columns


def _production_control(element: object) -> str:
    """Control of a production in the Buildings model: boiler part load ratio or compressor speed."""
    if getattr(element, "variant", "default") == "air_water_heat_pump":
        return f"{element.name}.airToWaterTableData2D.ySet"  # type: ignore[attr-defined]
    return f"{element.name}.boi.y"  # type: ignore[attr-defined]


def _hourly(values: np.ndarray) -> np.ndarray:
    steps = 3600 // INTERVAL
    usable = len(values) - len(values) % steps
    return np.asarray(values[:usable].reshape(-1, steps).mean(axis=1))


def _cv_rmse_and_nmbe(actual: np.ndarray, expected: np.ndarray) -> tuple[float, float]:
    """ASHRAE Guideline 14 hourly CV(RMSE) and NMBE of a heat flow."""
    actual, expected = _hourly(actual), _hourly(expected)
    mean = float(np.mean(expected))
    if abs(mean) < 1e-9:
        return 0.0, 0.0
    return float(np.sqrt(np.mean((actual - expected) ** 2)) / mean), float(
        np.sum(actual - expected) / len(expected) / mean
    )


def _radiator_heat(reader: Reader, network: Network, grid: np.ndarray) -> dict[str, np.ndarray]:
    """Heat the radiators of the mpc model gave each zone [W], mean over each interval."""
    from trano.mpc.components.hydronics import hydronic_emitters, zone_of

    heat: dict[str, np.ndarray] = {}
    for radiator in hydronic_emitters(network):
        space = zone_of(radiator, network)
        if space is None:
            continue
        name = sanitize_name(str(radiator.name))
        delivered = -(
            _interval_mean(reader, f"rc.{name}.heatPortCon.Q_flow", grid)
            + _interval_mean(reader, f"rc.{name}.heatPortRad.Q_flow", grid)
        )
        zone = sanitize_name(space.name)
        heat[zone] = heat.get(zone, 0) + delivered
    return heat


def _circuits(reader: Reader, mpc_reader: Reader, network: Network, grid: np.ndarray) -> list[CircuitComparison]:
    """Return temperature of each production and temperature of each sensor of the circuits."""
    from trano.elements.system import Boiler, TemperatureSensor
    from trano.mpc.components.hydronics import hydronic_graph

    pairs = []
    for element in hydronic_graph(network).nodes:
        name = sanitize_name(element.name)
        if isinstance(element, Boiler) and getattr(element, "variant", "default") == "air_water_heat_pump":
            pairs.append((f"{name} supply", f"{element.name}.supTemp.T", f"rc.{name}.T"))
            pairs.append((f"{name} return", f"{element.name}.retTemp.T", f"rc.{name}.inlet.T"))
        elif isinstance(element, Boiler):
            pairs.append((f"{name} supply", f"{element.name}.TSupply.T", f"rc.{name}.T"))
            pairs.append((f"{name} return", f"{element.name}.TReturn.T", f"rc.{name}.inlet.T"))
        elif isinstance(element, TemperatureSensor):
            pairs.append((f"{name}", f"{element.name}.T", f"rc.{name}.T"))
    comparisons = []
    for label, reference_variable, mpc_variable in pairs:
        try:
            expected = _on_grid(reader, find_variable(reader, reference_variable), grid)
        except ValueError:
            continue
        error = _on_grid(mpc_reader, mpc_variable, grid) - expected
        comparisons.append(
            CircuitComparison(
                variable=label,
                rmse=float(np.sqrt(np.mean(error**2))),
                bias=float(np.mean(error)),
                mean_reference=float(np.mean(expected) - 273.15),
            )
        )
    return comparisons


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
    replay = np.arange(case.start_time, case.end_time + 1, HYDRONIC_REPLAY_INTERVAL if case.hydronic else INTERVAL)
    columns = {
        f"{sanitize_name(space.name)}_QHea": _emitter_heat(reader, space, replay.astype(float)) for space in spaces
    }
    if multilayer:
        columns |= _hydronic_replay(reader, buildings_network, replay.astype(float))
    network.external_data = _write_data(directory / "heat.csv", replay - case.start_time, columns)
    result = _simulate(network, directory, _options(case), f"{case.id}-{variant}")
    mpc_reader = _reader(result)
    radiator_heat = _radiator_heat(mpc_reader, network, grid) if multilayer else {}

    zones = []
    for zone, expected in reference_temperature.items():
        actual = _on_grid(mpc_reader, f"{zone}_TZon", grid)
        error = actual - expected
        emitter: dict[str, float] = {}
        if zone in radiator_heat:
            cv_rmse, nmbe = _cv_rmse_and_nmbe(radiator_heat[zone], heat[zone])
            emitter = {
                "emitter_heat_kwh": float(np.sum(radiator_heat[zone][:-1]) * INTERVAL / 3.6e6),
                "emitter_cv_rmse": cv_rmse,
                "emitter_nmbe": nmbe,
            }
        zones.append(
            ZoneComparison(
                zone=zone,
                rmse=float(np.sqrt(np.mean(error**2))),
                bias=float(np.mean(error)),
                max_abs=float(np.max(np.abs(error))),
                mean_reference=float(np.mean(expected) - 273.15),
                heat_kwh=float(np.sum(heat[zone][:-1]) * INTERVAL / 3.6e6),
                **emitter,
            )
        )
    return CaseComparison(
        case=case_id,
        model="Multilayer" if multilayer else f"{model_type.value}{', connected' if connected else ''}",
        zones=zones,
        circuits=_circuits(reader, mpc_reader, network, grid) if multilayer else [],
        wall_time=time.monotonic() - started,
    )
