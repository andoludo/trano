"""Replay a Buildings simulation through the MPC model (``mpc`` or ``FastModel`` library) of the same YAML and compare.

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
from trano.elements.bus import DataBus
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
FINE_REFERENCE_INTERVAL = 60  # [s] output interval of the Buildings reference of a fine-replay case
FINE_REPLAY_INTERVAL = 300  # [s] interval of the replayed controls of a fine-replay case
# the variables of a Buildings reference the comparison reads
REFERENCE_VARIABLES = (
    r".*(heaPorAir\\.T|heatPortCon\\.Q_flow|heatPortRad\\.Q_flow|QHea_flow|QCoo_flow|\\.m_flow|\\.y|"
    r"TSupply\\.T|TReturn\\.T|supTemp\\.T|retTemp\\.T|ySet|yMea|QCon_flow|TableData2D\\.P|_sensor_[0-9]+\\.T|"
    r"TSup\\.T|PFan\\.y|PEle\\.y|QFue\\.y|pv_[0-9]+\\.P)"
)
REFERENCE = "Buildings"
REFERENCES = ("Buildings", "IDEAS")  # libraries whose simulation FastModel can replay
# the zone air temperature in the reference libraries
ZONE_AIR_TEMPERATURE = {"Buildings": "heaPorAir.T", "IDEAS": "TAir"}


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
    circulating: float = Field(1.0, description="Share of the time compared: while the water circulates")


class EnergyComparison(BaseModel):
    """Energy drawn or produced by a system over the case, mpc meter against Buildings."""

    system: str
    quantity: str = Field(description="fuel, electricity or production")
    reference_kwh: float
    model_kwh: float

    @property
    def deviation(self) -> float:
        return (self.model_kwh - self.reference_kwh) / self.reference_kwh if self.reference_kwh else 0.0


class CaseComparison(BaseModel):
    case: str
    model: str = Field(description="FastModel, or an RC model type of the mpc library")
    reference: str = Field(REFERENCE, description="Library of the simulation replayed and compared with")
    zones: list[ZoneComparison]
    circuits: list[CircuitComparison] = Field(default_factory=list)
    energies: list[EnergyComparison] = Field(default_factory=list)
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


def _options(case: MPCCase, reference: bool = False, library: str = REFERENCE) -> SimulationOptions:
    fine = reference and case.fine_replay
    variables = REFERENCE_VARIABLES if library == REFERENCE else REFERENCE_VARIABLES.replace(".*(", r".*(\\.TAir|", 1)
    return SimulationOptions(
        start_time=case.start_time,
        end_time=case.end_time,
        tolerance=TOLERANCE,
        number_of_intervals=(case.end_time - case.start_time) // (FINE_REFERENCE_INTERVAL if fine else INTERVAL),
        variable_filter=variables if fine else None,
    )


_MPC_PACKAGE = re.compile(r"^package MPC\b.*?^end MPC;", flags=re.DOTALL | re.MULTILINE)


def _hashed_text(model: str, network: Network) -> str:
    """The model text the cache depends on: every generated package embeds ``Trano.MPC``, which only the models of
    the ``mpc`` library use, so that the reference simulations survive changes of that package."""
    return model if network.library.name == "mpc" else _MPC_PACKAGE.sub("", model)


def _data_bus(network: Network) -> DataBus | None:
    """The data bus of a FastModel network, with the external data its runnable model replays."""
    if network.library.name != FAST_MODEL_MPC or not network.external_data:
        return None
    return DataBus(external_data=network.external_data)


def _simulate(network: Network, directory: Path, options: SimulationOptions, name: str) -> Path:
    """Simulate unless the cached result was produced by the same model; return the result file."""
    model = network.model(data_bus=_data_bus(network))
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


def reference(case: MPCCase, library: str = REFERENCE) -> tuple[Network, Path]:
    """The simulation of the case with ``library`` (Buildings or IDEAS), with its own controls."""
    if library not in REFERENCES:
        raise ValueError(f"FastModel replays {', '.join(REFERENCES)} simulations, not {library}")
    directory = case_directory(case, library)
    network = _network(case, Library.from_configuration(library), directory)
    return network, _simulate(
        network, directory, _options(case, reference=True, library=library), f"{case.id}-{library}"
    )


def _write_data(path: Path, grid: np.ndarray, columns: dict[str, np.ndarray]) -> Path:
    index = pd.Timestamp("2025-01-01") + pd.to_timedelta(grid, unit="s")
    pd.DataFrame(columns, index=index).to_csv(path)
    return path


def _of_type(network: Network, kind: type) -> list:  # type: ignore[type-arg]
    return sorted((node for node in network.graph.nodes if isinstance(node, kind)), key=lambda node: str(node.name))


def _is_hydronic_radiator(element: object) -> bool:
    from trano.elements.system import Radiator

    return isinstance(element, Radiator) and getattr(element, "variant", "default") == "default"


def _production_control(element: object) -> str:
    """Control of a production in the Buildings model: boiler part load ratio or compressor speed."""
    if getattr(element, "variant", "default") == "air_water_heat_pump":
        return f"{element.name}.airToWaterTableData2D.ySet"  # type: ignore[attr-defined]
    return f"{element.name}.boi.y"  # type: ignore[attr-defined]


def _fast_model_replay(reader: Reader, network: Network, grid: np.ndarray) -> dict[str, np.ndarray]:
    """The inputs of the FastModel model as Buildings ran its systems, as means over each interval (they move faster
    than it): the heat of each ideal emitter (<emitter>_Q), the flow of each valve relative to its nominal flow,
    the three-way valve positions, the part load ratios or compressor speeds, the fan speeds (<system>_y)."""
    from trano.elements.system import AirHandlingUnit, Boiler, Duct, IdealHeatingCooling, Radiator, ThreeWayValve, Valve

    columns: dict[str, np.ndarray] = {}
    for emitter in [*_of_type(network, Radiator), *_of_type(network, IdealHeatingCooling)]:
        if not _is_hydronic_radiator(emitter):
            columns[f"{emitter.name}_Q"] = _element_heat(reader, emitter, grid)
    for valve in _of_type(network, Valve):
        nominal = float(valve.processed_parameters(Library.from_configuration(FAST_MODEL_MPC))["m_flow_nominal"])
        columns[f"{valve.name}_y"] = (
            _interval_mean(reader, find_variable(reader, f"{valve.name}.m_flow"), grid) / nominal
        )
    for valve in _of_type(network, ThreeWayValve):
        columns[f"{valve.name}_y"] = _interval_mean(reader, find_variable(reader, f"{valve.name}.y"), grid)
    for production in _of_type(network, Boiler):
        columns[f"{production.name}_y"] = _interval_mean(
            reader, find_variable(reader, _production_control(production)), grid
        )
    for unit in _of_type(network, AirHandlingUnit):
        ducts = [duct for duct in network.graph.succ[unit] if isinstance(duct, Duct)]
        library = Library.from_configuration(FAST_MODEL_MPC)
        nominal = sum(float(duct.processed_parameters(library)["m_flow_nominal"]) for duct in ducts)
        flow = sum(
            (_interval_mean(reader, find_variable(reader, f"{duct.name}.m_flow"), grid) for duct in ducts),
            start=np.zeros(len(grid)),
        )
        columns[f"{unit.name}_y"] = flow / nominal if nominal else flow
    return columns


def _element_heat(reader: Reader, emission: object, grid: np.ndarray) -> np.ndarray:
    """Heat delivered by one emitter of the Buildings model [W], mean over each interval."""
    name = emission.name  # type: ignore[attr-defined]
    try:
        convective = find_variable(reader, f"{name}.heatPortCon.Q_flow")
        radiative = find_variable(reader, f"{name}.heatPortRad.Q_flow")
    except ValueError:
        heating = _interval_mean(reader, find_variable(reader, f"{name}.QHea_flow"), grid)
        with contextlib.suppress(ValueError):  # heating only
            heating = heating - _interval_mean(reader, find_variable(reader, f"{name}.QCoo_flow"), grid)
        return heating
    return np.asarray(-(_interval_mean(reader, convective, grid) + _interval_mean(reader, radiative, grid)))


def _energies(reader: Reader, model_reader: Reader, network: Network, grid: np.ndarray) -> list[EnergyComparison]:
    """Fuel and electricity of the productions and fans, production of the PV arrays, against the meters."""
    from trano.elements.solar import Photovoltaic
    from trano.elements.system import AirHandlingUnit, Boiler

    rows: list[tuple[str, str, str, str]] = []  # system, quantity, Buildings power, FastModel meter
    for production in _of_type(network, Boiler):
        if getattr(production, "variant", "default") == "air_water_heat_pump":
            rows.append((production.name, "electricity", f"{production.name}.PEle.y", f"mpc.{production.name}.EEle"))
        else:
            rows.append((production.name, "fuel", f"{production.name}.QFue.y", f"mpc.{production.name}.EFue"))
    rows += [
        (unit.name, "electricity", f"{unit.name}.PFan.y", f"mpc.{unit.name}.EFan")
        for unit in _of_type(network, AirHandlingUnit)
    ]
    rows += [
        (array.name, "production", f"{array.name}.P", f"mpc.{array.name}.EPro")
        for array in _of_type(network, Photovoltaic)
    ]
    comparisons = []
    for system, quantity, power, meter in rows:
        try:
            reference = float(
                np.sum(_interval_mean(reader, find_variable(reader, power), grid)[:-1]) * INTERVAL / 3.6e6
            )
        except ValueError:
            continue
        readings = _on_grid(model_reader, meter, grid)
        comparisons.append(
            EnergyComparison(
                system=system,
                quantity=quantity,
                reference_kwh=abs(reference),
                model_kwh=float(readings[-1] - readings[0]),
            )
        )
    return comparisons


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
    """Heat the radiators of the FastModel model gave each zone [W], mean over each interval."""
    heat: dict[str, np.ndarray] = {}
    for space in _zones(network):
        for emission in space.emissions:
            if _is_hydronic_radiator(emission):
                delivered = _interval_mean(reader, f"mpc.{emission.name}.Q_flow", grid)
                heat[space.name] = heat.get(space.name, 0) + delivered
    return heat


CIRCULATING = 0.1  # share of the largest flow of the radiators above which the water temperatures are compared


def _circulating(reader: Reader, network: Network, grid: np.ndarray) -> np.ndarray:
    """Times when the water circulates: the temperature sensors of Buildings are dynamic (their time constant grows
    as the flow drops), so that at a near-zero flow their reading is that of stagnant water, not of the circuit."""
    from trano.elements.system import Valve

    flow = sum(
        (
            _interval_mean(reader, find_variable(reader, f"{valve.name}.m_flow"), grid)
            for valve in _of_type(network, Valve)
        ),
        start=np.zeros(len(grid)),
    )
    return np.asarray(flow > CIRCULATING * flow.max()) if flow.max() > 0 else np.ones(len(grid), dtype=bool)


def _circuits(reader: Reader, model_reader: Reader, network: Network, grid: np.ndarray) -> list[CircuitComparison]:
    """Supply and return temperatures of each production and temperature of each sensor, while the water
    circulates, and the supply air of the ventilation units."""
    from trano.elements.system import AirHandlingUnit, Boiler, TemperatureSensor

    pairs = []
    for production in _of_type(network, Boiler):
        heat_pump = getattr(production, "variant", "default") == "air_water_heat_pump"
        supply, back = ("supTemp", "retTemp") if heat_pump else ("TSupply", "TReturn")
        pairs.append((f"{production.name} supply", f"{production.name}.{supply}.T", f"mpc.{production.name}.T"))
        pairs.append((f"{production.name} return", f"{production.name}.{back}.T", f"mpc.{production.name}.TIn"))
    pairs += [
        (str(sensor.name), f"{sensor.name}.T", f"mpc.{sensor.name}.T")
        for sensor in _of_type(network, TemperatureSensor)
    ]
    active = _circulating(reader, network, grid)
    always = np.ones(len(grid), dtype=bool)
    masks = dict.fromkeys(label for label, _, _ in pairs)
    for unit in _of_type(network, AirHandlingUnit):
        pairs.append((f"{unit.name} supply air", f"{unit.name}.TSup.T", f"mpc.{unit.name}.TSup"))
        masks[f"{unit.name} supply air"] = always
    comparisons = []
    for label, reference_variable, model_variable in pairs:
        mask = masks.get(label)
        selected = active if mask is None else mask
        try:
            expected = _on_grid(reader, find_variable(reader, reference_variable), grid)[selected]
        except ValueError:
            continue
        error = _on_grid(model_reader, model_variable, grid)[selected] - expected
        comparisons.append(
            CircuitComparison(
                variable=label,
                rmse=float(np.sqrt(np.mean(error**2))),
                bias=float(np.mean(error)),
                mean_reference=float(np.mean(expected) - 273.15),
                circulating=float(np.mean(selected)),
            )
        )
    return comparisons


FAST_MODEL = "FastModel"
FAST_MODEL_MPC = "FastModelMPC"  # FastModel with the controls as inputs: the replay drives them


def compare(case_id: str, model: str = FAST_MODEL, reference_library: str = REFERENCE) -> CaseComparison:
    """Replay what the systems of the ``reference_library`` model (Buildings or IDEAS) did through the model of the
    same YAML file and compare: ``FastModel`` or an RC model type of the flat ``mpc`` library (R1C1, R3C2, ...)."""
    case = CASES[case_id]
    started = time.monotonic()
    buildings_network, buildings_result = reference(case, reference_library)
    reader = _reader(buildings_result)
    grid = np.arange(case.start_time, case.end_time + 1, INTERVAL, dtype=float)
    spaces = _zones(buildings_network)
    heat = {sanitize_name(space.name): _emitter_heat(reader, space, grid) for space in spaces}
    reference_temperature = {
        sanitize_name(space.name): _on_grid(
            reader, find_variable(reader, f"{space.name}.{ZONE_AIR_TEMPERATURE[reference_library]}"), grid
        )
        for space in spaces
    }
    fast_model = model == FAST_MODEL
    variant = FAST_MODEL if fast_model else f"mpc-{model}"
    directory = case_directory(case, variant if reference_library == REFERENCE else f"{variant}-{reference_library}")
    library = (
        Library.from_configuration(FAST_MODEL_MPC)
        if fast_model
        else Library.from_configuration("mpc").model_copy(update={"rc_model_type": RCModelType(model)})
    )
    network = _network(case, library, directory)
    replay = np.arange(case.start_time, case.end_time + 1, FINE_REPLAY_INTERVAL if case.fine_replay else INTERVAL)
    if fast_model:
        columns = _fast_model_replay(reader, buildings_network, replay.astype(float))
    else:
        columns = {
            f"{sanitize_name(space.name)}_QHea": _emitter_heat(reader, space, replay.astype(float)) for space in spaces
        }
    if columns:  # nothing to replay in a free-floating case
        network.external_data = _write_data(directory / "replay.csv", replay - case.start_time, columns)
    name = f"{case.id}-{model}" if reference_library == REFERENCE else f"{case.id}-{model}-{reference_library}"
    result = _simulate(network, directory, _options(case), name)
    model_reader = _reader(result)
    radiator_heat = _radiator_heat(model_reader, network, grid) if fast_model else {}

    zones = []
    for zone, expected in reference_temperature.items():
        variable = f"mpc.{zone}.TAir" if fast_model else f"{zone}_TZon"
        actual = _on_grid(model_reader, variable, grid)
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
        model=model,
        reference=reference_library,
        zones=zones,
        circuits=_circuits(reader, model_reader, network, grid) if fast_model else [],
        energies=_energies(reader, model_reader, network, grid) if fast_model else [],
        wall_time=time.monotonic() - started,
    )
