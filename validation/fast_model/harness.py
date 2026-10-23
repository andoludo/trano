"""Simulate a case with Buildings and with FastModel in OpenModelica; hourly series and KPIs of each run.

Both libraries are generated from the same YAML file (perturbed or not) and simulated by OpenModelica in the same
container image: FastModel is a Modelica library, so the comparison does not go through the CasADi export. The runs
are cached by the generated model (as ``validation.bestest``); the hourly series and the KPIs of each run are written
to ``validation/fast_model/results/<case>/<variant>/`` and committed, so that the report renders without Docker.
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
import yaml
from pydantic import BaseModel, Field

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.elements.space import Space
from trano_simulate.simulate import CONTAINER_MODEL_DIR, SimulationOptions
from validation.bestest.harness import reset_element_names
from validation.bestest.kpi import find_variable
from validation.fast_model.cases import BASE, CASES, ROOT, StudyCase
from validation.mpc.harness import _interval_mean, _of_type, _on_grid, _reader, _simulate

if TYPE_CHECKING:
    from buildingspy.io.outputfile import Reader  # type: ignore

    from trano.topology import Network

logger = logging.getLogger(__name__)

CACHE_ROOT = ROOT / ".cache" / "fast_model_study"
RESULTS = ROOT / "validation" / "fast_model" / "results"
BUILDINGS, FAST_MODEL = "Buildings", "FastModel"
LIBRARIES = (BUILDINGS, FAST_MODEL)
TOLERANCE = 1e-6
HOUR = 3600.0
KELVIN = 273.15
# The result variables the series need, in either library (OpenModelica's variableFilter): zone air temperatures,
# the heat of the emitters, the powers or meters of the productions, fans and PV arrays, the outdoor temperature.
VARIABLES = (
    r".*(heaPorAir\\.T|\\.TAir|QHea_flow|QCoo_flow|heatPortCon\\.Q_flow|heatPortRad\\.Q_flow|"
    r"PEle\\.y|QFue\\.y|PFan\\.y|\\.PEle|\\.QFue|\\.PFan|\\.EEle|\\.EFue|\\.EFan|\\.EPro|pv_[0-9]+\\.P|TDryBul)"
)
ZONE_TEMPERATURE = {BUILDINGS: "heaPorAir.T", FAST_MODEL: "TAir"}
METERS = ("electricity", "fuel", "fans", "pv")
LOG_LINES = 60  # of the OpenModelica log shown when a run fails


class ZoneKpis(BaseModel):
    zone: str
    mean_temperature: float = Field(description="degC, of the hourly means")
    min_temperature: float = Field(description="degC, lowest hourly mean")
    max_temperature: float = Field(description="degC, highest hourly mean")
    heating_kwh: float = Field(description="heat delivered by the emitters of the zone")
    cooling_kwh: float
    peak_heating_kw: float = Field(description="largest hourly mean")
    peak_cooling_kw: float


class RunKpis(BaseModel):
    case: str
    variant: str
    library: str
    zones: list[ZoneKpis]
    heating_kwh: float
    cooling_kwh: float
    peak_heating_kw: float
    peak_cooling_kw: float
    electricity_kwh: float | None = Field(None, description="of the heat pumps")
    fuel_kwh: float | None = Field(None, description="of the boilers, lower heating value")
    fans_kwh: float | None = Field(None, description="of the ventilation units")
    pv_kwh: float | None = Field(None, description="production of the PV arrays")
    wall_time: float = Field(description="s of the OpenModelica run: translation, compilation and simulation")
    model_hash: str

    def zone(self, name: str) -> ZoneKpis:
        return next(zone for zone in self.zones if zone.zone == name)

    def value(self, name: str) -> float | None:
        """A case-level KPI by name (``heating_kwh``, ``electricity_kwh``, ...), or the mean, lowest or highest of
        the zone temperatures (``mean_temperature``, ``min_temperature``, ``max_temperature``)."""
        if name == "mean_temperature":
            return float(np.mean([zone.mean_temperature for zone in self.zones]))
        if name == "min_temperature":
            return min(zone.min_temperature for zone in self.zones)
        if name == "max_temperature":
            return max(zone.max_temperature for zone in self.zones)
        value = getattr(self, name)
        return None if value is None else float(value)


class ZoneFidelity(BaseModel):
    """Hourly series of a zone, FastModel against Buildings."""

    zone: str
    rmse: float = Field(description="K, of the air temperature")
    bias: float = Field(description="K, FastModel minus Buildings")
    max_abs: float = Field(description="K")
    mean_reference: float = Field(description="degC, Buildings")
    heating_cv_rmse: float | None = Field(None, description="hourly CV(RMSE) of the heat delivered (ASHRAE G14)")
    heating_nmbe: float | None = Field(None, description="hourly NMBE of the heat delivered")
    heating_deviation: float | None = Field(None, description="relative difference of the heat delivered")


def result_directory(case_id: str, variant: str) -> Path:
    return RESULTS / case_id / variant


def series_path(case_id: str, variant: str, library: str) -> Path:
    return result_directory(case_id, variant) / f"{library}.csv.gz"


def kpis_path(case_id: str, variant: str, library: str) -> Path:
    return result_directory(case_id, variant) / f"{library}.json"


def zone_names(frame: pd.DataFrame) -> list[str]:
    return [str(column).removesuffix(".T") for column in frame.columns if str(column).endswith(".T")]


def variant_yaml(case: StudyCase, variant: str, directory: Path) -> Path:
    """The YAML of the case with the change of ``variant`` applied, written next to the run (same file name, so that
    the elements keep the names of the base case)."""
    document = yaml.safe_load(case.path.read_text())
    perturbation = case.perturbation(variant)
    if perturbation is not None:
        perturbation.change(document)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / case.path.name
    path.write_text(yaml.safe_dump(document, sort_keys=False, width=200))
    return path


def network_for(path: Path, library: str, directory: Path) -> Network:
    reset_element_names()
    network = convert_network(path.stem, path, library=Library.from_configuration(library))
    network.set_weather_path(directory, CONTAINER_MODEL_DIR)
    return network


def options(case: StudyCase) -> SimulationOptions:
    return SimulationOptions(
        start_time=case.start_time,
        end_time=case.end_time,
        tolerance=TOLERANCE,
        number_of_intervals=(case.end_time - case.start_time) // case.output_interval,
        variable_filter=VARIABLES,
    )


def _container_suffix(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]", "-", text)


def run(case_id: str, variant: str = BASE, library: str = BUILDINGS, force: bool = False) -> RunKpis:
    """Simulate (unless cached), then write the hourly series and the KPIs of the run to ``results/``."""
    case = CASES[case_id]
    if variant not in case.variants:
        raise KeyError(f"Case {case_id} has no variant {variant!r}; its variants are {case.variants}")
    directory = CACHE_ROOT / case.id / variant / library
    network = network_for(variant_yaml(case, variant, directory), library, directory)
    hash_file, timing = directory / "model.hash", directory / "wall_time.txt"
    if force:
        hash_file.unlink(missing_ok=True)
    before = hash_file.read_text() if hash_file.exists() else None
    started = time.monotonic()
    try:
        result = _simulate(network, directory, options(case), _container_suffix(f"study-{case.id}-{variant}-{library}"))
    except RuntimeError:
        log = directory / "omc.log"
        if log.exists():
            logger.error("Last lines of %s:\n%s", log, "\n".join(log.read_text().splitlines()[-LOG_LINES:]))
        raise
    if hash_file.read_text() != before or not timing.exists():
        timing.write_text(f"{time.monotonic() - started:.1f}")
    frame = hourly_series(network, library, result, case)
    kpis = compute_kpis(case, variant, library, frame, float(timing.read_text()), hash_file.read_text())
    result_directory(case.id, variant).mkdir(parents=True, exist_ok=True)
    frame.to_csv(series_path(case.id, variant, library), float_format="%.4f")
    kpis_path(case.id, variant, library).write_text(kpis.model_dump_json(indent=1))
    logger.info(
        "%s %s %s: heating %.0f kWh, %d zones, %.0f s",
        case.id,
        variant,
        library,
        kpis.heating_kwh,
        len(kpis.zones),
        kpis.wall_time,
    )
    return kpis


def _mean(reader: Reader, variable: str, edges: np.ndarray) -> np.ndarray:
    """Mean of a signal over each interval between the ``edges``."""
    return np.asarray(_interval_mean(reader, variable, edges)[:-1])


def _first_variable(reader: Reader, suffix: str) -> str | None:
    return next((str(name) for name in reader.varNames() if name == suffix or name.endswith("." + suffix)), None)


def _zone_loads(reader: Reader, space: Space, edges: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Heating and cooling delivered to a zone by its emitters [W], means over the intervals: the heating and cooling
    of an ideal system, else the heat the radiators give to the air and to the surfaces."""
    heating, cooling = np.zeros(len(edges) - 1), np.zeros(len(edges) - 1)
    for emission in space.emissions:
        name = str(emission.name)
        try:
            heating += _mean(reader, find_variable(reader, f"{name}.QHea_flow"), edges)
        except ValueError:
            pass
        else:
            with contextlib.suppress(ValueError):  # heating only
                cooling += _mean(reader, find_variable(reader, f"{name}.QCoo_flow"), edges)
            continue
        try:
            convective = find_variable(reader, f"{name}.heatPortCon.Q_flow")
            radiative = find_variable(reader, f"{name}.heatPortRad.Q_flow")
        except ValueError:
            continue  # valves and other elements of the emission chain that give no heat
        heating -= _mean(reader, convective, edges) + _mean(reader, radiative, edges)
    return heating, cooling


def _meter(reader: Reader, powers: tuple[str, ...], energy: str, edges: np.ndarray) -> np.ndarray | None:
    """Mean power over the intervals [W], from a power variable or from the increments of a kWh meter."""
    for suffix in powers:
        with contextlib.suppress(ValueError):
            return _mean(reader, find_variable(reader, suffix), edges)
    try:
        readings = _on_grid(reader, find_variable(reader, energy), edges)
    except ValueError:
        return None
    return np.asarray(np.diff(readings) * 1000.0 * HOUR / np.diff(edges))


def _add(columns: dict[str, np.ndarray], name: str, values: np.ndarray | None) -> None:
    if values is not None:
        columns[name] = columns[name] + values if name in columns else values


def _meters(reader: Reader, network: Network, edges: np.ndarray) -> dict[str, np.ndarray]:
    """Electricity of the heat pumps, fuel of the boilers, electricity of the fans, production of the PV [W]."""
    from trano.elements.solar import Photovoltaic
    from trano.elements.system import AirHandlingUnit, Boiler

    columns: dict[str, np.ndarray] = {}
    for production in _of_type(network, Boiler):
        name = str(production.name)
        if getattr(production, "variant", "default") == "air_water_heat_pump":
            _add(columns, "electricity", _meter(reader, (f"{name}.PEle.y", f"{name}.PEle"), f"{name}.EEle", edges))
        else:
            _add(columns, "fuel", _meter(reader, (f"{name}.QFue.y", f"{name}.QFue"), f"{name}.EFue", edges))
    for unit in _of_type(network, AirHandlingUnit):
        name = str(unit.name)
        _add(columns, "fans", _meter(reader, (f"{name}.PFan.y", f"{name}.PFan"), f"{name}.EFan", edges))
    for array in _of_type(network, Photovoltaic):
        name = str(array.name)
        _add(columns, "pv", _meter(reader, (f"{name}.P",), f"{name}.EPro", edges))
    return columns


def hourly_series(network: Network, library: str, result: Path, case: StudyCase) -> pd.DataFrame:
    """Hourly means over the case: outdoor temperature, air temperature [degC] and heating and cooling [W] of each
    zone, their totals and the meters of the systems [W]; indexed by the time of the start of the hour [s]."""
    reader = _reader(result)
    edges = np.arange(case.start_time, case.end_time + 1, HOUR, dtype=float)
    columns: dict[str, np.ndarray] = {}
    outdoor = _first_variable(reader, "TDryBul")
    if outdoor is not None:
        columns["TOut"] = _mean(reader, outdoor, edges) - KELVIN
    heating, cooling = np.zeros(len(edges) - 1), np.zeros(len(edges) - 1)
    for space in _of_type(network, Space):
        name = str(space.name)
        temperature = find_variable(reader, f"{name}.{ZONE_TEMPERATURE[library]}")
        columns[f"{name}.T"] = _mean(reader, temperature, edges) - KELVIN
        columns[f"{name}.heating"], columns[f"{name}.cooling"] = _zone_loads(reader, space, edges)
        heating += columns[f"{name}.heating"]
        cooling += columns[f"{name}.cooling"]
    columns["heating"], columns["cooling"] = heating, cooling
    columns.update(_meters(reader, network, edges))
    return pd.DataFrame(columns, index=pd.Index(edges[:-1], name="time"))


def compute_kpis(  # noqa: PLR0913
    case: StudyCase, variant: str, library: str, frame: pd.DataFrame, wall_time: float, model_hash: str
) -> RunKpis:
    zones = []
    for name in zone_names(frame):
        temperature = frame[f"{name}.T"].to_numpy(dtype=float)
        heating = frame[f"{name}.heating"].to_numpy(dtype=float)
        cooling = frame[f"{name}.cooling"].to_numpy(dtype=float)
        zones.append(
            ZoneKpis(
                zone=name,
                mean_temperature=float(temperature.mean()),
                min_temperature=float(temperature.min()),
                max_temperature=float(temperature.max()),
                heating_kwh=float(heating.sum() / 1000),
                cooling_kwh=float(cooling.sum() / 1000),
                peak_heating_kw=float(heating.max() / 1000),
                peak_cooling_kw=float(cooling.max() / 1000),
            )
        )
    meters = {f"{name}_kwh": float(frame[name].sum() / 1000) for name in METERS if name in frame}
    return RunKpis(
        case=case.id,
        variant=variant,
        library=library,
        zones=zones,
        heating_kwh=float(frame["heating"].sum() / 1000),
        cooling_kwh=float(frame["cooling"].sum() / 1000),
        peak_heating_kw=float(frame["heating"].max() / 1000),
        peak_cooling_kw=float(frame["cooling"].max() / 1000),
        wall_time=wall_time,
        model_hash=model_hash,
        **meters,
    )


def compare_series(reference: pd.DataFrame, model: pd.DataFrame) -> list[ZoneFidelity]:
    """The zones of ``model`` (FastModel) against those of ``reference`` (Buildings), hour by hour."""
    rows = []
    for name in zone_names(reference):
        expected = reference[f"{name}.T"].to_numpy(dtype=float)
        error = model[f"{name}.T"].to_numpy(dtype=float) - expected
        heating_expected = reference[f"{name}.heating"].to_numpy(dtype=float)
        heating_error = model[f"{name}.heating"].to_numpy(dtype=float) - heating_expected
        mean = float(heating_expected.mean())
        heating: dict[str, float] = {}
        if mean > 1e-6:
            heating = {
                "heating_cv_rmse": float(np.sqrt(np.mean(heating_error**2)) / mean),
                "heating_nmbe": float(heating_error.mean() / mean),
                "heating_deviation": float(heating_error.sum() / heating_expected.sum()),
            }
        rows.append(
            ZoneFidelity(
                zone=name,
                rmse=float(np.sqrt(np.mean(error**2))),
                bias=float(error.mean()),
                max_abs=float(np.max(np.abs(error))),
                mean_reference=float(expected.mean()),
                **heating,
            )
        )
    return rows


def load_kpis(case_id: str, variant: str, library: str) -> RunKpis | None:
    path = kpis_path(case_id, variant, library)
    return RunKpis.model_validate_json(path.read_text()) if path.exists() else None


def load_series(case_id: str, variant: str, library: str) -> pd.DataFrame | None:
    path = series_path(case_id, variant, library)
    return pd.read_csv(path, index_col="time") if path.exists() else None
