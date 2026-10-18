"""Heating KPIs of a building over the simulated period, read from an OpenModelica result file.

Phase 1 supports the ``ideal_heating_cooling`` emission only: the heating power it delivers is a variable
of the result, whatever the zone model of the library.
"""

import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import numpy.typing as npt
from buildingspy.io.outputfile import Reader  # type: ignore[import-untyped]
from pydantic import BaseModel, Field

from trano.elements.space import Space
from trano.elements.system import Emission, IdealHeatingCooling
from trano.topology import Network

JOULES_PER_KWH = 3.6e6
SECONDS_PER_HOUR = 3600.0
SETPOINT_TOLERANCE_K = 0.5


class IdealHeatingVariables(BaseModel):
    """The variables of an ideal heating component, relative to the component."""

    heating_power: str = Field(description="Heat flow delivered to the zone [W].")
    air_temperature: str = Field(description="Zone air temperature [K].")
    heating_setpoint: str = Field(description="Heating set point [K].")


# The ideal heating is Trano's own model (Trano.HeatTransfer.IdealHeatingSystem.IdealHeatingCooling) for every
# library, so its variables are the same. Found by simulating tests/tutorials/first_model.yaml for one day
# with each library and listing the variables of its ideal heating ``heating_001``:
#     ``Reader(result_path, "openmodelica").varNames("heating_001")``
# which gives, among its parameters, in both result files:
#     building.emission1.heating_001.QHea_flow  (heating power, W)
#     building.emission1.heating_001.TAir.T     (zone air temperature sensor, K)
#     building.emission1.heating_001.heaSch.y[1] (heating set point schedule, K)
# The prefix (``building.emission1.``) depends on how the components are grouped in containers: the
# variables are looked up by the component name and these suffixes.
IDEAL_HEATING_VARIABLES: dict[str, IdealHeatingVariables] = {
    library: IdealHeatingVariables(heating_power="QHea_flow", air_temperature="TAir.T", heating_setpoint="heaSch.y[1]")
    for library in ("Buildings", "IDEAS")
}


class ZoneKpi(BaseModel):
    zone_id: str = Field(description="Modelica name of the space (its YAML id in lower case, ':' as '_').")
    heating_energy_kwh: float
    peak_heating_w: float
    hours_below_setpoint: float | None = Field(
        default=None,
        description=f"Hours with the air more than {SETPOINT_TOLERANCE_K} K below the heating set point; "
        "None when the result file has no set point.",
    )


class BuildingKpi(BaseModel):
    period_start_s: float
    period_end_s: float
    heated_floor_area_m2: float
    heating_energy_kwh: float
    heating_energy_kwh_per_m2: float
    peak_heating_w: float
    zones: list[ZoneKpi]
    source_variables: dict[str, str] = Field(description="KPI name -> Modelica variable read (traceability).")


@dataclass(frozen=True, slots=True)
class _Series:
    time: npt.NDArray[np.float64]
    values: npt.NDArray[np.float64]


def building_kpis(result_path: Path, network: Network) -> BuildingKpi:
    """The heating KPIs of the simulated period of ``result_path``, the result file of ``network``.

    Energy is the time integral (trapezoid on the result time grid) of the heating power, peak its maximum.

    Raises:
        NotImplementedError: when a space is served by another emission than an ideal heating, or when the
            library of the network has no variable mapping.
        ValueError: when no space is heated, or a variable is missing from the result file.
    """
    variables = _ideal_heating_variables(network)
    reader = Reader(str(result_path), "openmodelica")
    names = reader.varNames(".*")
    spaces = sorted((node for node in network.graph.nodes if isinstance(node, Space)), key=lambda space: space.name)
    zones: list[ZoneKpi] = []
    sources: dict[str, str] = {}
    building_power: _Series | None = None
    heated_floor_area = 0.0
    for space in spaces:
        heatings = _ideal_heatings(space)
        if not heatings:
            continue
        power_names = [_find(names, heating, variables.heating_power) for heating in heatings]
        power = _sum([_read(reader, name) for name in power_names])
        sources[f"{space.name}.heating_power"] = " + ".join(power_names)
        zones.append(
            ZoneKpi(
                zone_id=space.name,
                heating_energy_kwh=_energy_kwh(power),
                peak_heating_w=float(np.max(power.values)),
                hours_below_setpoint=_hours_below_setpoint(reader, names, heatings[0], variables, space, sources),
            )
        )
        heated_floor_area += float(space.parameters.floor_area)  # type: ignore[union-attr]
        building_power = power if building_power is None else _sum([building_power, power])
    if building_power is None:
        raise ValueError("kpi: no space of the model is heated by an ideal heating.")
    energy = _energy_kwh(building_power)
    return BuildingKpi(
        period_start_s=float(building_power.time[0]),
        period_end_s=float(building_power.time[-1]),
        heated_floor_area_m2=heated_floor_area,
        heating_energy_kwh=energy,
        heating_energy_kwh_per_m2=energy / heated_floor_area if heated_floor_area > 0 else float("nan"),
        peak_heating_w=float(np.max(building_power.values)),
        zones=zones,
        source_variables=sources,
    )


def _ideal_heating_variables(network: Network) -> IdealHeatingVariables:
    for node in network.graph.nodes:
        if isinstance(node, Emission) and not isinstance(node, IdealHeatingCooling):
            raise NotImplementedError("kpi: only ideal heating in phase 1")
    library = network.library.name
    if library not in IDEAL_HEATING_VARIABLES:
        raise NotImplementedError(
            f"kpi: library {library} is not supported; use one of {list(IDEAL_HEATING_VARIABLES)}"
        )
    return IDEAL_HEATING_VARIABLES[library]


def _ideal_heatings(space: Space) -> list[IdealHeatingCooling]:
    return sorted(
        (emission for emission in space.emissions if isinstance(emission, IdealHeatingCooling)),
        key=lambda emission: str(emission.name),
    )


def _find(names: list[str], heating: IdealHeatingCooling, variable: str) -> str:
    """The full name of ``variable`` of ``heating`` in the result file, wherever its container puts it."""
    pattern = re.compile(rf"(^|\.){re.escape(str(heating.name))}\.{re.escape(variable)}$")
    found = [name for name in names if pattern.search(name)]
    if len(found) != 1:
        raise ValueError(f"kpi: expected one variable {heating.name}.{variable} in the result file, found {found}.")
    return found[0]


def _read(reader: Reader, name: str) -> _Series:
    time, values = reader.values(name)
    return _Series(time=np.asarray(time, dtype=float), values=np.asarray(values, dtype=float))


def _sum(series: list[_Series]) -> _Series:
    """The sum of series of one result file, which share its time grid."""
    return _Series(time=series[0].time, values=np.sum([serie.values for serie in series], axis=0))


def _energy_kwh(power: _Series) -> float:
    return float(np.trapezoid(power.values, power.time)) / JOULES_PER_KWH


def _hours_below_setpoint(  # noqa: PLR0913
    reader: Reader,
    names: list[str],
    heating: IdealHeatingCooling,
    variables: IdealHeatingVariables,
    space: Space,
    sources: dict[str, str],
) -> float | None:
    try:
        temperature_name = _find(names, heating, variables.air_temperature)
        setpoint_name = _find(names, heating, variables.heating_setpoint)
    except ValueError:
        return None
    temperature, setpoint = _read(reader, temperature_name), _read(reader, setpoint_name)
    sources[f"{space.name}.air_temperature"] = temperature_name
    sources[f"{space.name}.heating_setpoint"] = setpoint_name
    below = temperature.values < setpoint.values - SETPOINT_TOLERANCE_K
    # each sample holds until the next one
    durations = np.diff(temperature.time)
    return float(np.sum(durations[below[:-1]])) / SECONDS_PER_HOUR
