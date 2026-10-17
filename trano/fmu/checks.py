"""The three checks a parameter passes before it is called calibratable, run on the FMU with FMPy.

1. **Readback**: the perturbed value set before initialisation is read back unchanged after it,
   and the calculated parameters depending on it moved.
2. **Impact**: the perturbed FMU moves the outputs that matter (zone temperatures, heat pump
   powers, radiator heat flows, supply air temperature, PV power) by more than a threshold.
3. **Equivalence**: the FMU rebuilt from the YAML with the same change gives the same outputs as
   the baseline FMU with the parameter override (the proof that the override is the change of the
   building description).

The metrics are relative: the RMS of the difference over the RMS of the baseline for powers and
flows, the mean absolute difference in kelvin for temperatures (a relative error on an absolute
temperature says nothing).
"""

import math
import shutil
import tempfile
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
from pydantic import BaseModel, ConfigDict, Field

from trano.elements.space import BaseSpace
from trano.elements.solar import Photovoltaic
from trano.elements.system import AirHandlingUnit, Boiler, Emission
from trano.exceptions import FmuError
from trano.fmu.inventory import TOP_INSTANCE
from trano.topology import Network

DAY = 24 * 3600
IMPACT_THRESHOLD = 0.01
"""Relative RMS change of a power or flow output above which a parameter has an impact."""
TEMPERATURE_THRESHOLD = 0.02
"""[K] mean absolute change of a temperature output above which a parameter has an impact."""
EQUIVALENCE_TOLERANCE = 1e-3
"""Relative RMS difference tolerated between the override run and the rebuilt FMU."""
READBACK_TOLERANCE = 1e-9
BUS = f"{TOP_INSTANCE}.bus1.dataBus"


class Window(BaseModel):
    """Simulation horizon: a warm-up then the window the outputs are compared over."""

    model_config = ConfigDict(frozen=True)
    start_time: float = Field(default=0.0, description="[s] start of the simulation (0 is January 1st)")
    warm_up: float = Field(default=7 * DAY, description="[s] discarded before the comparison window")
    duration: float = Field(default=14 * DAY, description="[s] length of the comparison window")
    output_interval: float = Field(default=600.0, description="[s] sampling of the outputs")
    tolerance: float = Field(default=1e-6, description="Relative tolerance of the solver embedded in the FMU")

    @property
    def stop_time(self) -> float:
        return self.start_time + self.warm_up + self.duration

    @property
    def window_start(self) -> float:
        return self.start_time + self.warm_up


class Outputs(BaseModel):
    """The outputs an impact is measured on, with their kind (temperature or power)."""

    model_config = ConfigDict(frozen=True)
    temperatures: tuple[str, ...]
    powers: tuple[str, ...]

    @property
    def names(self) -> list[str]:
        return [*self.temperatures, *self.powers]


def _bus_name(signal: str, element_name: str) -> str:
    """Data bus signals are named ``<signal><Element_name>`` (``heat_flowGenerator_001``)."""
    return f"{BUS}.{signal}{element_name.capitalize()}"


def impact_outputs(network: Network) -> Outputs:
    """The outputs of a network: air temperature of each zone, heat pump thermal and electrical power, heat flow
    of each radiator, supply air temperature of the air handling units, PV power."""
    temperatures, powers = [], []
    for node in network.graph.nodes:
        if node.name is None:
            continue
        if isinstance(node, BaseSpace):
            temperatures.append(_bus_name("TZon", node.name))
        elif isinstance(node, Boiler):
            powers += [_bus_name("heat_flow", node.name), _bus_name("electricity", node.name)]
        elif isinstance(node, Emission):
            powers.append(f"{TOP_INSTANCE}.{node.container_type}1.{node.name}.Q_flow")
        elif isinstance(node, AirHandlingUnit):
            temperatures.append(_bus_name("supply_temperature", node.name))
        elif isinstance(node, Photovoltaic):
            powers.append(_bus_name("pv_power", node.name))
    return Outputs(temperatures=tuple(sorted(temperatures)), powers=tuple(sorted(powers)))


def simulate_fmu(
    fmu: Path,
    outputs: Iterable[str],
    window: Window,
    start_values: Mapping[str, float] | None = None,
) -> pd.DataFrame:
    """Simulate the FMU over the window with FMPy; the outputs sampled over the comparison window, indexed by time."""
    import fmpy  # type: ignore[import-untyped]

    outputs = list(outputs)
    with tempfile.TemporaryDirectory() as unzip_directory:
        # FMPy would extract next to the FMU otherwise; a scratch directory keeps the FMU untouched and
        # simulates it away from where it was built, as another tool would.
        extracted = fmpy.extract(str(fmu), unzipdir=unzip_directory)
        result = fmpy.simulate_fmu(
            extracted,
            start_time=window.start_time,
            stop_time=window.stop_time,
            output_interval=window.output_interval,
            relative_tolerance=window.tolerance,
            start_values=dict(start_values or {}),
            output=outputs,
            fmi_type="CoSimulation",
        )
    frame = pd.DataFrame({name: result[name] for name in ["time", *outputs]}).set_index("time")
    frame = frame[frame.index >= window.window_start]
    if not np.isfinite(frame.to_numpy()).all():
        raise FmuError(f"The simulation of {fmu.name} gave non-finite outputs")
    return frame


def read_back(fmu: Path, start_values: Mapping[str, float], read: Iterable[str]) -> dict[str, float]:
    """Set the start values before initialisation and read ``read`` after it (FMI 2.0 co-simulation)."""
    import fmpy
    from fmpy.fmi2 import FMU2Slave  # type: ignore[import-untyped]

    read = list(read)
    with tempfile.TemporaryDirectory() as unzip_directory:
        extracted = fmpy.extract(str(fmu), unzipdir=unzip_directory)
        description = fmpy.read_model_description(extracted)
        references = {variable.name: variable.valueReference for variable in description.modelVariables}
        slave = FMU2Slave(
            guid=description.guid,
            unzipDirectory=extracted,
            modelIdentifier=description.coSimulation.modelIdentifier,
            instanceName="readback",
        )
        slave.instantiate()
        try:
            slave.setupExperiment(startTime=0.0)
            for name, value in start_values.items():
                slave.setReal([references[name]], [float(value)])
            slave.enterInitializationMode()
            slave.exitInitializationMode()
            values = slave.getReal([references[name] for name in read])
        finally:
            slave.terminate()
            slave.freeInstance()
        shutil.rmtree(unzip_directory, ignore_errors=True)
    return dict(zip(read, values, strict=True))


def relative_rms(baseline: pd.Series, other: pd.Series) -> float:
    """RMS of the difference over the RMS of the baseline; the RMS of the difference when the baseline is zero."""
    difference = float(np.sqrt(np.mean((other.to_numpy() - baseline.to_numpy()) ** 2)))
    scale = float(np.sqrt(np.mean(baseline.to_numpy() ** 2)))
    return difference / scale if scale > 0 else difference


def mean_absolute(baseline: pd.Series, other: pd.Series) -> float:
    return float(np.mean(np.abs(other.to_numpy() - baseline.to_numpy())))


class Impact(BaseModel):
    """The change of every output between two runs."""

    temperatures: dict[str, float] = Field(description="[K] mean absolute change per temperature output")
    powers: dict[str, float] = Field(description="relative RMS change per power or flow output")

    @property
    def significant(self) -> bool:
        return any(value > TEMPERATURE_THRESHOLD for value in self.temperatures.values()) or any(
            value > IMPACT_THRESHOLD for value in self.powers.values()
        )

    @property
    def largest(self) -> tuple[str, float, str]:
        """``(output, value, metric)`` of the largest change, each metric scaled by its threshold."""
        scaled = [(name, value, "K", value / TEMPERATURE_THRESHOLD) for name, value in self.temperatures.items()]
        scaled += [(name, value, "rel_rms", value / IMPACT_THRESHOLD) for name, value in self.powers.items()]
        name, value, metric, _ = max(scaled, key=lambda item: item[3], default=("", 0.0, "", 0.0))
        return name, value, metric

    def summary(self) -> str:
        name, value, metric = self.largest
        return f"{metric}={value:.3g} ({name.rsplit('.', 1)[-1]})" if name else ""


def compare(baseline: pd.DataFrame, other: pd.DataFrame, outputs: Outputs) -> Impact:
    return Impact(
        temperatures={name: mean_absolute(baseline[name], other[name]) for name in outputs.temperatures},
        powers={name: relative_rms(baseline[name], other[name]) for name in outputs.powers},
    )


def max_relative_rms(first: pd.DataFrame, second: pd.DataFrame) -> float:
    """The largest relative RMS difference over the columns; temperatures are compared in kelvin above 0 degC."""
    worst = 0.0
    for column in first.columns:
        base, other = first[column], second[column]
        if base.mean() > 200:  # a temperature in kelvin: compare the deviations from the freezing point
            base, other = base - 273.15, other - 273.15
        worst = max(worst, relative_rms(base, other))
    return worst


def perturb(value: float, factor: float = 1.2, lower: float | None = None, upper: float | None = None) -> float:
    """``value * factor`` kept inside physical bounds (a fraction stays below 1, a flow stays positive)."""
    perturbed = value * factor if value != 0 else 1.0 / factor
    if upper is not None and perturbed > upper:
        perturbed = value / factor
    if lower is not None and perturbed < lower:
        perturbed = value / factor
    if math.isclose(perturbed, value):
        raise FmuError(f"Cannot perturb {value} within [{lower}, {upper}]")
    return perturbed


def as_float(value: Any) -> float:  # noqa: ANN401
    return float(value)
