"""The water loop of the house before renovation (gas boiler, three-way valve, six radiators) in both libraries.

Simulates the base case of ``examples/residential_before_renovation.yaml`` with Buildings and with FastModel in
OpenModelica (Docker, ``TRANO_MODELICA_LIBRARIES`` set as for the validation), keeping the variables of the loop, and
answers three questions:

* **restart**: what the boiler, its water and the valves do when the set points step up in the morning (day 8, 05:30
  to 08:00, every 15 minutes);
* **balance**: whether each loop closes its energy balance (fuel x efficiency - jacket loss + pump = heat of the
  radiators + water stored), from the exact energy meters, and how far the integral of the *sampled* fuel power is
  from the meter; over days 8 to 10 for ``restart``, over the study period, its first day (the warm-up from the
  initial temperatures) and the rest for ``month``;
* **components** (FastModel): the energy balance of each hydronic component, enthalpy in minus out plus heat.

    python -m analysis.fast_model.scripts.water_loop restart            # days 7 to 10, both libraries
    python -m analysis.fast_model.scripts.water_loop month              # the 28 days of the study, both libraries
    python -m analysis.fast_model.scripts.water_loop restart --tag tau0.5 --library FastModel \
        --patch 'tauHyd(unit="s") = 5 ' 'tauHyd(unit="s") = 0.5 '

The runs are cached in ``.cache/analysis/water_loop/<mode>-<tag>/<library>`` by the hash of the model, as the
validation caches its runs. Writes ``water_loop_<mode>[_<tag>].json`` and, for ``restart``, the restart table and
the component balances as CSV.
"""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from analysis.fast_model.scripts._common import CACHE, KELVIN, configure_logging, logger, write_csv, write_json
from trano.utils.utils import is_success
from trano_simulate.simulate import SimulationOptions, simulate
from validation.bestest.harness import model_hash
from validation.fast_model.cases import BASE, CASES
from validation.fast_model.harness import BUILDINGS, FAST_MODEL, network_for, variant_yaml

if TYPE_CHECKING:
    from buildingspy.io.outputfile import Reader  # type: ignore[import-untyped]

CASE = CASES["residential_before"]
ROOMS = ("bathroom", "bedroom_1", "bedroom_2", "bedroom_3", "kitchen", "living")
DAY, HOUR = 86400.0, 3600.0
CP_WATER = 4184.0
ETA = 0.85  # constant efficiency of the boiler of the YAML, on the lower heating value
LOOP = r".*(three_way_valve_001|split_valve_001|generator_001|pump_001|temperature_sensor_001|radiator_|valve_).*"
# OpenModelica reads the filter from a Modelica string: its backslashes are doubled, as in the study harness
METERS = (
    r".*(generator_001\\.(EFue|EHea|QFue|T|y|UA|TAmb|C)|generator_001\\.boi\\.(QFue_flow|QWat_flow|UAOve\\.Q_flow|T|y)"
    r"|pump_001\\.pumRad\\.P|radiator_[a-z0-9_]+\\.heatPort(Con|Rad)\\.Q_flow"
    r"|radiator_[a-z0-9_]+\\.(T\\[[0-9]\\]|C|nEle)).*"
)


@dataclass(frozen=True)
class Mode:
    first_day: int
    days: int
    interval: float  # [s] output interval
    variable_filter: str
    windows: dict[str, tuple[float, float]]  # [s] the periods the balances are drawn over; the first is the main one


MODES = {
    "restart": Mode(7, 3, 300.0, LOOP, {"days 8 to 10": (8 * DAY, 10 * DAY)}),
    "month": Mode(
        CASE.start_day,
        CASE.days,
        300.0,
        METERS,
        {
            "the study period": (CASE.start_time, CASE.end_time),
            "first day (warm-up from the initial temperatures)": (CASE.start_time, CASE.start_time + DAY),
            "after the first day": (CASE.start_time + DAY, CASE.end_time),
        },
    ),
}


# ---------------------------------------------------------------------------------------------------------- running
def run(library: str, mode: Mode, directory: Path, patches: list[tuple[str, str]]) -> Path:
    """Simulate the base case unless the cache holds a run of the same model; return the result file."""
    directory.mkdir(parents=True, exist_ok=True)
    network = network_for(variant_yaml(CASE, BASE, directory), library, directory)
    model = network.model(include_container=library == BUILDINGS)
    for old, new in patches:
        if old not in model:
            raise ValueError(f"{old!r} is not in the {library} model")
        model = model.replace(old, new)
    start = mode.first_day * DAY
    options = SimulationOptions(
        start_time=start,
        end_time=start + mode.days * DAY,
        tolerance=1e-6,
        number_of_intervals=int(mode.days * DAY / mode.interval),
        variable_filter=mode.variable_filter,
    )
    expected = model_hash(model, library, options)
    result = directory / "results" / f"{network.name}.building_res.mat"
    hash_file = directory / "model.hash"
    if result.exists() and hash_file.exists() and hash_file.read_text() == expected:
        logger.info("%s read from the cache", directory)
        return result
    directory.joinpath("model.mo").write_text(model)
    name = re.sub(r"[^A-Za-z0-9_.-]", "-", f"analysis-water-loop-{directory.parent.name}-{library}")
    outcome = simulate(directory, network, options=options, container_name=name, model=model)
    output = outcome.output.decode(errors="replace") if isinstance(outcome.output, bytes) else str(outcome.output)
    directory.joinpath("omc.log").write_text(output)
    if not is_success(outcome, options=options):
        raise RuntimeError(f"{library} did not simulate; see {directory / 'omc.log'}")
    hash_file.write_text(expected)
    return result


# --------------------------------------------------------------------------------------------------------- reading
class Series:
    """The variables of a result file on a regular grid of a window."""

    def __init__(self, path: Path, window: tuple[float, float], step: float = 60.0) -> None:
        from buildingspy.io.outputfile import Reader

        self.reader: Reader = Reader(str(path), "dymola")
        self.names = set(self.reader.varNames())
        self.grid = np.arange(window[0], window[1] + step / 2, step)

    def __call__(self, name: str) -> np.ndarray:
        time, values = self.reader.values(name)
        return np.asarray(np.interp(self.grid, time, values))

    def at(self, name: str, time: float) -> float:
        times, values = self.reader.values(name)
        return float(np.interp(time, times, values))

    def integral(self, name: str) -> float:
        """[J] over the window, of a signal resampled every minute."""
        return float(np.trapezoid(self(name), self.grid))

    def sampled_integral(self, name: str) -> float:
        """[J] over the window, of the stored samples only: what an hourly mean of the output points gives."""
        time, values = self.reader.values(name)
        inside = (time >= self.grid[0]) & (time <= self.grid[-1])
        return float(np.trapezoid(values[inside], time[inside]))

    def increment(self, name: str) -> float:
        return self.at(name, self.grid[-1]) - self.at(name, self.grid[0])


def _prefixes(library: str) -> dict[str, str]:
    if library == BUILDINGS:
        return {
            "generator": "building.production1.generator_001.",
            "boiler": "building.production1.generator_001.boi.",
            "emission": "building.emission1.",
            "distribution": "building.distribution1.",
        }
    return {
        "generator": "mpc.generator_001.",
        "boiler": "mpc.generator_001.",
        "emission": "mpc.",
        "distribution": "mpc.",
    }


def radiators_heat(series: Series, library: str) -> float:
    """[kWh] the heat the six radiators give to their rooms over the window."""
    emission = _prefixes(library)["emission"]
    return (
        -sum(
            series.integral(f"{emission}radiator_{room}_001.heatPort{port}.Q_flow")
            for room in ROOMS
            for port in ("Con", "Rad")
        )
        / 3.6e6
    )


def fast_model_stored(series: Series) -> float:
    """[kWh] the change of the heat stored in the water and metal of the radiators and of the boiler."""
    stored = 0.0
    start, end = series.grid[0], series.grid[-1]
    for room in ROOMS:
        component = f"mpc.radiator_{room}_001"
        capacity, elements = series.at(f"{component}.C", start), int(series.at(f"{component}.nEle", start))
        stored += (
            sum(
                series.at(f"{component}.T[{i}]", end) - series.at(f"{component}.T[{i}]", start)
                for i in range(1, elements + 1)
            )
            * capacity
            / elements
        )
    stored += series.at("mpc.generator_001.C", start) * (
        series.at("mpc.generator_001.T", end) - series.at("mpc.generator_001.T", start)
    )
    return stored / 3.6e6


# ------------------------------------------------------------------------------------------------------- analyses
def balance(series: Series, library: str) -> dict[str, float | None]:
    """The energy balance of the water loop over the window, from the energy meters, in kWh."""
    generator, boiler = _prefixes(library)["generator"], _prefixes(library)["boiler"]
    fuel = series.increment(f"{generator}EFue.y" if library == BUILDINGS else f"{generator}EFue")
    sampled_fuel = series.sampled_integral(f"{boiler}QFue_flow" if library == BUILDINGS else f"{boiler}QFue") / 3.6e6
    if library == BUILDINGS:
        # QWat_flow = eta*QFue + UAOve.Q_flow goes into the water, and UAOve conducts out of it once more (Buildings 13)
        heat_meter = series.increment(f"{generator}EHea.y")
        jacket = series.integral(f"{boiler}UAOve.Q_flow") / 3.6e6
        pump_name = next(name for name in series.names if name.endswith("pump_001.pumRad.P"))
        pump = series.integral(pump_name) / 3.6e6
        water_in = heat_meter + jacket + pump
        jacket_loss = ETA * fuel - heat_meter - jacket
        stored = None
    else:
        ua, ambient = series.at(f"{boiler}UA", series.grid[0]), series.at(f"{boiler}TAmb", series.grid[0])
        jacket_loss = ua * (series.integral(f"{boiler}T") - ambient * (series.grid[-1] - series.grid[0])) / 3.6e6
        pump = 0.0
        water_in = ETA * fuel - jacket_loss
        stored = fast_model_stored(series)
    radiators = radiators_heat(series, library)
    return {
        "fuel_meter_kwh": fuel,
        "fuel_from_samples_kwh": sampled_fuel,
        "fuel_samples_error": sampled_fuel / fuel - 1 if fuel else None,
        "jacket_loss_kwh": jacket_loss,
        "pump_heat_kwh": pump,
        "water_in_kwh": water_in,
        "radiators_kwh": radiators,
        "stored_kwh": stored,
        "residual_kwh": water_in - radiators - (stored or 0.0),
        "fuel_per_heat": fuel / radiators,
        "mean_boiler_temperature_degc": float(np.mean(series(f"{boiler}T"))) - KELVIN,
    }


def restart_rows(series: dict[str, Series], day: int = 8) -> list[dict[str, Any]]:
    """Boiler, water and valves of both libraries from 05:30 to 08:00 on a morning, every 15 minutes."""
    rows = []
    for time in day * DAY + np.arange(5.5 * HOUR, 8 * HOUR + 1, 900.0):
        row: dict[str, Any] = {"day": day, "time_h": round((time - day * DAY) / HOUR, 2)}
        for library, data in series.items():
            boiler, emission, distribution = (_prefixes(library)[key] for key in ("boiler", "emission", "distribution"))
            fuel = f"{boiler}QFue_flow" if library == BUILDINGS else f"{boiler}QFue"
            row |= {
                f"{library}_firing": data.at(f"{boiler}y", time),
                f"{library}_water_degc": data.at(f"{boiler}T", time) - KELVIN,
                f"{library}_fuel_kw": data.at(fuel, time) / 1000,
                f"{library}_largest_valve_opening": max(
                    data.at(f"{emission}valve_{room}_001.y", time) for room in ROOMS
                ),
                f"{library}_three_way_valve": data.at(f"{distribution}three_way_valve_001.y", time),
            }
        rows.append(row)
    return rows


def lowest_boiler_temperature(series: Series, library: str) -> float:
    return float(np.min(series(f"{_prefixes(library)['boiler']}T"))) - KELVIN


def component_rows(series: Series, references: tuple[float, ...] = (300.0, 340.0)) -> list[dict[str, Any]]:
    """FastModel: enthalpy into each hydronic component minus enthalpy out, plus its heat, mean over the window [W].

    The ports carry the temperature of the water entering, so a component that conserves energy and holds no more
    heat at the end than at the start sums to zero whatever the reference temperature of the enthalpy; the samples
    five minutes apart do not resolve the pressure transients of the inlet nodes, which shows as a dependence on the
    reference."""
    port = re.compile(r"mpc\.([a-z_0-9]+)\.port_[a-z0-9]+\[\d+\]\.m_flow$")
    components = sorted({match.group(1) for name in series.names if (match := port.match(name))})
    duration = series.grid[-1] - series.grid[0]
    rows = []
    for component in components:
        ports = sorted(
            name[: -len(".m_flow")]
            for name in series.names
            if (match := port.match(name)) and match.group(1) == component
        )
        heat = np.zeros_like(series.grid)
        if component == "generator_001":
            heat = series("mpc.generator_001.QWat")
        elif component.startswith("radiator_"):
            heat = series(f"mpc.{component}.heatPortCon.Q_flow") + series(f"mpc.{component}.heatPortRad.Q_flow")
        row: dict[str, Any] = {"component": component, "ports": len(ports)}
        for reference in references:
            flux = sum(
                (series(f"{name}.m_flow") * CP_WATER * (series(f"{name}.T") - reference) for name in ports), heat
            )
            row[f"balance_w_tref_{reference:g}K"] = float(np.trapezoid(flux, series.grid)) / duration
        rows.append(row)
    return rows


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("mode", choices=sorted(MODES))
    parser.add_argument("--library", choices=(BUILDINGS, FAST_MODEL), action="append", help="both by default")
    parser.add_argument("--tag", default="", help="names a variant of the runs (with --patch)")
    parser.add_argument(
        "--patch",
        nargs=2,
        action="append",
        default=[],
        metavar=("OLD", "NEW"),
        help="replace OLD by NEW in the generated model text before simulating",
    )
    arguments = parser.parse_args(argv)
    configure_logging()
    mode = MODES[arguments.mode]
    libraries = arguments.library or [BUILDINGS, FAST_MODEL]
    label = arguments.mode + (f"-{arguments.tag}" if arguments.tag else "")
    results = {
        library: run(library, mode, CACHE / "water_loop" / label / library, arguments.patch) for library in libraries
    }
    windows = {
        name: {library: Series(path, window) for library, path in results.items()}
        for name, window in mode.windows.items()
    }
    series = next(iter(windows.values()))
    suffix = arguments.mode + (f"_{arguments.tag}" if arguments.tag else "")
    content: dict[str, Any] = {
        "case": CASE.id,
        "patches": arguments.patch,
        "balances": {
            name: {"days": [window[0] / DAY, window[1] / DAY]}
            | {library: balance(data, library) for library, data in windows[name].items()}
            for name, window in mode.windows.items()
        },
        "lowest_boiler_temperature_degc": {
            library: lowest_boiler_temperature(data, library) for library, data in series.items()
        },
    }
    if arguments.mode == "restart":
        if len(series) == 2:
            write_csv(
                f"water_loop_restart_table{'_' + arguments.tag if arguments.tag else ''}.csv", restart_rows(series)
            )
        if FAST_MODEL in series:
            write_csv(f"water_loop_components_{suffix}.csv", component_rows(series[FAST_MODEL]))
    write_json(f"water_loop_{suffix}.json", content)


if __name__ == "__main__":
    main()
