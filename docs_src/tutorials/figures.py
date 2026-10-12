"""Figures of the simulated tutorials: zone temperatures, heat and electrical power, energy.

    uv run python docs_src/tutorials/figures.py <tutorial> <results directory>

Each figure is written to docs/tutorials/figures/<name>.html as an HTML fragment that loads
plotly.js from its CDN, and included in the tutorial pages. The results are the .mat files of
OpenModelica (see `simulate` in the tutorials); every signal is looked up by pattern so that the
same code reads the models of IDEAS and Buildings.
"""

import argparse
import re
from collections.abc import Sequence
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from buildingspy.io.outputfile import Reader
from plotly.subplots import make_subplots

FIGURES = Path(__file__).parents[2].joinpath("docs", "tutorials", "figures")
HOUR, DAY = 3600.0, 86400.0
# categorical slots, fixed order (validated reference palette); neutral ink for references
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948")
NEUTRAL, MUTED = "#8a8985", "#b8b7b1"
MONTHS = ("Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec")


@dataclass(frozen=True)
class Series:
    time: np.ndarray  # [s]
    values: np.ndarray


class Result:
    """A simulation result of trano with signals looked up by pattern."""

    def __init__(self, mat: Path) -> None:
        self.reader = Reader(str(mat), "dymola")
        self.names: list[str] = self.reader.varNames()

    def find(self, pattern: str) -> list[str]:
        return sorted(name for name in self.names if re.search(pattern, name))

    def series(self, name: str, step: float = HOUR, scale: float = 1.0, offset: float = 0.0) -> Series:
        """The signal averaged over steps of `step` seconds (energy-preserving for powers)."""
        t, y = self.reader.values(name)
        grid = np.arange(t[0], t[-1] + step / 2, step / 10)
        fine = np.interp(grid, t, y) * scale + offset
        bins = int(len(grid) // 10)
        means = fine[: bins * 10].reshape(bins, 10).mean(axis=1)
        return Series(time=grid[: bins * 10 : 10] + step / 2, values=means)

    def zones(self) -> dict[str, str]:
        """Zone temperature signals of the data bus, by zone name."""
        zones: dict[str, str] = {}
        for name in self.find(r"\.dataBus\.TZon\w+$"):
            zone = name.rsplit("TZon", 1)[1].lower()
            zones.setdefault(zone, name)
        return zones


def _days(time: np.ndarray) -> np.ndarray:
    return time / DAY


def _labels(zones: Sequence[str]) -> list[str]:
    """Zone names for the legend: living_001 -> Living; space_001, space_002 -> Space 1, Space 2."""
    split = [re.fullmatch(r"(.*?)(?:_0*(\d+))?", zone).groups() for zone in zones]  # type: ignore[union-attr]
    bases = [base for base, _ in split]
    return [
        base.replace("_", " ").capitalize() + (f" {number}" if number and bases.count(base) > 1 else "")
        for base, number in split
    ]


def _layout(figure: go.Figure, title: str, height: int) -> go.Figure:
    figure.update_layout(
        title={"text": title, "x": 0.0, "xanchor": "left", "font": {"size": 15}},
        height=height,
        margin={"l": 60, "r": 190, "t": 70, "b": 40},
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        font={"family": "Roboto, Helvetica, Arial, sans-serif", "size": 12, "color": NEUTRAL},
        hovermode="x unified",
    )
    # a legend beside every panel (legend, legend2, ...) at the top of its domain
    for index, axis in enumerate(sorted(key for key in figure.layout if key.startswith("yaxis"))):
        top = figure.layout[axis].domain[1] if figure.layout[axis].domain else 1.0
        figure.layout["legend" if index == 0 else f"legend{index + 1}"] = {
            "x": 1.02, "xanchor": "left", "y": top, "yanchor": "top", "bgcolor": "rgba(0,0,0,0)",
            "font": {"size": 11}, "tracegroupgap": 2,
        }  # fmt: skip
    figure.update_xaxes(showgrid=False, linecolor=MUTED, ticks="outside", tickcolor=MUTED)
    figure.update_yaxes(gridcolor="rgba(138,137,133,0.25)", zeroline=False, linecolor=MUTED)
    return figure


def write(figure: go.Figure, name: str) -> Path:
    FIGURES.mkdir(parents=True, exist_ok=True)
    path = FIGURES.joinpath(f"{name}.html")
    html = figure.to_html(
        include_plotlyjs="cdn", full_html=False, config={"displaylogo": False, "responsive": True}, div_id=name
    )
    path.write_text(html)
    return path


def temperatures_and_power(
    result: Result,
    title: str,
    power: Sequence[tuple[str, str, float]],
    set_points: dict[str, float] | None = None,
    electricity: Sequence[tuple[str, str, float]] = (),
) -> go.Figure:
    """Zone and outdoor temperatures, then the heat delivered, then (optionally) electricity and PV.

    `power` and `electricity` are (pattern, label, scale) of signals in W, plotted in kW.
    """
    rows = 3 if electricity else 2
    titles = ["Temperature [degC]", "Heat [kW]", "Electricity [kW]"][:rows]
    figure = make_subplots(rows=rows, cols=1, shared_xaxes=True, vertical_spacing=0.06, subplot_titles=titles)
    zones = result.zones()
    for index, (label, name) in enumerate(zip(_labels(list(zones)), zones.values(), strict=True)):
        series = result.series(name, offset=-273.15)
        color = SERIES[index % len(SERIES)]
        figure.add_scatter(
            x=_days(series.time), y=series.values, name=label, line={"color": color, "width": 2},
            legend="legend", hovertemplate="%{y:.1f} degC", row=1, col=1,
        )  # fmt: skip
    outdoor = result.find(r"^building\.envelope1\.sim\.Te$") or result.find(r"weaBus\.TDryBul$")
    if outdoor:
        series = result.series(outdoor[0], offset=-273.15)
        figure.add_scatter(
            x=_days(series.time), y=series.values, name="Outdoor", line={"color": NEUTRAL, "width": 1.5, "dash": "dot"},
            legend="legend", hovertemplate="%{y:.1f} degC", row=1, col=1,
        )  # fmt: skip
    for label, value in (set_points or {}).items():
        figure.add_hline(y=value, line={"color": MUTED, "width": 1, "dash": "dash"}, row=1, col=1)
        figure.add_annotation(
            x=1.0, xref="paper", y=value, yref="y", text=label, showarrow=False, xanchor="right", yanchor="bottom",
            font={"size": 10, "color": NEUTRAL},
        )  # fmt: skip
    for row, signals in ((2, power), (3, electricity)):
        for index, (pattern, label, scale) in enumerate(signals):
            names = result.find(pattern)
            if not names:
                continue
            values = sum(result.series(name, scale=scale / 1000).values for name in names)
            time = result.series(names[0]).time
            color = SERIES[index % len(SERIES)]
            figure.add_scatter(
                x=_days(time), y=values, name=label, line={"color": color, "width": 2}, hovertemplate="%{y:.2f} kW",
                legend=f"legend{row}", row=row, col=1,
            )  # fmt: skip
    figure.update_xaxes(title_text="Day of the year", row=rows, col=1)
    return _layout(figure, title, 300 * rows)


def water_temperatures(
    result: Result,
    title: str,
    temperatures: Sequence[tuple[str, str]],
    openings: Sequence[tuple[str, str]],
    set_point: tuple[str, float],
) -> go.Figure:
    """Water temperatures against a set point, then the openings of the valves that hold it.

    `temperatures` and `openings` are (pattern, label); the temperatures are in K, plotted in degC.
    """
    figure = make_subplots(
        rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.06, subplot_titles=["Water [degC]", "Valve opening [1]"]
    )
    for row, signals, offset, unit in ((1, temperatures, -273.15, "degC"), (2, openings, 0.0, "")):
        index = 0
        for pattern, label in signals:
            for number, name in enumerate(result.find(pattern), start=1):
                series = result.series(name, offset=offset)
                figure.add_scatter(
                    x=_days(series.time), y=series.values, name=label.format(number),
                    line={"color": SERIES[index % len(SERIES)], "width": 2}, legend="legend" if row == 1 else "legend2",
                    hovertemplate=f"%{{y:.2f}} {unit}", row=row, col=1,
                )  # fmt: skip
                index += 1
    label, value = set_point
    figure.add_hline(y=value, line={"color": MUTED, "width": 1, "dash": "dash"}, row=1, col=1)
    figure.add_annotation(
        x=1.0, xref="paper", y=value, yref="y", text=label, showarrow=False, xanchor="right", yanchor="bottom",
        font={"size": 10, "color": NEUTRAL},
    )  # fmt: skip
    figure.update_xaxes(title_text="Day of the year", row=2, col=1)
    return _layout(figure, title, 600)


def monthly_energy(results: dict[str, Result], signals: Sequence[tuple[str, str, float]], title: str) -> go.Figure:
    """Monthly energy [kWh] of signals in W: one group of bars per month, one bar per case and signal."""
    figure = go.Figure()
    edges = np.linspace(0, 365 * DAY, 13)
    bars = [(case, result, signal) for case, result in results.items() for signal in signals]
    for index, (case, result, (pattern, label, scale)) in enumerate(bars):
        names = result.find(pattern)
        if not names:
            continue
        grid = np.arange(0, min(result.reader.values(names[0])[0][-1], 365 * DAY), 600.0)
        power = sum(np.interp(grid, *result.reader.values(name)) for name in names) * scale
        energy = [float(power[(grid >= a) & (grid < b)].sum() * 600 / 3.6e6) for a, b in pairwise(edges)]
        figure.add_bar(
            x=list(MONTHS), y=energy, name=label if len(results) == 1 else f"{label}, {case}",
            marker={"color": SERIES[index], "cornerradius": 4}, hovertemplate="%{y:.0f} kWh",
        )  # fmt: skip
    figure.update_layout(barmode="group", bargap=0.25, bargroupgap=0.08, yaxis_title="kWh")
    return _layout(figure, title, 380)


def comparison_bars(values: dict[str, dict[str, float]], title: str, unit: str) -> go.Figure:
    """Grouped bars: one group per quantity, one bar per library (or case)."""
    figure = go.Figure()
    quantities = list(next(iter(values.values())).keys())
    for index, (name, by_quantity) in enumerate(values.items()):
        figure.add_bar(
            x=quantities, y=[by_quantity[q] for q in quantities], name=name,
            text=[f"{by_quantity[q]:.0f}" for q in quantities], textposition="outside",
            marker={"color": SERIES[index], "cornerradius": 4}, hovertemplate=f"%{{y:.0f}} {unit}",
        )  # fmt: skip
    figure.update_layout(barmode="group", bargap=0.3, bargroupgap=0.08, yaxis_title=unit, hovermode="closest")
    return _layout(figure, title, 380)


# --------------------------------------------------------------------------------------------------
RADIATORS = (r"emission1\.radiator_\w+\.Q_flow$", "Radiators", -1.0)


# energy counters of the data bus [kWh], summed over the elements that publish them
BALANCE = {
    "Heat from the generator": r"^building\.bus1\.dataBus\.heat_energy\w+$",
    "Gas": r"^building\.bus1\.dataBus\.fuel_energy\w+$",
    "Electricity": r"^building\.bus1\.dataBus\.electric_energy\w+$",
    "PV production": r"^building\.bus1\.dataBus\.pv_energy\w+$",
}


def annual_balance(result: Result) -> dict[str, float]:
    """Final value of the energy counters of the data bus [kWh]."""
    return {
        label: float(sum(result.reader.values(name)[1][-1] for name in result.find(pattern)))
        for label, pattern in BALANCE.items()
    }


def first_model(results: Path) -> list[Path]:
    result = Result(next(results.glob("first_model*.mat")))
    figure = temperatures_and_power(
        result,
        "Single-zone house, two January weeks in Uccle (ideal heating and cooling, Buildings)",
        power=[(r"emission1\.\w+\.(QHea_flow|heaPorHea\.Q_flow|QHeating)$", "Heating", 1.0)],
        set_points={"21 degC by day": 21.0, "18 degC at night": 18.0},
    )
    return [write(figure, "first_model")]


def hydronic_heating(results: Path) -> list[Path]:
    result = Result(next(results.glob("three_zones_hydronic_heating*.mat")))
    figure = temperatures_and_power(
        result,
        "Three zones heated by a gas boiler and radiators, two January weeks (IDEAS)",
        power=[RADIATORS, (r"production1\.boiler_001\.QFue\.y$", "Boiler, gas", 1.0)],
        set_points={"21 degC set point": 21.0},
    )  # fmt: skip
    return [write(figure, "three_zones_hydronic_heating")]


def hydronic_mixing(results: Path) -> list[Path]:
    result = Result(next(results.glob("three_zones_hydronic_mixing*.mat")))
    rooms = temperatures_and_power(
        result,
        "Three zones heated through mixing circuits, two January weeks (IDEAS)",
        power=[RADIATORS, (r"production1\.boiler_001\.QFue\.y$", "Boiler, gas", 1.0)],
        set_points={"21 degC set point": 21.0},
    )  # fmt: skip
    water = water_temperatures(
        result,
        "The boiler tank at up to 95 degC, the radiators held at 70 degC by the mixing valves (IDEAS)",
        temperatures=[(r"^building\.bus1\.dataBus\.TStoTopBoiler_001$", "Top of the boiler tank"),
                      (r"distribution1\.temperature_sensor_\d+\.T$", "To the radiators, circuit {}")],
        openings=[(r"distribution1\.three_way_valve_\d+\.y$", "Three-way valve {}")],
        set_point=("70 degC set point of the valves", 70.0),
    )  # fmt: skip
    return [write(rooms, "three_zones_hydronic_mixing"), write(water, "three_zones_hydronic_mixing_water")]


def photovoltaics(results: Path) -> list[Path]:
    result = Result(next(results.glob("multizones_with_pv*.mat")))
    figure = temperatures_and_power(
        result,
        "Three zones, an air-water heat pump on radiators and 7 kWp of PV (Buildings)",
        power=[RADIATORS, (r"production1\.boiler_001\.QHea\.y$", "Heat pump, heat", 1.0)],
        electricity=[(r"production1\.boiler_001\.PEle\.y$", "Heat pump", 1.0),
                     (r"\.pv_\d+\.PPv\.y$", "PV production", 1.0)],
        set_points={"21 degC set point": 21.0},
    )  # fmt: skip
    return [write(figure, "multizones_with_pv")]


def residential(results: Path) -> list[Path]:
    """Figures of the residential cases: a January week of each case, the year, IDEAS against Buildings.

    `results` holds week/<case>_Buildings/results/*.mat (7 days) and year/<case>_<library>/results/*.mat
    (365 days), with case before or after; the yearly figures are drawn for the runs found.
    """
    paths = []
    week = {
        case: Result(next(results.joinpath("week", f"{case}_Buildings", "results").glob("*.mat")))
        for case in ("before", "after")
    }
    paths.append(write(temperatures_and_power(
        week["before"], "Before renovation: a January week (Buildings)",
        power=[RADIATORS, (r"production1\.generator_001\.QFue\.y$", "Boiler, gas", 1.0)],
        set_points={"21 degC": 21.0, "18 degC": 18.0},
    ), "residential_before_week"))  # fmt: skip
    paths.append(write(temperatures_and_power(
        week["after"], "After renovation: the same week (Buildings)",
        power=[RADIATORS, (r"production1\.generator_001\.QHea\.y$", "Heat pump, heat", 1.0)],
        electricity=[(r"production1\.generator_001\.PEle\.y$", "Heat pump", 1.0),
                     (r"ventilation1\.ahu_001\.PFan\.y$", "Ventilation fans", 1.0),
                     (r"\.pv_001\.PPv\.y$", "PV production", 1.0)],
        set_points={"21 degC": 21.0, "18 degC": 18.0},
    ), "residential_after_week"))  # fmt: skip
    year_dir = results.joinpath("year")
    year = {
        f"{case}, {library}": Result(next(year_dir.joinpath(f"{case}_{library}", "results").glob("*.mat")))
        for case in ("before", "after")
        for library in ("Buildings", "IDEAS")
        if any(year_dir.joinpath(f"{case}_{library}", "results").glob("*.mat"))
    }
    if year:
        paths.append(write(comparison_bars(
            {name[0].upper() + name[1:]: annual_balance(result) for name, result in year.items()},
            "A year in Uccle: energy of the house before and after renovation, IDEAS against Buildings",
            "kWh",
        ), "residential_annual_balance"))  # fmt: skip
    if {"before, Buildings", "after, Buildings"} <= year.keys():
        paths.append(write(monthly_energy(
            {"before": year["before, Buildings"], "after": year["after, Buildings"]},
            [(r"emission1\.radiator_\w+\.Q_flow$", "Heat from the radiators", -1.0)],
            "Monthly heat delivered by the radiators, before and after renovation (Buildings)",
        ), "residential_monthly_heat"))  # fmt: skip
        paths.append(write(monthly_energy(
            {"after": year["after, Buildings"]},
            [(r"production1\.generator_001\.PEle\.y$", "Heat pump", 1.0),
             (r"ventilation1\.ahu_001\.PFan\.y$", "Ventilation fans", 1.0),
             (r"\.pv_001\.PPv\.y$", "PV production", 1.0)],
            "After renovation: monthly electricity of the heat pump and the fans against the PV production (Buildings)",
        ), "residential_monthly_electricity"))  # fmt: skip
    return paths


TUTORIALS = {
    "first_model": first_model,
    "hydronic_heating": hydronic_heating,
    "hydronic_mixing": hydronic_mixing,
    "photovoltaics": photovoltaics,
    "residential": residential,
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("tutorial", choices=TUTORIALS)
    parser.add_argument("results", type=Path, help="Directory of the simulation results")
    arguments = parser.parse_args()
    for path in TUTORIALS[arguments.tutorial](arguments.results):
        print(f"written {path}")


if __name__ == "__main__":
    main()
