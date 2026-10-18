"""Benchmark of FastModel on a case: accuracy against Buildings and IDEAS, and speed in OpenModelica and in CasADi.

    python -m validation.mpc compare residential_after --models FastModel --references Buildings IDEAS
    python -m validation.mpc.benchmark residential_after

The comparisons are read from the stored results of ``compare``; the benchmark adds:

* the OpenModelica timings of the Buildings, IDEAS and FastModel runs (translation and simulation);
* the translation of ``building_mpc`` with rumoca, and a CasADi simulation (CVODES) of the whole case with the
  inputs of the OpenModelica run of the runnable FastModel model: the CasADi ODE must reproduce it;
* an optimal control problem of a day (IPOPT) from the middle of the case;
* the figures of the documentation (``docs/validation/figures``).
"""

from __future__ import annotations

import argparse
import json
import re
import time
import warnings
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import plotly.graph_objects as go  # type: ignore[import-untyped]
from plotly.subplots import make_subplots  # type: ignore[import-untyped]
from pydantic import BaseModel, Field

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano_optimization import KELVIN, CasadiModel, OptimalControl, OptimalControlSettings
from trano_optimization.collocation import Collocated, CollocationSettings
from validation.mpc.cases import CASES, ROOT, MPCCase
from validation.mpc.harness import (
    CACHE_ROOT,
    FAST_MODEL,
    FAST_MODEL_MPC,
    FINE_REPLAY_INTERVAL,
    REFERENCE,
    REFERENCES,
    CaseComparison,
    case_directory,
)

if TYPE_CHECKING:
    from buildingspy.io.outputfile import Reader  # type: ignore

FIGURES = ROOT / "docs" / "validation" / "figures"
RESULTS = CACHE_ROOT / "results.json"
OM_TIMINGS = ("timeFrontend", "timeBackend", "timeSimCode", "timeTemplates", "timeCompile", "timeSimulation")
COLORS = {"Buildings": "#2a78d6", "IDEAS": "#e34948", "FastModel": "#1baf7a"}
NEUTRAL, MUTED = "#8a8985", "#b8b7b1"
DAY = 86400


class OpenModelicaRun(BaseModel):
    model: str
    translation: float = Field(description="Front end, back end, code generation and compilation [s]")
    simulation: float = Field(description="Simulation [s]")


class CasadiBenchmark(BaseModel):
    states: int
    inputs: int
    controls: int
    parameters: int
    translation: float = Field(description="rumoca translation and CasADi export [s]")
    simulation: float = Field(description="CVODES simulation of the whole case [s]")
    steps: int = Field(description="Steps of the CasADi simulation (inputs held over each)")
    deviation: dict[str, float] = Field(description="RMSE of each zone, CasADi against OpenModelica [K]")
    ocp_hours: int
    ocp_solve: float = Field(description="IPOPT solve of the optimal control problem [s]")
    ocp_status: str


class Benchmark(BaseModel):
    case: str
    openmodelica: list[OpenModelicaRun]
    casadi: CasadiBenchmark


def _reader(path: Path) -> Reader:
    from buildingspy.io.outputfile import Reader

    return Reader(str(path), "dymola")


def _result(case: MPCCase, variant: str) -> Path:
    return next((case_directory(case, variant) / "results").glob("*_res.mat"))


def _openmodelica_run(case: MPCCase, variant: str, label: str) -> OpenModelicaRun:
    log = (case_directory(case, variant) / "omc.log").read_text()
    timings = {
        name: float(match.group(1)) for name in OM_TIMINGS if (match := re.search(rf"{name} = ([0-9.eE+-]+)", log))
    }
    return OpenModelicaRun(
        model=label,
        translation=sum(value for name, value in timings.items() if name != "timeSimulation"),
        simulation=timings.get("timeSimulation", float("nan")),
    )


def _on_grid(reader: Reader, variable: str, grid: np.ndarray) -> np.ndarray:
    time_, values = reader.values(variable)
    _, last = np.unique(time_[::-1], return_index=True)
    keep = len(time_) - 1 - last
    return np.asarray(np.interp(grid, time_[keep], values[keep]))


def _step_means(reader: Reader, variable: str, grid: np.ndarray) -> np.ndarray:
    """Mean of a signal over each step of ``grid``, its values linear between the grid points."""
    values = _on_grid(reader, variable, grid)
    return np.asarray((values[:-1] + values[1:]) / 2)


def _casadi(case: MPCCase, ocp_hours: int) -> tuple[CasadiBenchmark, dict[str, np.ndarray], np.ndarray]:
    """Translate building_mpc, replay the OpenModelica run of the runnable model through CVODES and solve an OCP."""
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        network = convert_network(case.path.stem, case.path, library=Library.from_configuration(FAST_MODEL_MPC))
        source = network.model(include_container=False)  # flat, as the replayed run of the harness
    started = time.perf_counter()
    model = CasadiModel.from_modelica(source, f"{case.path.stem}.building_mpc")
    translation = time.perf_counter() - started
    description = model.description

    reader = _reader(_result(case, FAST_MODEL))
    step = FINE_REPLAY_INTERVAL
    grid = np.arange(case.start_time, case.end_time + 1, step, dtype=float)
    simulator = Collocated(model, settings=CollocationSettings(time_step=step))
    controls = np.array([_step_means(reader, f"mpc.{name}", grid) for name in simulator.roles.controls])
    disturbances = np.array([_step_means(reader, f"mpc.{name}", grid) for name in simulator.roles.disturbances])
    initial = np.array([_on_grid(reader, f"mpc.{name}", grid[:1])[0] for name in description.state_names])
    started = time.perf_counter()
    states = simulator.simulate(initial, controls, disturbances, start_time=case.start_time)
    simulation = time.perf_counter() - started
    zones = {name: states[description.state_names.index(name)] for name in simulator.roles.comfort}
    deviation = {
        name.removesuffix(".TAir"): float(np.sqrt(np.mean((values - _on_grid(reader, f"mpc.{name}", grid)) ** 2)))
        for name, values in zones.items()
    }

    # a day of optimal control from the middle of the case, with the hourly weather and gains of the run
    ocp = OptimalControl(model, settings=OptimalControlSettings(time_step=3600))
    start = case.start_time + (case.days // 2) * DAY
    hourly = np.arange(start, start + ocp_hours * 3600 + 1, 3600, dtype=float)
    forecast = np.array([_step_means(reader, f"mpc.{name}", hourly) for name in ocp.roles.disturbances])
    hours = (hourly[:-1] - case.start_time) / 3600 % 24
    comfort = np.where((hours >= 7) & (hours < 22), KELVIN + 20, KELVIN + 17)
    begin = int((start - case.start_time) // step)
    result = ocp.solve(
        states[:, begin], forecast, comfort, price=np.full(ocp_hours, 0.30), feed_in_price=np.full(ocp_hours, 0.05),
        start_time=start,
    )  # fmt: skip
    benchmark = CasadiBenchmark(
        states=len(description.states),
        inputs=len(description.inputs),
        controls=len(simulator.roles.controls),
        parameters=len(description.parameters),
        translation=translation,
        simulation=simulation,
        steps=controls.shape[1],
        deviation=deviation,
        ocp_hours=ocp_hours,
        ocp_solve=result.solve_time,
        ocp_status=result.status,
    )
    return benchmark, zones, grid


def _layout(figure: go.Figure, title: str, height: int) -> go.Figure:
    figure.update_layout(
        title={"text": title, "x": 0.0, "xanchor": "left", "font": {"size": 15}},
        height=height,
        margin={"l": 60, "r": 20, "t": 70, "b": 40},
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        font={"family": "Roboto, Helvetica, Arial, sans-serif", "size": 12, "color": NEUTRAL},
        hovermode="x unified",
        legend={"orientation": "h", "y": -0.12, "x": 0},
    )
    figure.update_xaxes(showgrid=False, linecolor=MUTED, ticks="outside", tickcolor=MUTED)
    figure.update_yaxes(gridcolor="rgba(138,137,133,0.25)", zeroline=False, linecolor=MUTED)
    return figure


def _write(figure: go.Figure, name: str) -> Path:
    FIGURES.mkdir(parents=True, exist_ok=True)
    path = FIGURES / f"{name}.html"
    path.write_text(
        figure.to_html(
            include_plotlyjs="cdn", full_html=False, config={"displaylogo": False, "responsive": True}, div_id=name
        )
    )
    return path


def _zone_figure(case: MPCCase, zones: list[str], days: tuple[int, int]) -> go.Figure:
    """Air temperature of some zones over a few days: each library and FastModel replaying it."""
    grid = np.arange(case.start_time + days[0] * DAY, case.start_time + days[1] * DAY + 1, 900, dtype=float)
    figure = make_subplots(
        rows=len(zones), cols=1, shared_xaxes=True, vertical_spacing=0.08,
        subplot_titles=[f"{zone.removesuffix('_001').replace('_', ' ').capitalize()} [degC]" for zone in zones],
    )  # fmt: skip
    variable = {"Buildings": "{zone}.heaPorAir.T", "IDEAS": "{zone}.TAir"}
    for reference in REFERENCES:
        library = _reader(_result(case, reference))
        variant = FAST_MODEL if reference == REFERENCE else f"{FAST_MODEL}-{reference}"
        fast = _reader(_result(case, variant))
        for row, zone in enumerate(zones, start=1):
            first = row == 1
            name = next(n for n in library.varNames() if n.endswith(variable[reference].format(zone=zone)))
            figure.add_scatter(
                x=(grid - case.start_time) / DAY, y=_on_grid(library, name, grid) - KELVIN, name=reference,
                legendgroup=reference, showlegend=first, line={"color": COLORS[reference], "width": 2},
                hovertemplate="%{y:.2f} degC", row=row, col=1,
            )  # fmt: skip
            figure.add_scatter(
                x=(grid - case.start_time) / DAY, y=_on_grid(fast, f"mpc.{zone}.TAir", grid) - KELVIN,
                name=f"FastModel replaying {reference}", legendgroup=f"fast-{reference}", showlegend=first,
                line={"color": COLORS[reference], "width": 1.5, "dash": "dash"}, hovertemplate="%{y:.2f} degC",
                row=row, col=1,
            )  # fmt: skip
    figure.update_xaxes(title_text="Day of January", row=len(zones), col=1)
    return _layout(figure, "Zone air temperature: Buildings, IDEAS and FastModel replaying each", 260 * len(zones))


def _energy_figure(comparisons: list[CaseComparison]) -> go.Figure:
    figure = go.Figure()
    for comparison in comparisons:
        labels = [f"{energy.system} {energy.quantity}" for energy in comparison.energies]
        figure.add_bar(
            x=labels, y=[energy.reference_kwh for energy in comparison.energies], name=comparison.reference,
            marker_color=COLORS[comparison.reference], hovertemplate="%{y:.1f} kWh",
        )  # fmt: skip
        figure.add_bar(
            x=labels, y=[energy.model_kwh for energy in comparison.energies],
            name=f"FastModel replaying {comparison.reference}", marker_color=COLORS[comparison.reference],
            marker_pattern_shape="/", opacity=0.6, hovertemplate="%{y:.1f} kWh",
        )  # fmt: skip
    figure.update_layout(barmode="group", yaxis_title="kWh over 28 days")
    return _layout(figure, "Energy of the systems over the four weeks", 420)


def _speed_figure(benchmark: Benchmark) -> go.Figure:
    runs = benchmark.openmodelica
    figure = go.Figure()
    figure.add_bar(
        y=[run.model for run in runs], x=[run.simulation for run in runs], orientation="h", name="Simulation",
        marker_color=[COLORS.get(run.model.split()[0], NEUTRAL) for run in runs], hovertemplate="%{x:.0f} s",
    )  # fmt: skip
    casadi = benchmark.casadi
    figure.add_bar(
        y=["FastModel, CasADi (CVODES)"], x=[casadi.simulation], orientation="h", name="CasADi",
        marker_color=COLORS["FastModel"], marker_pattern_shape="/", hovertemplate="%{x:.0f} s",
    )  # fmt: skip
    figure.update_layout(showlegend=False, xaxis_title="Simulation time of the 28 days [s]")
    return _layout(figure, "Simulation time", 300)


def _stored(case_id: str) -> list[CaseComparison]:
    results = json.loads(RESULTS.read_text())
    comparisons = [CaseComparison.model_validate(item) for item in results.values()]
    return [item for item in comparisons if item.case == case_id and item.model == FAST_MODEL]


def run(case_id: str, ocp_hours: int = 24) -> Benchmark:
    case = CASES[case_id]
    comparisons = sorted(_stored(case_id), key=lambda item: REFERENCES.index(item.reference))
    if {item.reference for item in comparisons} != set(REFERENCES):
        raise RuntimeError(
            f"Run `python -m validation.mpc compare {case_id} --models FastModel --references Buildings IDEAS` first"
        )
    openmodelica = [
        _openmodelica_run(case, "Buildings", "Buildings"),
        _openmodelica_run(case, "IDEAS", "IDEAS"),
        _openmodelica_run(case, FAST_MODEL, "FastModel, OpenModelica"),
    ]
    casadi, _, _ = _casadi(case, ocp_hours)
    benchmark = Benchmark(case=case_id, openmodelica=openmodelica, casadi=casadi)
    zones = [zone.zone for zone in comparisons[0].zones]
    _write(_zone_figure(case, [zones[0], zones[2]], (7, 14)), f"fast_model_{case_id}_zones")
    _write(_energy_figure(comparisons), f"fast_model_{case_id}_energy")
    _write(_speed_figure(benchmark), f"fast_model_{case_id}_speed")
    return benchmark


def render(benchmark: Benchmark) -> str:
    casadi = benchmark.casadi
    lines = [
        "| Model | Translation [s] | Simulation of the case [s] |",
        "|---|---|---|",
        *(f"| {run.model} | {run.translation:.0f} | {run.simulation:.0f} |" for run in benchmark.openmodelica),
        f"| FastModel, CasADi (CVODES, {casadi.steps} steps) | {casadi.translation:.1f} (rumoca) "
        f"| {casadi.simulation:.1f} |",
        "",
        f"building_mpc: {casadi.states} states, {casadi.inputs} inputs ({casadi.controls} controls), "
        f"{casadi.parameters} parameters.",
        f"CasADi against OpenModelica, zone RMSE: {max(casadi.deviation.values()):.3f} K at most.",
        f"Optimal control of {casadi.ocp_hours} h: {casadi.ocp_solve:.1f} s ({casadi.ocp_status}).",
    ]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(prog="python -m validation.mpc.benchmark")
    parser.add_argument("case", nargs="?", default="residential_after")
    parser.add_argument("--ocp-hours", type=int, default=24)
    arguments = parser.parse_args()
    benchmark = run(arguments.case, arguments.ocp_hours)
    CACHE_ROOT.joinpath(f"benchmark_{arguments.case}.json").write_text(benchmark.model_dump_json(indent=1))
    print(render(benchmark))


if __name__ == "__main__":
    main()
