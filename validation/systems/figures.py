"""Figures of the systems validation: the hourly traces of a case, one figure per case with the libraries side by
side, written as HTML fragments (plotly.js from its CDN) that the documentation page includes."""

from __future__ import annotations

from pathlib import Path

import plotly.graph_objects as go  # type: ignore[import-untyped]
from plotly.subplots import make_subplots  # type: ignore[import-untyped]

from validation.systems.cases import CASES, Family
from validation.systems.harness import CaseResult

FIGURES = Path(__file__).parents[2].joinpath("docs", "validation", "figures")
# categorical slots, fixed order (validated reference palette); neutral ink for references
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948")
NEUTRAL = "#8a8985"
TEMPERATURES: dict[Family, list[tuple[str, str]]] = {
    Family.heat_pump: [("zone_c", "zone"), ("hp_buffer_c", "buffer"), ("hp_supply_setpoint_c", "supply set point")],
    Family.boiler: [("zone_c", "zone"), ("boiler_water_c", "boiler water"), ("tank_top_c", "tank top")],
    Family.hydronics: [("zone_c", "zone"), ("mixing_supply_c", "radiator supply")],
    Family.ventilation: [("zone_c", "zone"), ("vent_supply_c", "supply air")],
    Family.house: [("zone_c", "zone")],
}
POWERS: dict[Family, list[tuple[str, str]]] = {
    Family.heat_pump: [("hp_heat_w", "heat"), ("hp_power_w", "electricity")],
    Family.boiler: [("boiler_heat_w", "heat to the water"), ("boiler_fuel_w", "fuel")],
    Family.hydronics: [("valve_opening", "valve opening [-]"), ("pump_power_w", "pump [W]")],
    Family.ventilation: [("heating_w", "heating")],
    Family.house: [],
}


def figure_name(case_id: str) -> str:
    return f"systems_{case_id}"


def case_figure(case_id: str, results: dict[str, CaseResult]) -> go.Figure:
    """Temperatures (top) and powers (bottom) of the week, one line style per library."""
    case = CASES[case_id]
    libraries = [library for library in results if results[library].traces]
    figure = make_subplots(
        rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.08, subplot_titles=("Temperatures", "Powers")
    )
    dashes = dict(zip(libraries, ("solid", "dash", "dot"), strict=False))
    outdoor_drawn = False
    for library in libraries:
        traces = results[library].traces
        hours = [hour / 24 for hour in range(len(next(iter(traces.values()))))]
        if "outdoor_c" in traces and not outdoor_drawn:
            figure.add_trace(
                go.Scatter(x=hours, y=traces["outdoor_c"], name="outdoor", line={"color": NEUTRAL, "width": 1}),
                row=1,
                col=1,
            )
            outdoor_drawn = True
        for index, (key, label) in enumerate(TEMPERATURES[case.family]):
            if key in traces:
                figure.add_trace(
                    go.Scatter(
                        x=hours,
                        y=traces[key],
                        name=f"{label} ({library})",
                        line={"color": SERIES[index % len(SERIES)], "dash": dashes[library]},
                    ),
                    row=1,
                    col=1,
                )
        for index, (key, label) in enumerate(POWERS[case.family]):
            if key in traces:
                figure.add_trace(
                    go.Scatter(
                        x=hours,
                        y=traces[key],
                        name=f"{label} ({library})",
                        line={"color": SERIES[(index + 3) % len(SERIES)], "dash": dashes[library]},
                    ),
                    row=2,
                    col=1,
                )
    figure.update_layout(
        title=f"{case_id}: {case.description}",
        height=640,
        template="plotly_white",
        legend={"orientation": "h", "y": -0.12},
        margin={"l": 50, "r": 20, "t": 60, "b": 40},
    )
    figure.update_xaxes(title_text="day of the week", row=2, col=1)
    figure.update_yaxes(title_text="degC", row=1, col=1)
    figure.update_yaxes(title_text="W", row=2, col=1)
    return figure


def write_figure(case_id: str, results: dict[str, CaseResult], directory: Path = FIGURES) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    path = directory.joinpath(f"{figure_name(case_id)}.html")
    case_figure(case_id, results).write_html(
        path,
        include_plotlyjs="cdn",
        full_html=False,
        config={"displaylogo": False, "responsive": True},
        div_id=figure_name(case_id),
    )
    return path
