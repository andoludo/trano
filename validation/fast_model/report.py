"""Markdown tables and figures of the FastModel-against-Buildings study, rendered from the committed results.

``render_report`` gives the tables of every case (KPIs of both libraries, hourly fidelity, sensitivity to the
perturbations, speed); ``write_figures`` the plotly figures of ``docs/validation/figures``; ``write_docs`` puts both
between the markers of ``docs/validation/fast_model_study.md``, whose prose is written by hand.
"""

from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path

import pandas as pd
import plotly.graph_objects as go  # type: ignore[import-untyped]
from plotly.subplots import make_subplots  # type: ignore[import-untyped]

from validation.fast_model.cases import BASE, CASES, ROOT, StudyCase
from validation.fast_model.harness import (
    BUILDINGS,
    FAST_MODEL,
    RunKpis,
    ZoneFidelity,
    compare_series,
    load_kpis,
    load_series,
    zone_names,
)

FIGURES = ROOT / "docs" / "validation" / "figures"
DOCS_PAGE = ROOT / "docs" / "validation" / "fast_model_study.md"
START, END = "<!-- results:start -->", "<!-- results:end -->"
COLORS = {BUILDINGS: "#2a78d6", FAST_MODEL: "#1baf7a"}
NEUTRAL, MUTED = "#8a8985", "#b8b7b1"
DAY = 86400
YEAR = 2001  # a non-leap year to name the hours of the series
# KPIs whose response to the perturbations is compared: attribute of RunKpis, label, unit
ENERGY_KPIS = (
    ("heating_kwh", "heating demand", "kWh"),
    ("peak_heating_kw", "peak heating", "kW"),
    ("cooling_kwh", "cooling demand", "kWh"),
    ("electricity_kwh", "heat pump electricity", "kWh"),
)
TEMPERATURE_RESOLUTION = 0.1  # [K] smallest change of a temperature KPI a sensitivity ratio is drawn from
ENERGY_RESOLUTION = 0.02  # smallest relative change of a load or a meter a sensitivity ratio is drawn from
TEMPERATURE_KPIS = (
    ("mean_temperature", "mean zone temperature", "degC"),
    ("min_temperature", "lowest hourly zone temperature", "degC"),
    ("max_temperature", "highest hourly zone temperature", "degC"),
)


def _number(value: float | None, digits: int = 1) -> str:
    return "" if value is None else f"{value:.{digits}f}"


def _percent(value: float | None, signed: bool = True) -> str:
    if value is None:
        return ""
    return f"{100 * value:+.1f} %" if signed else f"{100 * value:.1f} %"


def _signed(value: float | None, digits: int = 1) -> str:
    return "" if value is None else f"{value:+.{digits}f}"


def _runs(case: StudyCase, variant: str) -> tuple[RunKpis | None, RunKpis | None]:
    return load_kpis(case.id, variant, BUILDINGS), load_kpis(case.id, variant, FAST_MODEL)


def _variants_with_results(case: StudyCase) -> list[str]:
    return [variant for variant in case.variants if any(run is not None for run in _runs(case, variant))]


def kpi_table(case: StudyCase) -> str:
    """Zone temperatures, heating and cooling of both libraries, per variant and zone, with the totals of the case."""
    lines = [
        "| Variant | Zone | Mean T [degC] | Lowest T [degC] | Highest T [degC] | Heating [kWh] | Peak heating [kW] "
        "| Cooling [kWh] |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for variant in _variants_with_results(case):
        buildings, fast_model = _runs(case, variant)
        available = buildings or fast_model
        zones = [zone.zone for zone in available.zones] if available is not None else []
        # a perturbed multi-zone case shows the totals of the building only, the base case every zone
        for zone in zones if variant == BASE or len(zones) == 1 else []:
            pair = [run.zone(zone) if run is not None else None for run in (buildings, fast_model)]
            lines.append(
                f"| {variant} | {zone} | {_pair(pair, 'mean_temperature', 2)} | {_pair(pair, 'min_temperature', 2)} | "
                f"{_pair(pair, 'max_temperature', 2)} | {_pair(pair, 'heating_kwh', 0)} | "
                f"{_pair(pair, 'peak_heating_kw', 2)} | {_pair(pair, 'cooling_kwh', 0)} |"
            )
        if len(zones) > 1:
            runs = [buildings, fast_model]
            lines.append(
                f"| {variant} | **all zones** | {_pair(runs, 'mean_temperature', 2)} | "
                f"{_pair(runs, 'min_temperature', 2)} | {_pair(runs, 'max_temperature', 2)} | "
                f"{_pair(runs, 'heating_kwh', 0)} | "
                f"{_pair(runs, 'peak_heating_kw', 2)} | {_pair(runs, 'cooling_kwh', 0)} |"
            )
    return "\n".join(lines)


def _pair(items: Iterable[object], name: str, digits: int) -> str:
    """``Buildings / FastModel`` values of a KPI, as the two libraries report it."""
    values = []
    for item in items:
        if item is None:
            values.append("")
        elif isinstance(item, RunKpis):
            values.append(_number(item.value(name), digits))
        else:
            values.append(_number(getattr(item, name), digits))
    return " / ".join(values)


def meters_table(case: StudyCase) -> str:
    """Meters and speed of both libraries per variant: heat pump electricity, fuel, fans, PV, OpenModelica time."""
    lines = [
        "| Variant | Heating [kWh] | Heat pump electricity [kWh] | Fans [kWh] | PV [kWh] | OpenModelica time [s] |",
        "|---|---|---|---|---|---|",
    ]
    for variant in _variants_with_results(case):
        runs = list(_runs(case, variant))
        times = " / ".join(_number(run.wall_time, 0) if run is not None else "" for run in runs)
        lines.append(
            f"| {variant} | {_pair(runs, 'heating_kwh', 0)} {_deviation(runs, 'heating_kwh')} | "
            f"{_pair(runs, 'electricity_kwh', 0)} {_deviation(runs, 'electricity_kwh')} | "
            f"{_pair(runs, 'fans_kwh', 1)} | {_pair(runs, 'pv_kwh', 0)} | {times} |"
        )
    return "\n".join(lines)


def _deviation(runs: list[RunKpis | None], name: str) -> str:
    buildings, fast_model = runs
    if buildings is None or fast_model is None:
        return ""
    reference, value = buildings.value(name), fast_model.value(name)
    if reference is None or value is None or abs(reference) < 1e-9:
        return ""
    return f"({_percent((value - reference) / reference)})"


def fidelity(case: StudyCase, variant: str) -> list[ZoneFidelity]:
    reference, model = load_series(case.id, variant, BUILDINGS), load_series(case.id, variant, FAST_MODEL)
    if reference is None or model is None:
        return []
    return compare_series(reference, model)


def fidelity_table(case: StudyCase) -> str:
    """Hour by hour: the FastModel zone temperatures and heating against Buildings, per variant and zone."""
    lines = [
        "| Variant | Zone | T RMSE [K] | T bias [K] | T max error [K] | Heating CV(RMSE) | Heating NMBE "
        "| Heating deviation |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for variant in _variants_with_results(case):
        lines.extend(
            f"| {variant} | {row.zone} | {row.rmse:.2f} | {row.bias:+.2f} | {row.max_abs:.2f} | "
            f"{_percent(row.heating_cv_rmse, signed=False)} | {_percent(row.heating_nmbe)} | "
            f"{_percent(row.heating_deviation)} |"
            for row in fidelity(case, variant)
        )
    return "\n".join(lines)


def _kpis_of(case: StudyCase, base: RunKpis) -> list[tuple[str, str, str]]:
    """The KPIs that mean something for the case: temperatures always, loads only where the base case has them."""
    kpis = list(TEMPERATURE_KPIS)
    for name, label, unit in ENERGY_KPIS:
        value = base.value(name)
        if value is not None and abs(value) > 1e-6:
            kpis.append((name, label, unit))
    return kpis


def sensitivity_rows(case: StudyCase) -> list[dict[str, object]]:
    """For every perturbation and KPI: the change each library sees, and the ratio of the two changes."""
    base_buildings, base_fast_model = _runs(case, BASE)
    if base_buildings is None or base_fast_model is None:
        return []
    rows: list[dict[str, object]] = []
    for perturbation in case.perturbations:
        buildings, fast_model = _runs(case, perturbation.id)
        if buildings is None or fast_model is None:
            continue
        for name, label, unit in _kpis_of(case, base_buildings):
            reference = (base_buildings.value(name), buildings.value(name))
            model = (base_fast_model.value(name), fast_model.value(name))
            if None in reference or None in model:
                continue
            delta_reference = reference[1] - reference[0]  # type: ignore[operator]
            delta_model = model[1] - model[0]  # type: ignore[operator]
            # a change smaller than these moves within the noise of an hourly KPI: no ratio is drawn from it
            tolerance = TEMPERATURE_RESOLUTION if unit == "degC" else ENERGY_RESOLUTION * abs(reference[0])  # type: ignore[arg-type]
            if abs(delta_reference) <= tolerance and abs(delta_model) <= tolerance:
                continue  # neither library moves
            rows.append(
                {
                    "perturbation": perturbation.id,
                    "description": perturbation.description,
                    "group": perturbation.group,
                    "kpi": name,
                    "label": label,
                    "unit": unit,
                    "buildings_base": reference[0],
                    "buildings": reference[1],
                    "fast_model_base": model[0],
                    "fast_model": model[1],
                    "delta_buildings": delta_reference,
                    "delta_fast_model": delta_model,
                    "ratio": delta_model / delta_reference if abs(delta_reference) > tolerance else None,
                }
            )
    return rows


def sensitivity_table(case: StudyCase) -> str:
    lines = [
        "| Perturbation | KPI | Buildings: base, perturbed (change) | FastModel: base, perturbed (change) "
        "| FastModel change / Buildings change |",
        "|---|---|---|---|---|",
    ]
    for row in sensitivity_rows(case):
        unit = str(row["unit"])
        digits = 0 if unit == "kWh" else 2
        relative = unit != "degC"
        base_b, pert_b = float(row["buildings_base"]), float(row["buildings"])  # type: ignore[arg-type]
        base_f, pert_f = float(row["fast_model_base"]), float(row["fast_model"])  # type: ignore[arg-type]
        change_b = _change(base_b, pert_b, relative, digits)
        change_f = _change(base_f, pert_f, relative, digits)
        ratio = row["ratio"]
        lines.append(
            f"| {row['perturbation']} | {row['label']} [{unit}] | "
            f"{base_b:.{digits}f}, {pert_b:.{digits}f} ({change_b}) | "
            f"{base_f:.{digits}f}, {pert_f:.{digits}f} ({change_f}) | "
            f"{'' if ratio is None else f'{float(ratio):.2f}'} |"  # type: ignore[arg-type]
        )
    return "\n".join(lines)


def _change(base: float, perturbed: float, relative: bool, digits: int) -> str:
    if relative and abs(base) > 1e-9:
        return _percent((perturbed - base) / base)
    return _signed(perturbed - base, digits)


def render_case(case: StudyCase) -> str:
    variants = _variants_with_results(case)
    if not variants:
        return f"### {case.id}\n\n{case.description}.\n\nNo results yet.\n"
    missing = [variant for variant in case.variants if variant not in variants]
    parts = [f"### {case.id}", "", f"{case.description}."]
    if missing:
        parts += ["", f"Variants without results yet: {', '.join(missing)}."]
    parts += ["", "**KPIs of both libraries** (Buildings / FastModel):", "", kpi_table(case)]
    parts += ["", "**Meters and speed** (Buildings / FastModel, deviation of FastModel):", "", meters_table(case)]
    parts += ["", "**Hour by hour**, FastModel against Buildings:", "", fidelity_table(case)]
    if sensitivity_rows(case):
        parts += ["", "**Response to the perturbations**:", "", sensitivity_table(case)]
    for name in figure_names(case):
        parts += ["", f"{{!validation/figures/{name}.html!}}"]
    return "\n".join(parts) + "\n"


def render_report() -> str:
    return "\n".join(render_case(case) for case in CASES.values())


# ------------------------------------------------------------------------------------------------------- figures
def _layout(figure: go.Figure, title: str, height: int) -> go.Figure:
    figure.update_layout(
        title={"text": title, "x": 0.0, "xanchor": "left", "font": {"size": 15}},
        height=height,
        margin={"l": 60, "r": 20, "t": 70, "b": 40},
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        font={"family": "Roboto, Helvetica, Arial, sans-serif", "size": 12, "color": NEUTRAL},
        hovermode="x unified",
        legend={"orientation": "h", "y": -0.08, "x": 0},
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


def _window(frame: pd.DataFrame, first_day: int, days: int) -> pd.DataFrame:
    start, end = first_day * DAY, (first_day + days) * DAY
    selected = frame[(frame.index >= start) & (frame.index < end)].copy()
    selected.index = pd.Timestamp(f"{YEAR}-01-01") + pd.to_timedelta(selected.index, unit="s")
    return selected


def traces_figure(case: StudyCase, first_day: int, days: int, variant: str = BASE) -> go.Figure | None:
    """Zone temperatures and heating power of both libraries over a window of days."""
    frames = {library: load_series(case.id, variant, library) for library in (BUILDINGS, FAST_MODEL)}
    available = {library: frame for library, frame in frames.items() if frame is not None}
    if not available:
        return None
    zones = zone_names(next(iter(available.values())))
    rows = len(zones) + 1
    titles = [f"{zone}: air temperature [degC]" for zone in zones] + ["heating (positive) and cooling [kW]"]
    figure = make_subplots(rows=rows, cols=1, shared_xaxes=True, subplot_titles=titles, vertical_spacing=0.08 / rows)
    for library, frame in available.items():
        window = _window(frame, first_day, days)
        for row, zone in enumerate(zones, start=1):
            figure.add_trace(
                go.Scatter(
                    x=window.index,
                    y=window[f"{zone}.T"],
                    name=library,
                    legendgroup=library,
                    showlegend=row == 1,
                    line={"color": COLORS[library], "width": 1.5},
                ),
                row=row,
                col=1,
            )
        figure.add_trace(
            go.Scatter(
                x=window.index,
                y=(window["heating"] - window["cooling"]) / 1000,
                name=library,
                legendgroup=library,
                showlegend=False,
                line={"color": COLORS[library], "width": 1.5},
            ),
            row=rows,
            col=1,
        )
    if "TOut" in next(iter(available.values())):
        window = _window(next(iter(available.values())), first_day, days)
        figure.add_trace(
            go.Scatter(
                x=window.index, y=window["TOut"], name="outdoor", line={"color": MUTED, "width": 1, "dash": "dot"}
            ),
            row=1,
            col=1,
        )
    start = pd.Timestamp(f"{YEAR}-01-01") + pd.Timedelta(days=first_day)
    title = f"{case.id} ({variant}): {days} days from {start:%d %B}, Buildings and FastModel"
    return _layout(figure, title, 170 * rows + 90)


def sensitivity_figure(case: StudyCase) -> go.Figure | None:
    """The change of each KPI under each perturbation, Buildings beside FastModel."""
    rows = sensitivity_rows(case)
    if not rows:
        return None
    kpis = list(dict.fromkeys((str(row["kpi"]), str(row["label"]), str(row["unit"])) for row in rows))
    figure = make_subplots(
        rows=1,
        cols=len(kpis),
        subplot_titles=[f"{label} [{'K' if unit == 'degC' else '%'}]" for _, label, unit in kpis],
    )
    for column, (kpi, _, unit) in enumerate(kpis, start=1):
        selected = [row for row in rows if row["kpi"] == kpi]
        for library, delta, base in (
            (BUILDINGS, "delta_buildings", "buildings_base"),
            (FAST_MODEL, "delta_fast_model", "fast_model_base"),
        ):
            values = []
            for row in selected:
                change = float(row[delta])  # type: ignore[arg-type]
                reference = float(row[base])  # type: ignore[arg-type]
                values.append(
                    change if unit == "degC" else (100 * change / reference if abs(reference) > 1e-9 else 0.0)
                )
            figure.add_trace(
                go.Bar(
                    x=[str(row["perturbation"]) for row in selected],
                    y=values,
                    name=library,
                    legendgroup=library,
                    showlegend=column == 1,
                    marker_color=COLORS[library],
                ),
                row=1,
                col=column,
            )
    figure.update_layout(barmode="group")
    return _layout(figure, f"{case.id}: response of each library to the perturbations", 380)


def figure_names(case: StudyCase) -> list[str]:
    names = [f"fast_model_study_{case.id}_days_{first}_{days}" for first, days in case.trace_windows]
    if sensitivity_rows(case):
        names.append(f"fast_model_study_{case.id}_sensitivity")
    return names


def write_figures() -> list[Path]:
    written = []
    for case in CASES.values():
        for first_day, days in case.trace_windows:
            figure = traces_figure(case, first_day, days)
            if figure is not None:
                written.append(_write(figure, f"fast_model_study_{case.id}_days_{first_day}_{days}"))
        figure = sensitivity_figure(case)
        if figure is not None:
            written.append(_write(figure, f"fast_model_study_{case.id}_sensitivity"))
    return written


def write_docs() -> Path:
    """Replace the generated part of the documentation page (between the markers) with the current tables."""
    if DOCS_PAGE.exists():
        text = DOCS_PAGE.read_text()
    else:
        text = f"# FastModel against Buildings: BESTEST and a renovated house\n\n{START}\n{END}\n"
    if START not in text or END not in text:
        raise ValueError(f"{DOCS_PAGE} must hold the markers {START} and {END}")
    head, rest = text.split(START, 1)
    _, tail = rest.split(END, 1)
    DOCS_PAGE.write_text(f"{head}{START}\n{render_report()}\n{END}{tail}")
    return DOCS_PAGE
