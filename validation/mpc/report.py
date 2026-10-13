"""Markdown report of the comparisons between the ``mpc`` library and Buildings."""

from validation.mpc.cases import CASES
from validation.mpc.harness import CaseComparison


def _percent(value: float | None, signed: bool = True) -> str:
    if value is None:
        return ""
    return f"{100 * value:+.1f} %" if signed else f"{100 * value:.1f} %"


def render_report(comparisons: list[CaseComparison]) -> str:
    order = list(CASES)
    ordered = sorted(comparisons, key=lambda item: (order.index(item.case), item.model))
    lines = [
        "| Case | Model | Zone | RMSE [K] | Bias [K] | Max error [K] | Mean Buildings [degC] | Heat [kWh] "
        "| Radiator heat CV(RMSE) | Radiator heat NMBE |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for comparison in ordered:
        lines.extend(
            f"| {comparison.case} | {comparison.model} | {zone.zone} | {zone.rmse:.2f} | {zone.bias:+.2f} | "
            f"{zone.max_abs:.2f} | {zone.mean_reference:.2f} | {zone.heat_kwh:.0f} | "
            f"{_percent(zone.emitter_cv_rmse, signed=False)} | {_percent(zone.emitter_nmbe)} |"
            for zone in comparison.zones
        )
    circuits = [comparison for comparison in ordered if comparison.circuits]
    if circuits:
        lines += [
            "",
            "| Case | Water temperature | RMSE [K] | Bias [K] | Mean Buildings [degC] |",
            "|---|---|---|---|---|",
        ]
        lines.extend(
            f"| {comparison.case} | {circuit.variable} | {circuit.rmse:.2f} | {circuit.bias:+.2f} | "
            f"{circuit.mean_reference:.1f} |"
            for comparison in circuits
            for circuit in comparison.circuits
        )
    return "\n".join(lines)
