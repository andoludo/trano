"""Markdown report of the comparisons between the ``mpc`` library and Buildings."""

from validation.mpc.cases import CASES
from validation.mpc.harness import CaseComparison


def render_report(comparisons: list[CaseComparison]) -> str:
    lines = [
        "| Case | Model | Zone | RMSE [K] | Bias [K] | Max error [K] | Mean Buildings [degC] | Replayed heat [kWh] |",
        "|---|---|---|---|---|---|---|---|",
    ]
    order = list(CASES)
    for comparison in sorted(comparisons, key=lambda item: (order.index(item.case), item.model)):
        lines.extend(
            f"| {comparison.case} | {comparison.model} | {zone.zone} | {zone.rmse:.2f} | {zone.bias:+.2f} | "
            f"{zone.max_abs:.2f} | {zone.mean_reference:.2f} | {zone.heat_kwh:.0f} |"
            for zone in comparison.zones
        )
    return "\n".join(lines)
