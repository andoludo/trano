"""Markdown report of the comparisons between the MPC libraries (``mpc``, ``FastModel``) and Buildings or IDEAS."""

from validation.mpc.cases import CASES
from validation.mpc.harness import CaseComparison


def _percent(value: float | None, signed: bool = True) -> str:
    if value is None:
        return ""
    return f"{100 * value:+.1f} %" if signed else f"{100 * value:.1f} %"


def render_report(comparisons: list[CaseComparison]) -> str:
    order = list(CASES)
    ordered = sorted(comparisons, key=lambda item: (order.index(item.case), item.model, item.reference))
    lines = [
        "| Case | Model | Reference | Zone | RMSE [K] | Bias [K] | Max error [K] | Mean reference [degC] | Heat [kWh] "
        "| Radiator heat CV(RMSE) | Radiator heat NMBE |",
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for comparison in ordered:
        lines.extend(
            f"| {comparison.case} | {comparison.model} | {comparison.reference} | {zone.zone} | {zone.rmse:.2f} | "
            f"{zone.bias:+.2f} | "
            f"{zone.max_abs:.2f} | {zone.mean_reference:.2f} | {zone.heat_kwh:.0f} | "
            f"{_percent(zone.emitter_cv_rmse, signed=False)} | {_percent(zone.emitter_nmbe)} |"
            for zone in comparison.zones
        )
    circuits = [comparison for comparison in ordered if comparison.circuits]
    if circuits:
        lines += [
            "",
            "| Case | Reference | Water temperature | RMSE [K] | Bias [K] | Mean reference [degC] "
            "| Time compared (circulating) |",
            "|---|---|---|---|---|---|---|",
        ]
        lines.extend(
            f"| {comparison.case} | {comparison.reference} | {circuit.variable} | {circuit.rmse:.2f} | "
            f"{circuit.bias:+.2f} | "
            f"{circuit.mean_reference:.1f} | {100 * circuit.circulating:.0f} % |"
            for comparison in circuits
            for circuit in comparison.circuits
        )
    energies = [comparison for comparison in ordered if comparison.energies]
    if energies:
        lines += [
            "",
            "| Case | Reference | System | Energy | Reference [kWh] | FastModel [kWh] | Deviation |",
            "|---|---|---|---|---|---|---|",
        ]
        lines.extend(
            f"| {comparison.case} | {comparison.reference} | {energy.system} | {energy.quantity} | "
            f"{energy.reference_kwh:.1f} | "
            f"{energy.model_kwh:.1f} | {_percent(energy.deviation)} |"
            for comparison in energies
            for energy in comparison.energies
        )
    return "\n".join(lines)
