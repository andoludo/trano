"""Markdown tables of a benchmark run: timings and solver statistics, then the accuracy of each profile."""

from __future__ import annotations

from validation.performance.accuracy import ZONE_TEMPERATURE_MAX, ZONE_TEMPERATURE_RMSE, Comparison
from validation.performance.cases import BenchmarkCase
from validation.performance.runner import CaseRun, Settings


def _number(value: float | None, digits: int = 1) -> str:
    return "-" if value is None else f"{value:.{digits}f}"


def _integer(value: int | None) -> str:
    return "-" if value is None else f"{value:,}"


def _systems(sizes: list[int]) -> str:
    large = [size for size in sizes if size > 1]
    return ", ".join(map(str, large[:6])) + (" ..." if len(large) > 6 else "") if large else "-"


def _speedup(run: CaseRun, baseline: CaseRun | None) -> str:
    if baseline is None or run is baseline or not run.simulation_time or not baseline.simulation_time:
        return ""
    return f"{baseline.simulation_time / run.simulation_time:.2f}x"


def timing_table(cases: list[BenchmarkCase], runs: list[CaseRun]) -> str:
    by_id = {run.case_id: run for run in runs}
    lines = [
        "| model | library | profile | translation [s] | simulation [s] | speed-up | states | "
        "nonlinear systems | linear systems | steps | Jacobians | conv. failures | state events |",
        "|---|---|---|---:|---:|---:|---:|---|---|---:|---:|---:|---:|",
    ]
    for case, run in zip(cases, runs, strict=True):
        if not run.success:
            lines.append(
                f"| {case.model.name} | {case.library} | {case.profile.name} | "
                f"{_number(run.translation_time)} | failed | | | | | | | | |"
            )
            continue
        lines.append(
            f"| {case.model.name} | {case.library} | {case.profile.name} | {_number(run.translation_time)} | "
            f"{_number(run.simulation_time)} | {_speedup(run, by_id.get(case.baseline.id))} | "
            f"{_integer(run.translation.states)} | {_systems(run.translation.nonlinear_systems)} | "
            f"{_systems(run.translation.linear_systems)} | {_integer(run.solver.steps)} | "
            f"{_integer(run.solver.jacobian_evaluations)} | {_integer(run.solver.convergence_test_failures)} | "
            f"{_integer(run.solver.state_events)} |"
        )
    return "\n".join(lines)


def accuracy_table(comparisons: list[Comparison]) -> str:
    lines = [
        "| case | zone T RMSE [K] | zone T max [K] | supply T RMSE [K] | supply T max [K] | "
        "heating energy | within tolerance |",
        "|---|---:|---:|---:|---:|---:|---|",
    ]
    for comparison in comparisons:
        zone = comparison.worst("zone_temperature")
        supply = comparison.worst("supply_temperature")
        zone_rmse, zone_max = (zone.rmse, zone.max_deviation) if zone else (None, None)
        supply_rmse, supply_max = (supply.rmse, supply.max_deviation) if supply else (None, None)
        energy = comparison.heating_energy_relative()
        lines.append(
            f"| {comparison.case_id} | {_number(zone_rmse, 3)} | {_number(zone_max, 2)} | "
            f"{_number(supply_rmse, 2)} | {_number(supply_max, 1)} | "
            f"{'-' if energy is None else f'{energy:+.1%}'} | "
            f"{'yes' if comparison.within_tolerance else '**no**'} |"
        )
    return "\n".join(lines)


def render(cases: list[BenchmarkCase], runs: list[CaseRun], comparisons: list[Comparison], settings: Settings) -> str:
    return f"""# Simulation performance benchmark

OpenModelica, {settings.method.upper()}, tolerance {settings.tolerance:g}, {settings.days} days from
1 January, one output point every {settings.output_interval} s. Translation is the wall-clock time of
`buildModel` (libraries loading and C compilation included), simulation that of the executable.

{timing_table(cases, runs)}

Accuracy against the baseline profile (first day left out). Tolerances: zone air temperature RMSE
{ZONE_TEMPERATURE_RMSE} K and maximum {ZONE_TEMPERATURE_MAX} K; heating energy 2 %. Water temperatures are
reported, not held to a tolerance (with steady-state sensors they read the incoming water at zero flow).

{accuracy_table(comparisons)}
"""
