"""Compare the key outputs of a case with those of its baseline: RMSE, maximum deviation, energy.

The signals are found by name in the result files, so the comparison works for every library and system:
zone air temperatures, supply and return water temperatures, and heating powers (radiators and heat
generators). The first day is left out: both runs start from the same state but the first boiler cycles
differ the most, and the benchmark is about the steady periodic behaviour.
"""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np
from pydantic import BaseModel

DAY = 24 * 3600
GRID_POINTS = 4000

# Tolerances a faster profile is held to; a deviation beyond them is flagged in the report.
ZONE_TEMPERATURE_RMSE = 0.1  # [K]
ZONE_TEMPERATURE_MAX = 0.5  # [K]
ENERGY_RELATIVE = 0.02  # [-] heating energy over the compared period

SIGNAL_PATTERNS: dict[str, str] = {
    "zone_temperature": r"^building\.envelope1\.\w+\.(?:heaPorAir\.T|TSensor)$",
    "supply_temperature": r"^building\.\w+\.\w+\.(?:TSupply|supTemp|temSup|temperature_sensor\d)\.T$",
    "return_temperature": r"^building\.\w+\.\w+\.(?:TReturn|retTemp)\.T$",
    "heating_power": (
        r"^building\.(?:emission1\.radiator_\w+\.Q_flow|production1\.\w+\.(?:boi\.Q_flow|\w+\.QCon_flow))$"
    ),
}


class SignalComparison(BaseModel):
    kind: str
    name: str
    rmse: float
    max_deviation: float
    mean_baseline: float
    mean_case: float
    energy_relative: float | None = None  # heating powers only

    @property
    def within_tolerance(self) -> bool:
        """Zone temperatures are held to their tolerance one by one; the other signals are reported only."""
        if self.kind == "zone_temperature":
            return self.rmse <= ZONE_TEMPERATURE_RMSE and self.max_deviation <= ZONE_TEMPERATURE_MAX
        return True


class Comparison(BaseModel):
    case_id: str
    signals: list[SignalComparison]

    @property
    def within_tolerance(self) -> bool:
        """Every zone temperature within its tolerance and the total heating energy within its own.

        The energy is held to its tolerance in total: an emitter delivering a few watts on average has a
        relative difference that says nothing about the building.
        """
        energy = self.heating_energy_relative()
        energy_ok = energy is None or abs(energy) <= ENERGY_RELATIVE
        return energy_ok and all(signal.within_tolerance for signal in self.signals)

    def worst(self, kind: str) -> SignalComparison | None:
        candidates = [signal for signal in self.signals if signal.kind == kind]
        return max(candidates, key=lambda signal: signal.rmse, default=None)

    def heating_energy_relative(self) -> float | None:
        """Relative difference of the total heating delivered by the emitters (or produced, without emitters)."""
        powers = [signal for signal in self.signals if signal.kind == "heating_power"]
        emitters = [signal for signal in powers if ".radiator_" in signal.name] or powers
        baseline = sum(abs(signal.mean_baseline) for signal in emitters)
        if not emitters or baseline == 0:
            return None
        return sum(abs(signal.mean_case) for signal in emitters) / baseline - 1


def select_signals(names: list[str]) -> dict[str, list[str]]:
    return {
        kind: sorted(name for name in names if re.match(pattern, name)) for kind, pattern in SIGNAL_PATTERNS.items()
    }


def compare(case_id: str, baseline_file: Path, case_file: Path, skip: float = DAY) -> Comparison:
    from buildingspy.io.outputfile import Reader  # type: ignore[import-untyped]

    baseline, case = Reader(str(baseline_file), "dymola"), Reader(str(case_file), "dymola")
    common = sorted(set(baseline.varNames()) & set(case.varNames()))
    signals: list[SignalComparison] = []
    grid: np.ndarray | None = None
    for kind, names in select_signals(common).items():
        for name in names:
            time_a, values_a = baseline.values(name)
            time_b, values_b = case.values(name)
            if grid is None:
                first, stop = max(time_a[0], time_b[0]), min(time_a[-1], time_b[-1])
                start = first + skip if first + skip < stop else first  # a run of one day is compared whole
                grid = np.linspace(start, stop, GRID_POINTS)
            a, b = np.interp(grid, time_a, values_a), np.interp(grid, time_b, values_b)
            deviation = b - a
            energy_a = float(np.trapezoid(a, grid))
            signals.append(
                SignalComparison(
                    kind=kind,
                    name=name,
                    rmse=float(np.sqrt(np.mean(deviation**2))),
                    max_deviation=float(np.max(np.abs(deviation))),
                    mean_baseline=float(np.mean(a)),
                    mean_case=float(np.mean(b)),
                    energy_relative=(float(np.trapezoid(b, grid)) / energy_a - 1)
                    if kind == "heating_power" and abs(energy_a) > 0
                    else None,
                )
            )
    return Comparison(case_id=case_id, signals=signals)
