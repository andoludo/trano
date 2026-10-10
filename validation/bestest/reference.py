"""Reference results of ANSI/ASHRAE Standard 140-2020 (BESTEST), section 5.2 cases.

The data file is the one shipped with the Modelica Buildings library (see ``spec/README.md``): the
results of six reference programs for every case, the acceptance limits of the standard for the
annual heating and cooling loads, and the hourly data of a few cases.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, Field

DATA_FILE = Path(__file__).parent.joinpath("spec", "ashrae140_2020.dat")

Kpi = Literal[
    "annual_heating",
    "annual_cooling",
    "peak_heating",
    "peak_cooling",
    "maximum_temperature",
    "minimum_temperature",
    "mean_temperature",
]
KPIS: tuple[Kpi, ...] = (
    "annual_heating",
    "annual_cooling",
    "peak_heating",
    "peak_cooling",
    "maximum_temperature",
    "minimum_temperature",
    "mean_temperature",
)
UNITS: dict[Kpi, str] = {
    "annual_heating": "MWh",
    "annual_cooling": "MWh",
    "peak_heating": "kW",
    "peak_cooling": "kW",
    "maximum_temperature": "degC",
    "minimum_temperature": "degC",
    "mean_temperature": "degC",
}
_TABLE_KPIS: dict[str, Kpi] = {
    "Table B8-1": "annual_heating",
    "Table B8-2": "annual_cooling",
    "Table B8-3": "peak_heating",
    "Table B8-4": "peak_cooling",
    "MAXIMUM ANNUAL": "maximum_temperature",
    "MINIMUM ANNUAL": "minimum_temperature",
    "AVERAGE ANNUAL": "mean_temperature",
}
_MONTHS = {
    "JAN": 1, "FEB": 2, "MAR": 3, "APR": 4, "MAY": 5, "JUN": 6,
    "JUL": 7, "AUG": 8, "SEP": 9, "OCT": 10, "NOV": 11, "DEC": 12,
}  # fmt: skip
_VALUE = re.compile(r"^(?P<value>[-+]?\d+(?:\.\d+)?)(?:\((?P<at>[^)]*)\)?)?$")
_HOURLY = re.compile(
    r"HOURLY (?P<kind>FREE FLOAT TEMPERATURE|HEATING & COOLING LOAD) DATA .*: "
    r"CASE (?P<case>\w+?)V?,? (?P<month>[A-Z]+) (?P<day>\d+)"
)
_BINS = re.compile(r"HOURLY ANNUAL ZONE TEMPERATURE BIN DATA .*: CASE (?P<case>\w+)")


class ProgramValue(BaseModel):
    """Result of one reference program, with the time of occurrence for peaks and extremes."""

    value: float
    at: str | None = None  # "dd-Mon:h", h being the hour of the day ending at the value (1 to 24)


class KpiReference(BaseModel):
    case: str
    kpi: Kpi
    programs: dict[str, ProgramValue]
    lower_limit: float | None = None  # acceptance limits of the standard (annual loads only)
    upper_limit: float | None = None

    @property
    def values(self) -> list[float]:
        return [program.value for program in self.programs.values()]

    @property
    def minimum(self) -> float:
        return min(self.values)

    @property
    def maximum(self) -> float:
        return max(self.values)

    @property
    def mean(self) -> float:
        return sum(self.values) / len(self.values)

    @property
    def unit(self) -> str:
        return UNITS[self.kpi]


class HourlyReference(BaseModel):
    """Hourly values of one day: zone temperature [degC] or heating (+) and cooling (-) load [kWh]."""

    case: str
    quantity: Literal["temperature", "load"]
    month: int
    day: int
    programs: dict[str, list[float]]


class BinReference(BaseModel):
    """Hours per 1 K temperature bin over the year."""

    case: str
    temperatures: list[int]
    programs: dict[str, list[int]]


class ReferenceData(BaseModel):
    kpis: list[KpiReference] = Field(default_factory=list)
    hourly: list[HourlyReference] = Field(default_factory=list)
    bins: list[BinReference] = Field(default_factory=list)

    @property
    def cases(self) -> list[str]:
        return sorted({reference.case for reference in self.kpis}, key=_case_order)

    def kpi(self, case: str, kpi: Kpi) -> KpiReference | None:
        return next((reference for reference in self.kpis if reference.case == case and reference.kpi == kpi), None)

    def kpis_of(self, case: str) -> list[KpiReference]:
        return [reference for reference in self.kpis if reference.case == case]

    def hourly_of(self, case: str) -> list[HourlyReference]:
        return [reference for reference in self.hourly if reference.case == case]


def _case_order(case: str) -> tuple[int, str]:
    return int(re.sub(r"\D", "", case) or 0), case


def _split(line: str) -> list[str]:
    return [item.strip() for item in line.split(",")]


def _program_value(text: str) -> ProgramValue | None:
    if text in ("", "N/A"):
        return None
    match = _VALUE.match(text)
    if match is None:
        raise ValueError(f"Unreadable reference value {text!r}")
    return ProgramValue(value=float(match.group("value")), at=match.group("at"))


def load_reference(path: Path = DATA_FILE) -> ReferenceData:
    """Parse the data file: tables of KPIs per case, then hourly data and temperature bins."""
    data = ReferenceData()
    kpi: Kpi | None = None
    hourly: HourlyReference | None = None
    bins: BinReference | None = None
    columns: list[str] = []
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("---"):
            continue
        if line.startswith("#"):
            kpi, hourly, bins = _section(line.lstrip("# ").strip(), kpi, hourly, bins, data)
            continue
        cells = _split(line)
        if cells[0] in ("Case", "Hours", "Temp"):
            columns = cells[1:]
            continue
        if hourly is not None:
            for program, cell in zip(columns, cells[1:], strict=True):
                hourly.programs.setdefault(program, []).append(float(cell))
        elif bins is not None:
            bins.temperatures.append(int(cells[0]))
            for program, cell in zip(columns, cells[1:], strict=True):
                bins.programs.setdefault(program, []).append(int(cell))
        elif kpi is not None:
            data.kpis.append(_kpi_row(kpi, columns, cells))
    return data


def _section(
    header: str,
    kpi: Kpi | None,
    hourly: HourlyReference | None,
    bins: BinReference | None,
    data: ReferenceData,
) -> tuple[Kpi | None, HourlyReference | None, BinReference | None]:
    """Interpret a comment line: section headers switch the table being read, others are ignored."""
    for prefix, table_kpi in _TABLE_KPIS.items():
        if header.startswith(prefix):
            return table_kpi, None, None
    if match := _HOURLY.search(header):
        hourly = HourlyReference(
            case=match.group("case"),
            quantity="temperature" if "TEMPERATURE" in match.group("kind") else "load",
            month=_MONTHS[match.group("month")[:3]],
            day=int(match.group("day")),
            programs={},
        )
        data.hourly.append(hourly)
        return None, hourly, None
    if match := _BINS.search(header):
        bins = BinReference(case=match.group("case"), temperatures=[], programs={})
        data.bins.append(bins)
        return None, None, bins
    return kpi, hourly, bins


def _kpi_row(kpi: Kpi, columns: list[str], cells: list[str]) -> KpiReference:
    programs: dict[str, ProgramValue] = {}
    limits: dict[str, float] = {}
    for column, cell in zip(columns, cells[1:], strict=True):
        if column in ("LowerLimit", "UpperLimit"):
            limits[column] = float(cell)
        elif (value := _program_value(cell)) is not None:
            programs[column] = value
    return KpiReference(
        case=cells[0],
        kpi=kpi,
        programs=programs,
        lower_limit=limits.get("LowerLimit"),
        upper_limit=limits.get("UpperLimit"),
    )
