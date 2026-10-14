"""Cases comparing the MPC libraries (``mpc``, ``FastModel``) with the Buildings model of the same YAML file.

A case is a YAML building and a simulation period. The Buildings model is the reference; the ``mpc``
model replays the heat that the Buildings systems delivered to each zone and must reproduce the zone
temperatures (see :mod:`validation.mpc.harness`).
"""

from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field

ROOT = Path(__file__).resolve().parents[2]
BESTEST_CASES = ROOT / "validation" / "bestest" / "cases"
TEST_MODELS = ROOT / "tests" / "models"
EXAMPLES = ROOT / "examples"
DAY = 24 * 3600


class MPCCase(BaseModel):
    model_config = ConfigDict(frozen=True)

    id: str
    path: Path = Field(description="YAML building description")
    days: int = Field(gt=0, description="Simulated days")
    start_day: int = Field(0, ge=0, description="First simulated day of the year")
    description: str = ""
    fine_replay: bool = Field(
        False,
        description="The reference is recorded every minute and its controls replayed as means over 5 minutes: "
        "thermostatic valves and heater controllers act faster than the comparison interval",
    )

    @property
    def start_time(self) -> int:
        return self.start_day * DAY

    @property
    def end_time(self) -> int:
        return (self.start_day + self.days) * DAY


CASES = {
    case.id: case
    for case in (
        MPCCase(
            id="600FF",
            path=BESTEST_CASES / "case_600FF.yaml",
            days=365,
            description="BESTEST 600FF: lightweight single zone, free floating, Denver",
        ),
        MPCCase(
            id="900FF",
            path=BESTEST_CASES / "case_900FF.yaml",
            days=365,
            description="BESTEST 900FF: heavyweight single zone, free floating, Denver",
        ),
        MPCCase(
            id="600",
            path=BESTEST_CASES / "case_600.yaml",
            days=365,
            description="BESTEST 600: lightweight single zone, ideal heating and cooling (20/27 degC)",
        ),
        MPCCase(
            id="900",
            path=BESTEST_CASES / "case_900.yaml",
            days=365,
            description="BESTEST 900: heavyweight single zone, ideal heating and cooling (20/27 degC)",
        ),
        MPCCase(
            id="three_zones_ideal",
            fine_replay=True,
            path=TEST_MODELS / "three_zones_ideal_heaters.yaml",
            days=28,
            description="Three zones with internal walls and ideal heaters, January, Uccle",
        ),
        MPCCase(
            id="heat_pump_house",
            fine_replay=True,
            path=TEST_MODELS / "house_infiltration_air_water_heat_pump.yaml",
            days=28,
            description="Six-zone house, air-to-water heat pump with a buffer tank and radiators, January, Uccle",
        ),
        MPCCase(
            id="residential_before",
            fine_replay=True,
            path=EXAMPLES / "residential_before_renovation.yaml",
            days=28,
            description="Six-zone semi-detached house before renovation, gas boiler and radiators, January, Uccle",
        ),
        MPCCase(
            id="residential_after",
            fine_replay=True,
            path=EXAMPLES / "residential_after_renovation.yaml",
            days=28,
            description="Six-zone semi-detached house after renovation: air-to-water heat pump with a buffer, "
            "radiators, balanced ventilation with heat recovery, PV, January, Uccle",
        ),
    )
}
