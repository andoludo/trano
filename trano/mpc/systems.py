"""Energy systems of the ``mpc`` library: heat pump, boiler, chiller, DHW tank, PV, battery, EV charger.

The systems are rendered **inline** into ``building_mpc`` next to the zone equations, because the
CasADi target of rumoca accepts neither output variables nor sub-models: only explicit, smooth
state equations, inputs and parameters. Each converter therefore takes its *electrical* power as
control input and the heat it delivers is derived inside the ODE from a smooth performance
polynomial; the storages are extra states.

* :class:`HeatPump`: ``Q = COP(TOut, TSup) * P`` per zone, with a bilinear COP around the
  EN 14825 rating point A7/W35. The supply temperature is the emitter state of the zone plus an
  offset (R4C3) or a parameter (other zone models).
* :class:`GasBoiler`: the zones keep their thermal input; the efficiency is a parameter and the
  interface reports the carrier for the cost.
* :class:`Chiller`: ``Q = -EER(TOut) * P`` per zone.
* :class:`DHWTank`: a lumped tank temperature fed by a heat pump (or an electric resistance),
  losing heat to the zone it stands in, drained by a draw-off disturbance.
* :class:`Battery` and :class:`EVCharger`: an energy state with charging (and discharging)
  power inputs; the EV is also drained by a driving disturbance.
* :class:`Photovoltaic`: a generation disturbance input, computed from the irradiance on the
  panel orientation by the runnable model and by the MPC runtime.

What is *not* in the ODE (electrical balance at the meter, tariffs, the compressor limit shared
by the zones of one heat pump, state of charge bounds) is carried by the interface JSON.
"""

from enum import Enum
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator

from trano.mpc.parameters import ModelicaParameter, modelica_parameters_of, rc_field, validate_identifier

COP_REFERENCE_OUTDOOR = 280.15
"""Outdoor temperature of the COP rating point [K] (EN 14825 A7)."""
COP_REFERENCE_SUPPLY = 308.15
"""Supply temperature of the COP rating point [K] (EN 14825 W35)."""
EER_REFERENCE_OUTDOOR = 308.15
"""Outdoor temperature of the EER rating point [K] (EN 14825 A35)."""
WATER_VOLUMETRIC_HEAT_CAPACITY = 4.186e6
"""[J/(m3.K)]"""
JOULE_PER_KWH = 3.6e6
HOURS_PER_DAY = 24


class SystemKind(str, Enum):
    heat_pump = "heat_pump"
    boiler = "boiler"
    chiller = "chiller"
    dhw_tank = "dhw_tank"
    battery = "battery"
    ev_charger = "ev_charger"
    photovoltaic = "photovoltaic"


class BaseSystem(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")

    name: str = Field(description="Modelica identifier prefixing the symbols of the system")

    _name_validator = field_validator("name")(validate_identifier)

    @property
    def prefix(self) -> str:
        return f"{self.name}_"

    @property
    def state_names(self) -> list[str]:
        return []

    def modelica_parameters(self) -> list[ModelicaParameter]:
        """Parameters declared in ``building_mpc`` (without the prefix)."""
        return modelica_parameters_of(self)


class HeatPump(BaseSystem):
    """Modulating heat pump heating one or several zones, with the electrical power as control input."""

    kind: Literal[SystemKind.heat_pump] = SystemKind.heat_pump
    zones: list[str] = Field(default_factory=list, description="Zones heated by the heat pump")
    cop_nominal: float = rc_field(
        "cop0", "1", "COP at the rating point A7/W35 (7 degC outdoor, 35 degC supply)", 4.5, gt=0
    )
    cop_outdoor_slope: float = rc_field("copA", "1/K", "COP change per kelvin of outdoor temperature", 0.11)
    cop_supply_slope: float = rc_field("copS", "1/K", "COP change per kelvin of supply temperature", -0.075)
    cop_cross_term: float = rc_field(
        "copX", "1/K2", "COP cross term (outdoor deviation times supply deviation)", -0.0025
    )
    supply_temperature: float = rc_field(
        "TSup", "K", "Design supply temperature, used for the zones without emitter state", 308.15, gt=0
    )
    supply_offset: float = rc_field("dTSup", "K", "Supply temperature above the emitter temperature state", 5.0, ge=0)
    max_electrical_power: float = rc_field("PelMax", "W", "Maximum electrical power of the compressor", 2000.0, gt=0)

    def cop_expression(self, outdoor: str, supply: str) -> str:
        """Modelica expression of the COP for the given outdoor and supply temperature expressions."""
        p = self.prefix
        return (
            f"({p}cop0 + {p}copA*({outdoor} - {COP_REFERENCE_OUTDOOR}) + {p}copS*({supply} - {COP_REFERENCE_SUPPLY})"
            f" + {p}copX*({outdoor} - {COP_REFERENCE_OUTDOOR})*({supply} - {COP_REFERENCE_SUPPLY}))"
        )

    def cop(self, outdoor: float, supply: float) -> float:
        """Numerical COP [-] at an outdoor and a supply temperature [K] (same polynomial as the model)."""
        outdoor_deviation = outdoor - COP_REFERENCE_OUTDOOR
        supply_deviation = supply - COP_REFERENCE_SUPPLY
        return (
            self.cop_nominal
            + self.cop_outdoor_slope * outdoor_deviation
            + self.cop_supply_slope * supply_deviation
            + self.cop_cross_term * outdoor_deviation * supply_deviation
        )


class GasBoiler(BaseSystem):
    """Boiler: the zones keep their thermal control input, the fuel is costed with the efficiency."""

    kind: Literal[SystemKind.boiler] = SystemKind.boiler
    zones: list[str] = Field(default_factory=list, description="Zones heated by the boiler")
    efficiency: float = rc_field("eta", "1", "Heat delivered per unit of fuel energy", 0.9, gt=0, le=1.2)
    max_heating_power: float = rc_field("QMax", "W", "Nominal heating power", 20000.0, gt=0)


class Chiller(BaseSystem):
    """Modulating chiller cooling one or several zones, with the electrical power as control input."""

    kind: Literal[SystemKind.chiller] = SystemKind.chiller
    zones: list[str] = Field(default_factory=list, description="Zones cooled by the chiller")
    eer_nominal: float = rc_field("eer0", "1", "EER at 35 degC outdoor temperature", 3.5, gt=0)
    eer_outdoor_slope: float = rc_field("eerA", "1/K", "EER change per kelvin of outdoor temperature", -0.06)
    max_electrical_power: float = rc_field("PelMax", "W", "Maximum electrical power of the compressor", 2000.0, gt=0)

    def eer_expression(self, outdoor: str) -> str:
        p = self.prefix
        return f"({p}eer0 + {p}eerA*({outdoor} - {EER_REFERENCE_OUTDOOR}))"

    def eer(self, outdoor: float) -> float:
        return self.eer_nominal + self.eer_outdoor_slope * (outdoor - EER_REFERENCE_OUTDOOR)


DEFAULT_DRAW_OFF_FRACTIONS = (
    0.00, 0.00, 0.00, 0.00, 0.00, 0.02, 0.08, 0.12, 0.10, 0.05, 0.04, 0.04,
    0.04, 0.03, 0.03, 0.03, 0.04, 0.05, 0.08, 0.10, 0.08, 0.05, 0.02, 0.00,
)  # fmt: skip
"""Share of the daily domestic hot water draw-off per hour of the day (morning and evening peaks)."""


class DrawOffProfile(BaseModel):
    """Daily domestic hot water demand as a tap profile."""

    model_config = ConfigDict(frozen=True)

    daily_energy_kwh: float = Field(8.0, ge=0, description="Heat drawn per day [kWh]")
    hourly_fractions: tuple[float, ...] = Field(
        DEFAULT_DRAW_OFF_FRACTIONS, description="Share of the daily energy drawn in each hour of the day"
    )

    @field_validator("hourly_fractions")
    @classmethod
    def _normalise(cls, fractions: tuple[float, ...]) -> tuple[float, ...]:
        if len(fractions) != HOURS_PER_DAY:
            raise ValueError("hourly_fractions needs one value per hour of the day (24).")
        if any(fraction < 0 for fraction in fractions):
            raise ValueError("hourly_fractions must be non-negative.")
        total = sum(fractions)
        if total <= 0:
            raise ValueError("hourly_fractions must not all be zero.")
        return tuple(fraction / total for fraction in fractions)

    def hourly_power(self) -> list[float]:
        """Draw-off heat flow [W] for each hour of the day."""
        return [fraction * self.daily_energy_kwh * 1000 for fraction in self.hourly_fractions]


class DHWTank(BaseSystem):
    """Lumped domestic hot water tank: one temperature state, heated by a heat pump or a resistance."""

    kind: Literal[SystemKind.dhw_tank] = SystemKind.dhw_tank
    zone: str = Field(description="Zone receiving the standing losses of the tank")
    heat_pump: str | None = Field(None, description="Heat pump feeding the tank; None for an electric resistance")
    capacitance: float = rc_field(
        "C", "J/K", "Heat capacity of the stored water", 0.2 * WATER_VOLUMETRIC_HEAT_CAPACITY, gt=0
    )
    loss_coefficient: float = rc_field("UA", "W/K", "Standing loss coefficient of the tank", 2.0, ge=0)
    supply_offset: float = rc_field("dTSup", "K", "Heat pump supply temperature above the tank temperature", 5.0, ge=0)
    temperature_initial: float = Field(328.15, gt=0, description="Initial tank temperature [K]")
    set_temperature: float = Field(328.15, gt=0, description="Upper comfort bound of the tank temperature [K]")
    min_temperature: float = Field(318.15, gt=0, description="Lower comfort bound of the tank temperature [K]")
    draw_off: DrawOffProfile = Field(default_factory=DrawOffProfile)

    @classmethod
    def from_volume(cls, volume: float, **values: object) -> "DHWTank":
        """Tank of ``volume`` m3 of water."""
        return cls(capacitance=volume * WATER_VOLUMETRIC_HEAT_CAPACITY, **values)

    @property
    def state_names(self) -> list[str]:
        return [f"{self.prefix}T"]


class Battery(BaseSystem):
    """Stationary battery: energy state, charging and discharging power inputs."""

    kind: Literal[SystemKind.battery] = SystemKind.battery
    capacity: float = rc_field("EMax", "J", "Usable energy capacity", 10 * JOULE_PER_KWH, gt=0)
    max_charge_power: float = rc_field("PChaMax", "W", "Maximum charging power", 5000.0, gt=0)
    max_discharge_power: float = rc_field("PDisMax", "W", "Maximum discharging power", 5000.0, gt=0)
    charge_efficiency: float = rc_field("etaCha", "1", "Charging efficiency", 0.95, gt=0, le=1)
    discharge_efficiency: float = rc_field("etaDis", "1", "Discharging efficiency", 0.95, gt=0, le=1)
    min_soc: float = Field(0.1, ge=0, le=1, description="Lower bound of the state of charge")
    max_soc: float = Field(0.9, ge=0, le=1, description="Upper bound of the state of charge")
    initial_soc: float = Field(0.5, ge=0, le=1, description="Initial state of charge")

    @property
    def state_names(self) -> list[str]:
        return [f"{self.prefix}E"]

    @property
    def initial_energy(self) -> float:
        return self.initial_soc * self.capacity


class EVSessions(BaseModel):
    """Daily presence of the electric vehicle at the charger."""

    model_config = ConfigDict(frozen=True)

    arrival_hour: float = Field(18.0, ge=0, le=24, description="Hour of the day the vehicle plugs in")
    departure_hour: float = Field(7.0, ge=0, le=24, description="Hour of the day the vehicle leaves")
    energy_per_day_kwh: float = Field(10.0, ge=0, description="Energy consumed by driving per day [kWh]")
    target_soc: float = Field(0.8, ge=0, le=1, description="State of charge required at departure")
    weekend_present: bool = Field(True, description="Vehicle stays plugged in during the weekend")

    def away(self, hour: float) -> bool:
        """Whether the vehicle is away at ``hour`` of the day (between departure and arrival)."""
        if self.departure_hour <= self.arrival_hour:
            return self.departure_hour <= hour < self.arrival_hour
        return hour >= self.departure_hour or hour < self.arrival_hour

    @property
    def hours_away(self) -> float:
        return (self.arrival_hour - self.departure_hour) % HOURS_PER_DAY or float(HOURS_PER_DAY)

    def hourly_driving_power(self) -> list[float]:
        """Discharge by driving [W] for each hour of the day."""
        power = self.energy_per_day_kwh * 1000 / self.hours_away
        return [power if self.away(hour + 0.5) else 0.0 for hour in range(HOURS_PER_DAY)]


class EVCharger(BaseSystem):
    """Electric vehicle charger: the vehicle battery is an energy state while plugged in."""

    kind: Literal[SystemKind.ev_charger] = SystemKind.ev_charger
    capacity: float = rc_field("EMax", "J", "Battery capacity of the vehicle", 60 * JOULE_PER_KWH, gt=0)
    max_charge_power: float = rc_field("PChaMax", "W", "Maximum charging power", 7400.0, gt=0)
    charge_efficiency: float = rc_field("etaCha", "1", "Charging efficiency", 0.9, gt=0, le=1)
    initial_soc: float = Field(0.5, ge=0, le=1, description="Initial state of charge")
    sessions: EVSessions = Field(default_factory=EVSessions)

    @property
    def state_names(self) -> list[str]:
        return [f"{self.prefix}E"]

    @property
    def initial_energy(self) -> float:
        return self.initial_soc * self.capacity


class Photovoltaic(BaseSystem):
    """PV array: generated power as a disturbance input of the model."""

    kind: Literal[SystemKind.photovoltaic] = SystemKind.photovoltaic
    area: float = Field(20.0, gt=0, description="Module area [m2]")
    efficiency: float = Field(0.18, gt=0, le=1, description="Module and inverter efficiency at the array level")
    azimuth: float = Field(0.0, description="Surface azimuth [deg], 0 for south, 90 for west")
    tilt: float = Field(35.0, ge=0, le=180, description="Surface tilt [deg], 0 for a roof, 90 for a wall")

    @property
    def peak_power(self) -> float:
        """Power at 1000 W/m2 [W]."""
        return self.area * self.efficiency * 1000


AnySystem = Annotated[
    HeatPump | GasBoiler | Chiller | DHWTank | Battery | EVCharger | Photovoltaic,
    Field(discriminator="kind"),
]

__all__ = [
    "AnySystem",
    "BaseSystem",
    "Battery",
    "Chiller",
    "DHWTank",
    "DrawOffProfile",
    "EVCharger",
    "EVSessions",
    "GasBoiler",
    "HeatPump",
    "Photovoltaic",
    "SystemKind",
]
