"""Machine-readable interface of a model generated with the ``mpc`` library.

An MPC runtime (CasADi + IPOPT, ...) only needs two artefacts to use a Trano building:

* the Modelica package, whose ``<package>.building_mpc`` model is translated into a CasADi ODE
  ``xdot = rhs(t, x, u, p)`` (e.g. with rumoca),
* this interface (``<package>.mpc.json``), which describes, **in the order of the CasADi
  vectors** ``x``, ``u`` and ``p``, every state, input and parameter, which inputs are control
  inputs, which energy carrier they draw, where each disturbance comes from (weather file,
  solar irradiance on an orientation, occupancy, draw-off profile, driving, PV generation), so
  that the runtime can build its forecasts, and the energy systems (``systems``) with the
  bounds that are not part of the ODE: compressor limits shared by several inputs, state of
  charge bounds, tank temperature bounds.

Version 2 adds the systems; version 1 files (envelope only) are still read.
"""

from enum import Enum
from pathlib import Path
from typing import Annotated, Final, Literal

from pydantic import BaseModel, ConfigDict, Field

INTERFACE_VERSION: Final = "2"  # bump together with the ``version`` literal


class InputRole(str, Enum):
    control = "control"
    disturbance = "disturbance"


Carrier = Literal["heat", "electricity", "gas"]
"""What a control input draws: thermal power (ideal heating), electricity or fuel."""


class _Frozen(BaseModel):
    model_config = ConfigDict(frozen=True)


class OutdoorTemperatureSource(_Frozen):
    kind: Literal["weather"] = "weather"
    variable: str = Field("TDryBul", description="Variable of the weather data (Buildings weather bus)")
    unit: str = "K"


class IrradianceSource(_Frozen):
    """Total (direct + diffuse) solar irradiance on a tilted surface."""

    kind: Literal["irradiance"] = "irradiance"
    azimuth: float = Field(description="Surface azimuth [deg], 0 for south, 90 for west")
    tilt: float = Field(description="Surface tilt [deg], 0 for a roof, 90 for a wall")


class OccupancySource(_Frozen):
    """Sensible internal gains ``floor_area * (radiant + convective)`` of a Trano occupancy model."""

    kind: Literal["occupancy"] = "occupancy"
    zone: str
    floor_area: float
    model: str | None = Field(None, description="Trano.Occupancy model, None when the zone has no occupancy")
    parameters: dict[str, str | float] = Field(default_factory=dict)
    data_column: str | None = Field(None, description="External data column read by the occupancy model")


class PhotovoltaicSource(_Frozen):
    """Generated power ``area * efficiency * irradiance`` on the panel orientation [W]."""

    kind: Literal["photovoltaic"] = "photovoltaic"
    azimuth: float = Field(description="Surface azimuth [deg], 0 for south, 90 for west")
    tilt: float = Field(description="Surface tilt [deg], 0 for a roof, 90 for a wall")
    area: float = Field(description="Module area [m2]")
    efficiency: float = Field(description="Module and inverter efficiency")


class DrawOffSource(_Frozen):
    """Domestic hot water draw-off heat flow [W] from a daily tap profile."""

    kind: Literal["draw_off"] = "draw_off"
    daily_energy_kwh: float
    hourly_fractions: list[float] = Field(description="Share of the daily energy drawn in each hour of the day")


class DrivingSource(_Frozen):
    """Discharge of the vehicle battery by driving [W] while away from the charger."""

    kind: Literal["ev_driving"] = "ev_driving"
    arrival_hour: float
    departure_hour: float
    energy_per_day_kwh: float
    weekend_present: bool = True


DisturbanceSource = Annotated[
    OutdoorTemperatureSource | IrradianceSource | OccupancySource | PhotovoltaicSource | DrawOffSource | DrivingSource,
    Field(discriminator="kind"),
]


class StateSpec(_Frozen):
    name: str
    zone: str | None = Field(None, description="Zone of a thermal state, None for a system state")
    node: str = Field(
        description="RC node: Ti (indoor air), Te (envelope), Th (emitter), Tm (mass), T (tank), E (energy)"
    )
    unit: str = "K"
    initial_value: float
    description: str
    system: str | None = Field(None, description="System owning the state (tank, battery, vehicle)")


class InputSpec(_Frozen):
    name: str
    role: InputRole
    unit: str
    description: str
    zone: str | None = None
    source: DisturbanceSource | None = Field(None, description="Origin of a disturbance, None for controls")
    data_column: str | None = Field(None, description="External data column replaying this input, if any")
    carrier: Carrier = Field("heat", description="Energy carrier drawn by a control input")
    system: str | None = Field(None, description="System the input belongs to, if any")


class ParameterSpec(_Frozen):
    name: str
    value: float
    unit: str
    description: str
    zone: str | None = None
    system: str | None = None


class ZoneSpec(_Frozen):
    name: str
    model_type: str
    indoor_temperature: str = Field(description="State of the indoor air temperature (comfort variable)")
    heating_input: str = Field(
        description="Control input heating the zone (thermal power or heat pump electrical power)"
    )
    internal_gains_input: str
    floor_area: float
    design_heating_power: float | None = Field(None, description="Estimated design heating power [W]")
    heating_carrier: Carrier = "heat"
    heating_system: str | None = Field(None, description="Heat pump or boiler heating the zone, if any")
    cooling_input: str | None = Field(None, description="Electrical power of the chiller for the zone, if any")
    cooling_system: str | None = None


class _SystemSpec(_Frozen):
    name: str
    inputs: list[str] = Field(default_factory=list, description="Control inputs of the system")
    states: list[str] = Field(default_factory=list)


class HeatPumpSpec(_SystemSpec):
    kind: Literal["heat_pump"] = "heat_pump"
    zones: list[str]
    max_electrical_power: float = Field(description="Bound on the sum of the electrical inputs [W]")
    cop_nominal: float
    cop_outdoor_slope: float
    cop_supply_slope: float
    cop_cross_term: float
    supply_temperature: float
    supply_offset: float
    tank_inputs: list[str] = Field(default_factory=list, description="Tank inputs sharing the compressor limit")


class BoilerSpec(_SystemSpec):
    kind: Literal["boiler"] = "boiler"
    zones: list[str]
    efficiency: float
    max_heating_power: float
    carrier: Carrier = "gas"


class ChillerSpec(_SystemSpec):
    kind: Literal["chiller"] = "chiller"
    zones: list[str]
    max_electrical_power: float
    eer_nominal: float
    eer_outdoor_slope: float


class StorageTankSpec(_SystemSpec):
    kind: Literal["dhw_tank"] = "dhw_tank"
    zone: str
    heat_pump: str | None
    state: str
    heating_input: str
    draw_off_input: str
    capacitance: float
    min_temperature: float
    set_temperature: float
    carrier: Carrier = "electricity"


class BatterySpec(_SystemSpec):
    kind: Literal["battery"] = "battery"
    state: str
    charge_input: str
    discharge_input: str
    capacity: float
    max_charge_power: float
    max_discharge_power: float
    charge_efficiency: float
    discharge_efficiency: float
    min_soc: float
    max_soc: float


class EVChargerSpec(_SystemSpec):
    kind: Literal["ev_charger"] = "ev_charger"
    state: str
    charge_input: str
    driving_input: str
    capacity: float
    max_charge_power: float
    charge_efficiency: float
    arrival_hour: float
    departure_hour: float
    target_soc: float
    energy_per_day_kwh: float
    weekend_present: bool


class PhotovoltaicSpec(_SystemSpec):
    kind: Literal["photovoltaic"] = "photovoltaic"
    generation_input: str
    area: float
    efficiency: float
    azimuth: float
    tilt: float
    peak_power: float


AnySystemSpec = Annotated[
    HeatPumpSpec | BoilerSpec | ChillerSpec | StorageTankSpec | BatterySpec | EVChargerSpec | PhotovoltaicSpec,
    Field(discriminator="kind"),
]


class MPCModelInterface(_Frozen):
    """Description of ``<package>.building_mpc`` for an MPC runtime."""

    version: Literal["1", "2"] = INTERFACE_VERSION
    package: str
    model: str = Field(description="CasADi-compatible RC model, e.g. 'house.building_mpc'")
    runnable_model: str | None = Field(None, description="Runnable model wrapping 'model', e.g. 'house.building'")
    weather_file: str | None = Field(None, description="Weather file of the runnable model")
    external_data_columns: list[str] = Field(default_factory=list)
    states: list[StateSpec]
    inputs: list[InputSpec]
    parameters: list[ParameterSpec]
    zones: list[ZoneSpec]
    systems: list[AnySystemSpec] = Field(default_factory=list)

    @property
    def state_names(self) -> list[str]:
        return [state.name for state in self.states]

    @property
    def input_names(self) -> list[str]:
        return [signal.name for signal in self.inputs]

    @property
    def parameter_names(self) -> list[str]:
        return [parameter.name for parameter in self.parameters]

    @property
    def controls(self) -> list[InputSpec]:
        return [signal for signal in self.inputs if signal.role == InputRole.control]

    @property
    def disturbances(self) -> list[InputSpec]:
        return [signal for signal in self.inputs if signal.role == InputRole.disturbance]

    @property
    def electrical_controls(self) -> list[InputSpec]:
        return [signal for signal in self.controls if signal.carrier == "electricity"]

    @property
    def initial_state(self) -> list[float]:
        return [state.initial_value for state in self.states]

    @property
    def parameter_values(self) -> list[float]:
        return [parameter.value for parameter in self.parameters]

    def system(self, name: str) -> AnySystemSpec:
        for system in self.systems:
            if system.name == name:
                return system
        raise KeyError(f"No system named {name} in the interface.")

    def write(self, path: Path | str) -> Path:
        path = Path(path)
        path.write_text(self.model_dump_json(indent=2))
        return path

    @classmethod
    def read(cls, path: Path | str) -> "MPCModelInterface":
        return cls.model_validate_json(Path(path).read_text())
