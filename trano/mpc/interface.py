"""Machine-readable interface of a model generated with the ``mpc`` library.

An MPC runtime (CasADi + IPOPT, ...) only needs two artefacts to use a Trano building:

* the Modelica package, whose ``<package>.building_mpc`` model is translated into a CasADi ODE
  ``xdot = rhs(t, x, u, p)`` (e.g. with rumoca),
* this interface (``<package>.mpc.json``), which describes, **in the order of the CasADi
  vectors** ``x``, ``u`` and ``p``, every state, input and parameter, which inputs are control
  inputs and where each disturbance comes from (weather file, solar irradiance on an
  orientation, occupancy), so that the runtime can build its forecasts.
"""

from enum import Enum
from pathlib import Path
from typing import Annotated, Final, Literal

from pydantic import BaseModel, ConfigDict, Field

INTERFACE_VERSION: Final = "1"  # bump together with the ``version`` literal


class InputRole(str, Enum):
    control = "control"
    disturbance = "disturbance"


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


DisturbanceSource = Annotated[
    OutdoorTemperatureSource | IrradianceSource | OccupancySource, Field(discriminator="kind")
]


class StateSpec(_Frozen):
    name: str
    zone: str
    node: str = Field(description="RC node: Ti (indoor air), Te (envelope), Th (emitter) or Tm (mass)")
    unit: str = "K"
    initial_value: float
    description: str


class InputSpec(_Frozen):
    name: str
    role: InputRole
    unit: str
    description: str
    zone: str | None = None
    source: DisturbanceSource | None = Field(None, description="Origin of a disturbance, None for controls")
    data_column: str | None = Field(None, description="External data column replaying this input, if any")


class ParameterSpec(_Frozen):
    name: str
    value: float
    unit: str
    description: str
    zone: str | None = None


class ZoneSpec(_Frozen):
    name: str
    model_type: str
    indoor_temperature: str = Field(description="State of the indoor air temperature (comfort variable)")
    heating_input: str
    internal_gains_input: str
    floor_area: float
    design_heating_power: float | None = Field(None, description="Estimated design heating power [W]")


class MPCModelInterface(_Frozen):
    """Description of ``<package>.building_mpc`` for an MPC runtime."""

    version: Literal["1"] = "1"
    package: str
    model: str = Field(description="CasADi-compatible RC model, e.g. 'house.building_mpc'")
    runnable_model: str | None = Field(None, description="Runnable model wrapping 'model', e.g. 'house.building'")
    weather_file: str | None = Field(None, description="Weather file of the runnable model")
    external_data_columns: list[str] = Field(default_factory=list)
    states: list[StateSpec]
    inputs: list[InputSpec]
    parameters: list[ParameterSpec]
    zones: list[ZoneSpec]

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
    def initial_state(self) -> list[float]:
        return [state.initial_value for state in self.states]

    @property
    def parameter_values(self) -> list[float]:
        return [parameter.value for parameter in self.parameters]

    def write(self, path: Path | str) -> Path:
        path = Path(path)
        path.write_text(self.model_dump_json(indent=2))
        return path

    @classmethod
    def read(cls, path: Path | str) -> "MPCModelInterface":
        return cls.model_validate_json(Path(path).read_text())
