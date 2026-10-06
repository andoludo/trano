"""Typed parameter sets of the CasADi-compatible RC zone models.

Each zone model is a thermal resistance-capacitance network whose structure follows
well established grey-box building models used for model predictive control:

* ``R1C1`` - the single-state *Ti* model (Bacher & Madsen, 2011).
* ``R3C2`` - the two-state *TiTe* model (indoor air + building envelope), reported as
  the best accuracy/complexity trade-off in several grey-box comparisons.
* ``R4C3`` - the three-state *TiTeTh* model that adds the heat emitter dynamics
  (radiator, floor heating) of Bacher & Madsen (2011).
* ``ISO13790`` - the ISO 13790 simple hourly 5R1C network, enriched with a capacitive
  air node (5R2C) so that it becomes an explicit ODE.  The massless surface node is
  eliminated analytically, which keeps the model free of algebraic equations.

Field names are Pythonic; the Modelica/CasADi name of each parameter is stored as the
``symbol`` of the field.  The declaration order of the fields is the order of the
parameters in the generated Modelica model and therefore in the CasADi parameter vector.
"""

import re
from enum import Enum
from typing import Annotated, Any, ClassVar, Literal

from pydantic import BaseModel, ConfigDict, Field

MODELICA_IDENTIFIER = re.compile(r"^[A-Za-z_]\w*$")


def validate_identifier(value: str) -> str:
    if not MODELICA_IDENTIFIER.match(value):
        raise ValueError(f"'{value}' is not a valid Modelica identifier.")
    return value


class RCModelType(str, Enum):
    r1c1 = "R1C1"
    r3c2 = "R3C2"
    r4c3 = "R4C3"
    iso13790 = "ISO13790"


class ModelicaParameter(BaseModel):
    name: str
    value: float
    unit: str
    description: str


class ModelicaState(BaseModel):
    name: str
    description: str


def rc_field(symbol: str, unit: str, description: str, default: Any = ..., **constraints: float) -> Any:  # noqa: ANN401
    """Declare an RC parameter together with its Modelica symbol and unit."""
    return Field(default, description=description, json_schema_extra={"symbol": symbol, "unit": unit}, **constraints)  # type: ignore[call-overload]


def modelica_parameters_of(model: BaseModel) -> list[ModelicaParameter]:
    """The fields declared with :func:`rc_field`, in declaration order, as Modelica parameters."""
    parameters = []
    for name, field in type(model).model_fields.items():
        extra = field.json_schema_extra
        if not isinstance(extra, dict):
            continue
        parameters.append(
            ModelicaParameter(
                name=str(extra["symbol"]),
                value=getattr(model, name),
                unit=str(extra["unit"]),
                description=field.description or "",
            )
        )
    return parameters


class BaseZoneParameters(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")

    title: ClassVar[str]
    documentation: ClassVar[str]
    states: ClassVar[tuple[ModelicaState, ...]]

    model_type: RCModelType

    def modelica_parameters(self) -> list[ModelicaParameter]:
        return modelica_parameters_of(self)


_INDOOR = ModelicaState(name="Ti", description="Indoor air temperature")
_ENVELOPE = ModelicaState(name="Te", description="Building envelope and thermal mass temperature")


class R1C1Parameters(BaseZoneParameters):
    title: ClassVar[str] = "Ti model: one thermal capacity (1R1C + ground coupling)"
    documentation: ClassVar[str] = (
        "Single state model where the whole zone thermal mass is lumped with the indoor air. "
        "Reference: Bacher, P. and Madsen, H. (2011), Identifying suitable models for the heat "
        "dynamics of buildings, Energy and Buildings 43(7)."
    )
    states: ClassVar[tuple[ModelicaState, ...]] = (_INDOOR,)

    model_type: Literal[RCModelType.r1c1] = RCModelType.r1c1
    air_capacitance: float = rc_field("Ci", "J/K", "Lumped zone heat capacity", gt=0)
    indoor_outdoor_resistance: float = rc_field("Ria", "K/W", "Indoor to outdoor air thermal resistance", gt=0)
    ground_resistance: float = rc_field("Rig", "K/W", "Indoor to ground thermal resistance", gt=0)


class _EnvelopeZoneParameters(BaseZoneParameters):
    """Parameters shared by the models with an indoor air and an envelope node."""

    air_capacitance: float = rc_field("Ci", "J/K", "Indoor air and furniture heat capacity", gt=0)
    envelope_capacitance: float = rc_field("Ce", "J/K", "Building envelope and thermal mass heat capacity", gt=0)
    indoor_outdoor_resistance: float = rc_field(
        "Ria", "K/W", "Indoor to outdoor air thermal resistance (windows and ventilation)", gt=0
    )
    indoor_envelope_resistance: float = rc_field("Rie", "K/W", "Indoor air to envelope thermal resistance", gt=0)
    envelope_outdoor_resistance: float = rc_field("Rea", "K/W", "Envelope to outdoor air thermal resistance", gt=0)
    envelope_ground_resistance: float = rc_field("Reg", "K/W", "Envelope to ground thermal resistance", gt=0)


class R3C2Parameters(_EnvelopeZoneParameters):
    title: ClassVar[str] = "TiTe model: indoor air and envelope capacities (3R2C + ground coupling)"
    documentation: ClassVar[str] = (
        "Two states model: indoor air (Ti) and building envelope/thermal mass (Te). "
        "Windows and ventilation connect the indoor air directly to the outdoor. "
        "References: Bacher and Madsen (2011); Harb et al. (2016), Development and validation of "
        "grey-box models for forecasting the thermal response of occupied buildings, Energy and Buildings 117."
    )
    states: ClassVar[tuple[ModelicaState, ...]] = (_INDOOR, _ENVELOPE)

    model_type: Literal[RCModelType.r3c2] = RCModelType.r3c2


class R4C3Parameters(_EnvelopeZoneParameters):
    title: ClassVar[str] = "TiTeTh model: indoor air, envelope and heat emitter capacities (4R3C + ground coupling)"
    documentation: ClassVar[str] = (
        "Three states model: indoor air (Ti), envelope (Te) and heat emitter (Th). The heating power "
        "is injected into the emitter, which introduces the lag of radiators or floor heating. "
        "Reference: Bacher and Madsen (2011)."
    )
    states: ClassVar[tuple[ModelicaState, ...]] = (
        _INDOOR,
        _ENVELOPE,
        ModelicaState(name="Th", description="Heat emitter temperature"),
    )

    model_type: Literal[RCModelType.r4c3] = RCModelType.r4c3
    emitter_capacitance: float = rc_field("Ch", "J/K", "Heat emitter heat capacity", gt=0)
    emitter_resistance: float = rc_field("Rih", "K/W", "Heat emitter to indoor air thermal resistance", gt=0)


class ISO13790Parameters(BaseZoneParameters):
    title: ClassVar[str] = "ISO 13790 5R1C network with a capacitive air node (5R2C)"
    documentation: ClassVar[str] = (
        "ISO 13790:2008 simple hourly method. The air node receives the capacity of the indoor air and "
        "furniture, which turns the original differential-algebraic 5R1C network into an explicit ODE. "
        "The massless surface node is eliminated analytically."
    )
    states: ClassVar[tuple[ModelicaState, ...]] = (
        _INDOOR,
        ModelicaState(name="Tm", description="Building thermal mass temperature"),
    )

    model_type: Literal[RCModelType.iso13790] = RCModelType.iso13790
    air_capacitance: float = rc_field("Ci", "J/K", "Indoor air and furniture heat capacity", gt=0)
    mass_capacitance: float = rc_field("Cm", "J/K", "Building thermal mass heat capacity", gt=0)
    ventilation_conductance: float = rc_field("Hve", "W/K", "Ventilation and infiltration heat transfer", ge=0)
    window_conductance: float = rc_field("Hw", "W/K", "Windows heat transfer coefficient", ge=0)
    air_surface_conductance: float = rc_field("His", "W/K", "Air to surface node coupling", gt=0)
    surface_mass_conductance: float = rc_field("Hms", "W/K", "Surface to mass node coupling", gt=0)
    mass_outdoor_conductance: float = rc_field("Hem", "W/K", "Mass node to outdoor heat transfer", ge=0)
    ground_conductance: float = rc_field("Hg", "W/K", "Mass node to ground heat transfer", ge=0)
    convective_fraction: float = rc_field("fIa", "1", "Fraction of internal gains to the air node", ge=0, le=1)
    surface_fraction: float = rc_field("fSt", "1", "Fraction of radiant gains to the surface node", ge=0, le=1)
    mass_fraction: float = rc_field("fM", "1", "Fraction of radiant gains to the mass node", ge=0, le=1)


ZoneParameters = Annotated[
    R1C1Parameters | R3C2Parameters | R4C3Parameters | ISO13790Parameters,
    Field(discriminator="model_type"),
]

ZONE_PARAMETERS: dict[RCModelType, type[BaseZoneParameters]] = {
    RCModelType.r1c1: R1C1Parameters,
    RCModelType.r3c2: R3C2Parameters,
    RCModelType.r4c3: R4C3Parameters,
    RCModelType.iso13790: ISO13790Parameters,
}
