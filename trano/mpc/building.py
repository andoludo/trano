"""Multi-zone RC building model: the data rendered by the ``mpc`` library."""

import math
import re
from typing import Self

from pydantic import BaseModel, ConfigDict, Field, computed_field, field_validator, model_validator

from trano.mpc.parameters import ModelicaState, ZoneParameters

MODELICA_IDENTIFIER = re.compile(r"^[A-Za-z_]\w*$")


def _validate_identifier(value: str) -> str:
    if not MODELICA_IDENTIFIER.match(value):
        raise ValueError(f"'{value}' is not a valid Modelica identifier.")
    return value


def _format_angle(value: float) -> str:
    angle = f"{value:g}".replace("-", "m").replace(".", "p")
    return angle


class Orientation(BaseModel):
    """Orientation of a surface receiving solar irradiance (Buildings library convention)."""

    model_config = ConfigDict(frozen=True)

    azimuth: float = Field(description="Surface azimuth [deg], 0 for south, 90 for west")
    tilt: float = Field(ge=0, le=180, description="Surface tilt [deg], 0 for a roof, 90 for a wall")

    @property
    def name(self) -> str:
        return f"azi{_format_angle(self.azimuth)}_til{_format_angle(self.tilt)}"

    @property
    def irradiance(self) -> str:
        """Name of the building input with the total irradiance on this orientation."""
        return f"HSol_{self.name}"

    @property
    def azimuth_radians(self) -> float:
        return math.radians(self.azimuth)

    @property
    def tilt_radians(self) -> float:
        return math.radians(self.tilt)


class SolarAperture(BaseModel):
    """Solar heat gain coefficients of a zone for one orientation.

    The solar heat gains are ``window*H + opaque*H`` with ``H`` the total irradiance on the
    orientation [W/m2]: ``window`` goes to the indoor air, ``opaque`` to the envelope.
    """

    model_config = ConfigDict(frozen=True)

    orientation: Orientation
    window: float = Field(0.0, ge=0, description="Effective window aperture g.(1-Ff).A [m2]")
    opaque: float = Field(0.0, ge=0, description="Effective opaque absorption area alpha.Rse.U.A [m2]")


class RCZone(BaseModel):
    model_config = ConfigDict(frozen=True)

    name: str
    parameters: ZoneParameters
    solar_apertures: list[SolarAperture] = Field(default_factory=list)
    temperature_initial: float = Field(294.15, gt=0, description="Initial temperature of all the zone states [K]")
    floor_area: float = Field(100.0, gt=0, description="Floor area used to scale the occupancy gains [m2]")

    _name_validator = field_validator("name")(_validate_identifier)

    @property
    def prefix(self) -> str:
        return f"{self.name}_"

    @property
    def states(self) -> tuple[ModelicaState, ...]:
        return self.parameters.states

    @property
    def state_names(self) -> list[str]:
        return [f"{self.prefix}{state.name}" for state in self.states]


class ZoneCoupling(BaseModel):
    """Heat transfer through the internal walls between two zones."""

    model_config = ConfigDict(frozen=True)

    zone_a: str
    zone_b: str
    conductance: float = Field(gt=0, description="Heat transfer coefficient between the zones [W/K]")

    @property
    def parameter(self) -> str:
        return f"H_{self.zone_a}_{self.zone_b}"


class RCBuilding(BaseModel):
    """A multi-zone building made of CasADi-compatible RC zone models.

    Shared inputs are the outdoor temperature ``TOut`` [K] and the total irradiance on each
    orientation ``HSol_<orientation>`` [W/m2]. Each zone ``z`` has the internal gains
    ``z_QInt`` [W] as disturbance and the heating power ``z_QHea`` [W] as control input.
    """

    model_config = ConfigDict(frozen=True)

    name: str = "building_mpc"
    zones: list[RCZone] = Field(min_length=1)
    couplings: list[ZoneCoupling] = Field(default_factory=list)
    ground_temperature: float = Field(283.15, gt=0, description="Ground temperature below the slab [K]")

    _name_validator = field_validator("name")(_validate_identifier)

    @model_validator(mode="after")
    def _check_references(self) -> Self:
        names = [zone.name for zone in self.zones]
        if len(set(names)) != len(names):
            raise ValueError(f"Zone names must be unique, got {names}.")
        for coupling in self.couplings:
            if {coupling.zone_a, coupling.zone_b} - set(names):
                raise ValueError(f"Coupling {coupling.parameter} refers to an unknown zone.")
            if coupling.zone_a == coupling.zone_b:
                raise ValueError(f"Coupling {coupling.parameter} connects a zone to itself.")
        return self

    @computed_field  # type: ignore[prop-decorator]
    @property
    def model_type(self) -> str:
        return "/".join(sorted({zone.parameters.model_type.value for zone in self.zones}))

    @property
    def orientations(self) -> list[Orientation]:
        unique = {aperture.orientation for zone in self.zones for aperture in zone.solar_apertures}
        return sorted(unique, key=lambda orientation: (orientation.tilt, orientation.azimuth))

    @property
    def state_names(self) -> list[str]:
        return [name for zone in self.zones for name in zone.state_names]

    def to_modelica(self, package_name: str = "TranoRC") -> str:
        """Modelica package with the Trano library (including ``Trano.MPC``) and the flat RC model."""
        from trano.mpc.modelica import render_building

        _validate_identifier(package_name)
        return render_building(self, package_name)
