"""Multi-zone RC building model and its Modelica representation."""

import re
from typing import TYPE_CHECKING, Self

from pydantic import BaseModel, ConfigDict, Field, computed_field, field_validator, model_validator

from trano.mpc.parameters import ModelicaState, ZoneParameters

if TYPE_CHECKING:
    from trano.mpc.casadi_model import CasadiRCModel

MODELICA_IDENTIFIER = re.compile(r"^[A-Za-z_]\w*$")


def _validate_identifier(value: str) -> str:
    if not MODELICA_IDENTIFIER.match(value):
        raise ValueError(f"'{value}' is not a valid Modelica identifier.")
    return value


class RCZone(BaseModel):
    model_config = ConfigDict(frozen=True)

    name: str
    parameters: ZoneParameters
    temperature_initial: float = Field(294.15, gt=0, description="Initial temperature of all the zone states [K]")

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

    @property
    def indoor_temperature(self) -> str:
        return f"{self.prefix}Ti"

    @property
    def heating_input(self) -> str:
        return f"{self.prefix}QHea"

    @property
    def internal_gains_input(self) -> str:
        return f"{self.prefix}QInt"


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

    Shared (building level) inputs are the outdoor temperature ``TOut`` [K] and the global
    horizontal irradiance ``HGlo`` [W/m2]. Each zone ``z`` has the internal gains ``z_QInt`` [W]
    as disturbance and the heating power ``z_QHea`` [W] as control input.
    """

    model_config = ConfigDict(frozen=True)

    name: str = "building"
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
    def state_names(self) -> list[str]:
        return [name for zone in self.zones for name in zone.state_names]

    def get_zone(self, name: str) -> RCZone:
        return next(zone for zone in self.zones if zone.name == name)

    def to_modelica(self, package_name: str = "TranoRC") -> str:
        """Modelica package with the Trano library (including ``Trano.MPC``) and the flat building model."""
        from trano.mpc.modelica import render_building

        _validate_identifier(package_name)
        return render_building(self, package_name)

    def to_casadi(self, package_name: str = "TranoRC") -> "CasadiRCModel":
        """Translate the generated Modelica model into a symbolic CasADi model."""
        from trano.mpc.casadi_model import CasadiRCModel

        return CasadiRCModel.from_modelica(
            self.to_modelica(package_name), model=f"{package_name}.{self.name}", building=self
        )
