"""Physics-based initial RC parameters derived from a Trano building description.

The lumping rules follow the usual practice for reduced order building models
(ISO 13790, VDI 6007 / AixLib reduced order models, ISO 6946 surface resistances):

* opaque elements are split in two halves: the inner half connects the indoor air to the
  mass node (``Rie``) and the outer half connects the mass node to the outdoor (``Rea``)
  or to the ground (``Reg``);
* windows, ventilation and infiltration connect the indoor air directly to the outdoor;
* the indoor air capacity is multiplied by a factor accounting for the furniture;
* internal walls couple adjacent zones and half of their capacity is given to each zone.

These values are physically consistent *initial guesses*. For MPC in a real building they
should be calibrated on measurements: the parameters stay symbolic once ``building_mpc`` is
translated into CasADi, so a least-squares identification problem can be solved with IPOPT.
"""

from collections.abc import Iterable
from pathlib import Path
from typing import TYPE_CHECKING, cast

from pydantic import BaseModel, ConfigDict, Field

from trano.mpc.building import Orientation, RCBuilding, RCZone, SolarAperture, ZoneCoupling
from trano.mpc.parameters import (
    ISO13790Parameters,
    R1C1Parameters,
    R3C2Parameters,
    R4C3Parameters,
    RCModelType,
    ZoneParameters,
)

if TYPE_CHECKING:
    from trano.elements.construction import Construction, Glass
    from trano.elements.envelope import BaseInternalElement, BaseSimpleWall
    from trano.elements.space import Space
    from trano.topology import Network

AIR_DENSITY = 1.2  # kg/m3
AIR_HEAT_CAPACITY = 1005.0  # J/(kg.K)


class EstimationSettings(BaseModel):
    """Assumptions used to derive the RC parameters from the building description."""

    model_config = ConfigDict(frozen=True)

    air_capacity_multiplier: float = Field(5.0, gt=0, description="Furniture multiplier of the air heat capacity")
    air_change_rate: float = Field(0.5, ge=0, description="Default ventilation and infiltration rate [1/h]")
    internal_surface_resistance: float = Field(0.13, ge=0, description="ISO 6946 Rsi for walls [m2.K/W]")
    external_surface_resistance: float = Field(0.04, ge=0, description="ISO 6946 Rse [m2.K/W]")
    floor_surface_resistance: float = Field(0.17, ge=0, description="ISO 6946 Rsi for downward heat flow [m2.K/W]")
    ground_resistance: float = Field(0.5, ge=0, description="Equivalent resistance of the soil [m2.K/W]")
    ground_temperature: float = Field(283.15, gt=0, description="Ground temperature [K]")
    window_frame_fraction: float = Field(0.3, ge=0, lt=1, description="Frame area fraction of the windows")
    default_g_value: float = Field(0.6, ge=0, le=1, description="g-value when the glazing has no optical data")
    opaque_solar_absorptance: float = Field(0.6, ge=0, le=1, description="Solar absorptance of the facades")
    design_outdoor_temperature: float = Field(263.15, gt=0, description="Heating design outdoor temperature [K]")
    design_indoor_temperature: float = Field(293.15, gt=0, description="Heating design indoor temperature [K]")
    emitter_oversizing: float = Field(1.5, gt=0, description="Ratio between emitter and design heating power")
    emitter_temperature_difference: float = Field(
        40.0, gt=0, description="Emitter to indoor air temperature difference at nominal power [K]"
    )
    emitter_time_constant: float = Field(1200.0, gt=0, description="Heat emitter time constant [s]")
    iso_total_area_factor: float = Field(4.5, gt=0, description="ISO 13790 ratio At/Af")
    iso_mass_area_factor: float = Field(2.5, gt=0, description="ISO 13790 ratio Am/Af (medium class)")
    iso_air_surface_coefficient: float = Field(3.45, gt=0, description="ISO 13790 his [W/(m2.K)]")
    iso_surface_mass_coefficient: float = Field(9.1, gt=0, description="ISO 13790 hms [W/(m2.K)]")
    iso_convective_fraction: float = Field(0.5, ge=0, le=1, description="ISO 13790 internal gains to the air")


class ZoneEnvelope(BaseModel):
    """Aggregated heat transfer coefficients [W/K] and heat capacities [J/K] of a zone."""

    volume: float
    floor_area: float
    ventilation_conductance: float
    window_conductance: float = 0
    opaque_conductance: float = 0
    opaque_inner_conductance: float = 0
    opaque_outer_conductance: float = 0
    opaque_capacitance: float = 0
    floor_conductance: float = 0
    floor_inner_conductance: float = 0
    floor_outer_conductance: float = 0
    floor_capacitance: float = 0
    internal_inner_conductance: float = 0
    internal_capacitance: float = 0
    solar_apertures: list[SolarAperture] = Field(default_factory=list)

    @property
    def air_capacitance(self) -> float:
        return AIR_DENSITY * AIR_HEAT_CAPACITY * self.volume

    @property
    def mass_capacitance(self) -> float:
        return self.opaque_capacitance + self.floor_capacitance + self.internal_capacitance

    @property
    def mass_inner_conductance(self) -> float:
        return self.opaque_inner_conductance + self.floor_inner_conductance + self.internal_inner_conductance

    @property
    def outdoor_conductance(self) -> float:
        return self.window_conductance + self.ventilation_conductance + self.opaque_conductance

    @property
    def heat_loss_coefficient(self) -> float:
        return self.outdoor_conductance + self.floor_conductance


def _layers_resistance(construction: "Construction | Glass") -> float:
    return construction.total_thermal_resistance


def _capacitance_per_area(construction: "Construction | Glass") -> float:
    # ``total_thermal_capacitance`` is a pydantic computed field (a property at runtime).
    return cast(float, construction.total_thermal_capacitance)


def _g_value(glass: "Glass", settings: EstimationSettings) -> float:
    from trano.elements.construction import GlassMaterial

    transmittances = [
        layer.material.solar_transmittance[0]
        for layer in glass.layers
        if isinstance(layer.material, GlassMaterial) and layer.material.solar_transmittance
    ]
    if not transmittances:
        return settings.default_g_value
    g_value = 1.0
    for transmittance in transmittances:
        g_value *= transmittance
    return g_value


def _orientation(boundary: "BaseSimpleWall") -> Orientation:
    from trano.elements.types import TILT_MAPPING

    return Orientation(azimuth=float(boundary.azimuth), tilt=float(TILT_MAPPING[boundary.tilt.value]))


def zone_envelope(space: "Space", settings: EstimationSettings) -> ZoneEnvelope:
    """Aggregate the envelope of a Trano space into heat transfer coefficients and capacities."""
    from trano.elements.envelope import BaseFloorOnGround, BaseWindow

    volume = float(getattr(space.parameters, "volume", 0.0))
    air_change_rate = getattr(space.parameters, "ach", None) or settings.air_change_rate
    values: dict[str, float] = {
        "volume": volume,
        "floor_area": float(getattr(space.parameters, "floor_area", 0.0)),
        "ventilation_conductance": AIR_DENSITY * AIR_HEAT_CAPACITY * volume * air_change_rate / 3600,
    }

    def add(key: str, value: float) -> None:
        values[key] = values.get(key, 0.0) + value

    apertures: dict[Orientation, dict[str, float]] = {}

    def add_solar(boundary: "BaseSimpleWall", kind: str, value: float) -> None:
        aperture = apertures.setdefault(_orientation(boundary), {"window": 0.0, "opaque": 0.0})
        aperture[kind] += value

    rsi, rse = settings.internal_surface_resistance, settings.external_surface_resistance
    for boundary in space.external_boundaries:
        area = float(boundary.surface)
        construction = boundary.construction
        resistance = _layers_resistance(construction)
        if isinstance(boundary, BaseWindow):
            glazing_u_value = 1 / (resistance + rsi + rse)
            frame_u_value = getattr(construction, "u_value_frame", glazing_u_value)
            frame_fraction = settings.window_frame_fraction
            add("window_conductance", area * ((1 - frame_fraction) * glazing_u_value + frame_fraction * frame_u_value))
            g_value = _g_value(construction, settings)  # type: ignore[arg-type]
            add_solar(boundary, "window", area * (1 - frame_fraction) * g_value)
        elif isinstance(boundary, BaseFloorOnGround):
            rsi_floor, ground = settings.floor_surface_resistance, settings.ground_resistance
            add("floor_conductance", area / (resistance + rsi_floor + ground))
            add("floor_inner_conductance", area / (resistance / 2 + rsi_floor))
            add("floor_outer_conductance", area / (resistance / 2 + ground))
            add("floor_capacitance", area * _capacitance_per_area(construction))
        else:
            u_value = 1 / (resistance + rsi + rse)
            add("opaque_conductance", area * u_value)
            add("opaque_inner_conductance", area / (resistance / 2 + rsi))
            add("opaque_outer_conductance", area / (resistance / 2 + rse))
            add("opaque_capacitance", area * _capacitance_per_area(construction))
            add_solar(boundary, "opaque", settings.opaque_solar_absorptance * rse * u_value * area)
    for internal_element in _unique(space.internal_elements):
        area = float(internal_element.surface)
        resistance = _layers_resistance(internal_element.construction)
        add("internal_inner_conductance", area / (resistance / 2 + rsi))
        # Each side of the internal wall belongs to one of the two adjacent zones.
        add("internal_capacitance", area * _capacitance_per_area(internal_element.construction) / 2)
    solar_apertures = [
        SolarAperture(orientation=orientation, **aperture)
        for orientation, aperture in sorted(apertures.items(), key=lambda item: (item[0].tilt, item[0].azimuth))
    ]
    return ZoneEnvelope(**values, solar_apertures=solar_apertures)


def _unique(elements: Iterable["BaseInternalElement"]) -> list["BaseInternalElement"]:
    return list({element.name: element for element in elements}.values())


def design_heating_power(envelope: ZoneEnvelope, settings: EstimationSettings) -> float:
    """Oversized design heat load of the zone [W]."""
    design_temperature_difference = settings.design_indoor_temperature - settings.design_outdoor_temperature
    return settings.emitter_oversizing * envelope.heat_loss_coefficient * design_temperature_difference


def _emitter(envelope: ZoneEnvelope, settings: EstimationSettings) -> tuple[float, float]:
    nominal_power = design_heating_power(envelope, settings)
    resistance = settings.emitter_temperature_difference / nominal_power
    return settings.emitter_time_constant / resistance, resistance


def _safe_inverse(conductance: float, minimum: float = 1e-6) -> float:
    return 1 / max(conductance, minimum)


def estimate_zone_parameters(
    envelope: ZoneEnvelope,
    model_type: RCModelType,
    settings: EstimationSettings | None = None,
) -> ZoneParameters:
    """Derive the parameters of an RC zone model from its aggregated envelope."""
    settings = settings or EstimationSettings()
    air_capacitance = envelope.air_capacitance * settings.air_capacity_multiplier
    direct_conductance = envelope.window_conductance + envelope.ventilation_conductance
    if model_type == RCModelType.r1c1:
        return R1C1Parameters(
            air_capacitance=air_capacitance + envelope.mass_capacitance,
            indoor_outdoor_resistance=_safe_inverse(envelope.outdoor_conductance),
            ground_resistance=_safe_inverse(envelope.floor_conductance),
        )
    if model_type in (RCModelType.r3c2, RCModelType.r4c3):
        envelope_parameters = {
            "air_capacitance": air_capacitance,
            "envelope_capacitance": envelope.mass_capacitance,
            "indoor_outdoor_resistance": _safe_inverse(direct_conductance),
            "indoor_envelope_resistance": _safe_inverse(envelope.mass_inner_conductance),
            "envelope_outdoor_resistance": _safe_inverse(envelope.opaque_outer_conductance),
            "envelope_ground_resistance": _safe_inverse(envelope.floor_outer_conductance),
        }
        if model_type == RCModelType.r3c2:
            return R3C2Parameters(**envelope_parameters)
        emitter_capacitance, emitter_resistance = _emitter(envelope, settings)
        return R4C3Parameters(
            **envelope_parameters,
            emitter_capacitance=emitter_capacitance,
            emitter_resistance=emitter_resistance,
        )
    return _iso13790_parameters(envelope, air_capacitance, settings)


def _iso13790_parameters(
    envelope: ZoneEnvelope, air_capacitance: float, settings: EstimationSettings
) -> ISO13790Parameters:
    total_area = settings.iso_total_area_factor * envelope.floor_area
    mass_area = settings.iso_mass_area_factor * envelope.floor_area
    surface_mass_conductance = settings.iso_surface_mass_coefficient * mass_area
    # ISO 13790 (12.2.2): Hop = 1/(1/Hem + 1/Hms); the ground part is handled separately.
    opaque_conductance = min(envelope.opaque_conductance, 0.99 * surface_mass_conductance)
    mass_outdoor_conductance = 1 / (1 / max(opaque_conductance, 1e-6) - 1 / surface_mass_conductance)
    mass_fraction = mass_area / total_area
    surface_fraction = max(
        0.0,
        1 - mass_fraction - envelope.window_conductance / (settings.iso_surface_mass_coefficient * total_area),
    )
    return ISO13790Parameters(
        air_capacitance=air_capacitance,
        mass_capacitance=max(envelope.mass_capacitance, 1.0),
        ventilation_conductance=envelope.ventilation_conductance,
        window_conductance=envelope.window_conductance,
        air_surface_conductance=settings.iso_air_surface_coefficient * total_area,
        surface_mass_conductance=surface_mass_conductance,
        mass_outdoor_conductance=mass_outdoor_conductance,
        ground_conductance=envelope.floor_conductance,
        convective_fraction=settings.iso_convective_fraction,
        surface_fraction=surface_fraction,
        mass_fraction=mass_fraction,
    )


def reference_zone_parameters(model_type: RCModelType) -> ZoneParameters:
    """Parameters of a 100 m2 (250 m3) zone with 100 m2 of insulated facade and 10 m2 of windows."""
    reference = ZoneEnvelope(
        volume=250,
        floor_area=100,
        ventilation_conductance=AIR_DENSITY * AIR_HEAT_CAPACITY * 250 * 0.5 / 3600,
        window_conductance=10 * 1.4,
        opaque_conductance=100 * 0.3,
        opaque_inner_conductance=100 / (1.6 + 0.13),
        opaque_outer_conductance=100 / (1.6 + 0.04),
        opaque_capacitance=100 * 250_000,
        floor_conductance=100 * 0.35,
        floor_inner_conductance=100 / (1.2 + 0.17),
        floor_outer_conductance=100 / (1.2 + 0.5),
        floor_capacitance=100 * 400_000,
    )
    return estimate_zone_parameters(reference, model_type)


def sanitize_name(name: str) -> str:
    sanitized = "".join(character if character.isalnum() or character == "_" else "_" for character in name)
    return sanitized if sanitized[:1].isalpha() else f"z_{sanitized}"


def _couplings(spaces: list["Space"], settings: EstimationSettings) -> list[ZoneCoupling]:
    conductances: dict[tuple[str, str], float] = {}
    elements_per_space = {
        sanitize_name(space.name): {element.name for element in space.internal_elements} for space in spaces
    }
    for internal_element in _unique(element for space in spaces for element in space.internal_elements):
        adjacent = [name for name, elements in elements_per_space.items() if internal_element.name in elements]
        if len(adjacent) != 2:
            continue
        resistance = _layers_resistance(internal_element.construction) + 2 * settings.internal_surface_resistance
        key = (adjacent[0], adjacent[1])
        conductances[key] = conductances.get(key, 0.0) + float(internal_element.surface) / resistance
    return [ZoneCoupling(zone_a=a, zone_b=b, conductance=value) for (a, b), value in conductances.items()]


def rc_building_from_network(
    network: "Network",
    model_type: RCModelType | None = None,
    settings: EstimationSettings | None = None,
    name: str = "building_mpc",
) -> RCBuilding:
    """Create an RC building model from a Trano network (one RC zone per space).

    ``model_type`` defaults to the one of the network library (``R3C2`` if not set).
    """
    from trano.elements.space import Space

    model_type = model_type or network.library.rc_model_type or RCModelType.r3c2
    settings = settings or EstimationSettings()
    spaces = sorted((node for node in network.graph.nodes if isinstance(node, Space)), key=lambda space: space.name)
    if not spaces:
        raise ValueError("The network does not contain any space.")
    zones = []
    for space in spaces:
        envelope = zone_envelope(space, settings)
        zones.append(
            RCZone(
                name=sanitize_name(space.name),
                parameters=estimate_zone_parameters(envelope, model_type, settings),
                solar_apertures=envelope.solar_apertures,
                temperature_initial=getattr(space.parameters, "temperature_initial", None) or 294.15,
                floor_area=envelope.floor_area,
                design_heating_power=design_heating_power(envelope, settings),
            )
        )
    return RCBuilding(
        name=name,
        zones=zones,
        couplings=_couplings(spaces, settings),
        ground_temperature=settings.ground_temperature,
    )


def rc_building_from_yaml(
    model_path: Path | str,
    model_type: RCModelType | None = None,
    settings: EstimationSettings | None = None,
    name: str = "building_mpc",
) -> RCBuilding:
    """Create an RC building model from a Trano ``.yaml``/``.json`` building description."""
    from trano.data_models.conversion import convert_network
    from trano.elements.library.library import Library

    model_path = Path(model_path).resolve()
    network = convert_network(model_path.stem, model_path, library=Library.from_configuration("mpc"))
    return rc_building_from_network(network, model_type=model_type, settings=settings, name=name)
