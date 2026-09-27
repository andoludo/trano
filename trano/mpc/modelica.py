"""Rendering of the ``mpc`` library models into Modelica.

The zone equations live in ``templates/mpc.jinja2``. They are rendered as:

* the ``Trano.MPC.Zones`` library embedded in the ``Trano`` package of every generated model,
* the flat multi-zone ``building_mpc`` model (CasADi/IPOPT compatible),
* the runnable ``building`` model generated with the ``mpc`` library, which connects the
  weather file, the solar irradiance, the occupancy and the external data to ``building_mpc``.
"""

from functools import cache
from pathlib import Path
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel

from trano.mpc.parameters import ZONE_PARAMETERS, ModelicaParameter, ModelicaState, RCModelType

if TYPE_CHECKING:
    from trano.elements.base import BaseElement
    from trano.elements.bus import DataBus
    from trano.elements.system import BaseOccupancy
    from trano.mpc.building import Orientation, RCBuilding, RCZone
    from trano.topology import Network

LIBRARY_TEMPERATURE_INITIAL = 294.15
LIBRARY_GROUND_TEMPERATURE = 283.15
LIBRARY_IRRADIANCE = "HSol"
PREFIX = "@"  # replaced by the zone prefix in the templates


class SolarTerm(BaseModel):
    window: str | None = None
    opaque: str | None = None


class ZoneView(BaseModel):
    """Flattened view of a zone consumed by the Jinja templates."""

    name: str
    prefix: str
    model_type: str
    title: str
    documentation: str
    temperature_initial: float
    parameters: list[ModelicaParameter]
    states: list[ModelicaState]
    couplings: list[dict[str, str]]
    solar: list[SolarTerm]


class Irradiance(BaseModel):
    name: str
    description: str


class BuildingView(BaseModel):
    name: str
    model_type: str
    ground_temperature: float
    zones: list[ZoneView]
    couplings: list[dict[str, str | float]]
    irradiances: list[Irradiance]


class OrientationView(BaseModel):
    name: str
    irradiance: str
    azimuth: float
    tilt: float
    description: str


class OccupancyView(BaseModel):
    model: str
    parameters: str


class RunnableZoneView(BaseModel):
    name: str
    prefix: str
    floor_area: float
    occupancy: OccupancyView | None = None
    heating_column: int | None = None


class ExternalDataView(BaseModel):
    table: str
    columns: list[str]


class RunnableView(BaseModel):
    weather: str
    weather_name: str
    orientations: list[OrientationView]
    zones: list[RunnableZoneView]
    external_data: ExternalDataView | None = None


def _describe(orientation: "Orientation") -> str:
    return f"azimuth {orientation.azimuth:g} deg, tilt {orientation.tilt:g} deg"


def _solar_terms(zone: "RCZone") -> tuple[list[ModelicaParameter], list[SolarTerm]]:
    parameters, terms = [], []
    for aperture in zone.solar_apertures:
        orientation = aperture.orientation
        term = SolarTerm()
        if aperture.window > 0:
            name = f"gA_{orientation.name}"
            parameters.append(
                ModelicaParameter(
                    name=name,
                    value=aperture.window,
                    unit="m2",
                    description=f"Effective solar aperture of the windows, {_describe(orientation)}",
                )
            )
            term.window = f"{PREFIX}{name}*{orientation.irradiance}"
        if aperture.opaque > 0:
            name = f"aE_{orientation.name}"
            parameters.append(
                ModelicaParameter(
                    name=name,
                    value=aperture.opaque,
                    unit="m2",
                    description=f"Effective solar absorption area of the envelope, {_describe(orientation)}",
                )
            )
            term.opaque = f"{PREFIX}{name}*{orientation.irradiance}"
        terms.append(term)
    return parameters, terms


def _zone_view(building: "RCBuilding", zone: "RCZone") -> ZoneView:
    couplings = []
    for coupling in building.couplings:
        if zone.name in (coupling.zone_a, coupling.zone_b):
            neighbour = coupling.zone_b if coupling.zone_a == zone.name else coupling.zone_a
            couplings.append({"parameter": coupling.parameter, "neighbour": f"{neighbour}_"})
    solar_parameters, solar = _solar_terms(zone)
    return ZoneView(
        name=zone.name,
        prefix=zone.prefix,
        model_type=zone.parameters.model_type.value,
        title=zone.parameters.title,
        documentation=zone.parameters.documentation,
        temperature_initial=zone.temperature_initial,
        parameters=zone.parameters.modelica_parameters() + solar_parameters,
        states=list(zone.states),
        couplings=couplings,
        solar=solar,
    )


def _building_view(building: "RCBuilding") -> BuildingView:
    return BuildingView(
        name=building.name,
        model_type=building.model_type,
        ground_temperature=building.ground_temperature,
        zones=[_zone_view(building, zone) for zone in building.zones],
        couplings=[{**coupling.model_dump(), "parameter": coupling.parameter} for coupling in building.couplings],
        irradiances=[
            Irradiance(name=orientation.irradiance, description=f"Total solar irradiance, {_describe(orientation)}")
            for orientation in building.orientations
        ],
    )


@cache
def render_mpc_library() -> str:
    """The ``MPC`` package of the Trano library: one model per RC structure, for a reference zone."""
    from trano.elements.jinja import ENVIRONMENT
    from trano.mpc.estimation import reference_zone_parameters

    solar_parameters = [
        ModelicaParameter(name="gA", value=3.0, unit="m2", description="Effective solar aperture of the windows"),
        ModelicaParameter(
            name="aE", value=0.4, unit="m2", description="Effective solar absorption area of the envelope"
        ),
    ]
    library_zones = [
        ZoneView(
            name=model_type.value,
            prefix="",
            model_type=model_type.value,
            title=parameters_class.title,
            documentation=parameters_class.documentation,
            temperature_initial=LIBRARY_TEMPERATURE_INITIAL,
            parameters=reference_zone_parameters(model_type).modelica_parameters() + solar_parameters,
            states=list(parameters_class.states),
            couplings=[],
            solar=[SolarTerm(window=f"{PREFIX}gA*{LIBRARY_IRRADIANCE}", opaque=f"{PREFIX}aE*{LIBRARY_IRRADIANCE}")],
        )
        for model_type, parameters_class in ZONE_PARAMETERS.items()
    ]
    irradiances = [Irradiance(name=LIBRARY_IRRADIANCE, description="Total solar irradiance on the facade")]
    template = ENVIRONMENT.from_string(
        "{% import 'mpc.jinja2' as mpc %}{{ mpc.zones_library(library_zones, ground_temperature, irradiances) }}"
    )
    return template.render(
        library_zones=library_zones, ground_temperature=LIBRARY_GROUND_TEMPERATURE, irradiances=irradiances
    )


def _render(building: "RCBuilding", package_name: str, runnable: RunnableView | None = None) -> str:
    from trano.elements.jinja import ENVIRONMENT

    return ENVIRONMENT.get_template("mpc_base.jinja2").render(
        package_name=package_name, building=_building_view(building), runnable=runnable
    )


def render_building(building: "RCBuilding", package_name: str) -> str:
    """Modelica package with the Trano library (including ``Trano.MPC``) and the flat RC model."""
    return _render(building, package_name)


def _external_data(network: "Network", data_bus: "DataBus | None") -> ExternalDataView | None:
    from trano.elements.bus import transform_csv_to_table

    path: Path | None = (data_bus.external_data if data_bus else None) or network.external_data
    if path is None:
        return None
    data = transform_csv_to_table(path)
    if not data.data:
        return None
    return ExternalDataView(table=data.data, columns=data.columns)


def _column(external_data: ExternalDataView | None, name: str) -> int | None:
    if external_data is None or name not in external_data.columns:
        return None
    return external_data.columns.index(name) + 1


def _occupancy_view(
    occupancy: "BaseOccupancy", network: "Network", floor_area: float, external_data: ExternalDataView | None
) -> OccupancyView:
    parameters: dict[str, Any] = {
        key: value for key, value in occupancy.processed_parameters(network.library).items() if key != "data"
    }
    data_sources = getattr(occupancy.parameters, "data", None) or []
    if not data_sources:
        return OccupancyView(model="SimpleOccupancy", parameters=_render_parameters(parameters))
    variable = data_sources[0].variable
    column = _column(external_data, variable)
    if column is None:
        raise ValueError(
            f"Occupancy {occupancy.name} reads '{variable}' but the external data does not contain this column."
        )
    parameters["AFlo"] = floor_area
    # The measured CO2 concentration is bound to the (non-connector) input of the occupancy estimator.
    parameters["co2"] = f"(u=externalData.y[{column}])"
    return OccupancyView(model="OccupancyCo2", parameters=_render_parameters(parameters))


def _render_parameters(parameters: dict[str, Any]) -> str:
    return ", ".join(
        f"{key}{value}" if str(value).startswith("(") else f"{key}={value}"
        for key, value in parameters.items()
        if value is not None
    )


def _runnable_view(network: "Network", building: "RCBuilding", data_bus: "DataBus | None") -> RunnableView:
    from trano.elements.space import Space
    from trano.elements.system import Weather
    from trano.mpc.estimation import sanitize_name

    weather = next((node for node in network.graph.nodes if isinstance(node, Weather)), None)
    if weather is None or not weather.template:
        raise ValueError("The 'mpc' library requires a weather element with a template.")
    external_data = _external_data(network, data_bus)
    spaces = {sanitize_name(node.name): node for node in network.graph.nodes if isinstance(node, Space)}
    zones = []
    for zone in building.zones:
        space = spaces[zone.name]
        occupancy = (
            _occupancy_view(space.occupancy, network, zone.floor_area, external_data) if space.occupancy else None
        )
        zones.append(
            RunnableZoneView(
                name=zone.name,
                prefix=zone.prefix,
                floor_area=zone.floor_area,
                occupancy=occupancy,
                heating_column=_column(external_data, f"{zone.prefix}QHea"),
            )
        )
    return RunnableView(
        weather=_render_element(weather, network),
        weather_name=weather.name,
        orientations=[
            OrientationView(
                name=orientation.name,
                irradiance=orientation.irradiance,
                azimuth=orientation.azimuth_radians,
                tilt=orientation.tilt_radians,
                description=_describe(orientation),
            )
            for orientation in building.orientations
        ],
        zones=zones,
        external_data=external_data,
    )


def _render_element(element: "BaseElement", network: "Network") -> str:
    """Declaration of an element with its template of the ``mpc`` library (same as the other libraries)."""
    from trano.elements.jinja import compile_template

    template = compile_template("{% import 'macros.jinja2' as macros %}" + (element.template or ""))
    declaration = template.render(
        element=element,
        package_name=network.name,
        library_name=network.library.base_library(),
        parameters=element.processed_parameters(network.library),
    )
    return " ".join(declaration.split()) + ";"


def render_network(network: "Network", data_bus: "DataBus | None" = None) -> str:
    """Model of a network generated with the ``mpc`` library: flat RC model and runnable model."""
    from trano.mpc.estimation import rc_building_from_network

    model_type = network.library.rc_model_type or RCModelType.r3c2
    building = rc_building_from_network(network, model_type=model_type)
    return _render(building, network.name, _runnable_view(network, building, data_bus))
