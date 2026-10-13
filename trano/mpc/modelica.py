"""Rendering of the ``mpc`` library models into Modelica.

The zone equations live in ``templates/mpc.jinja2``. They are rendered as:

* the ``Trano.MPC.Zones`` library embedded in the ``Trano`` package of every generated model,
* the flat multi-zone ``building_mpc`` model (CasADi/IPOPT compatible),
* the runnable ``building`` model generated with the ``mpc`` library, which connects the
  weather file, the solar irradiance, the occupancy and the external data to ``building_mpc``.
"""

from functools import cache
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal

from pydantic import BaseModel, Field

from trano.mpc.interface import Carrier, InputRole
from trano.mpc.parameters import ZONE_PARAMETERS, ModelicaParameter, ModelicaState, RCModelType
from trano.mpc.systems import (
    AnySystem,
    Battery,
    Chiller,
    DHWTank,
    EVCharger,
    GasBoiler,
    HeatPump,
    Photovoltaic,
    SystemKind,
)

if TYPE_CHECKING:
    from trano.mpc.interface import MPCModelInterface
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


SourceKind = Literal["draw_off", "ev_driving", "photovoltaic"]


class InputView(BaseModel):
    """An input of ``building_mpc`` (full name, in declaration order)."""

    name: str
    unit: str
    description: str
    role: InputRole = InputRole.control
    carrier: Carrier = "heat"
    system: str | None = None
    source: SourceKind | None = None


class StateView(BaseModel):
    name: str
    unit: str
    start: float
    description: str


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
    inputs: list[InputView]
    heating: str = Field(description="Expression of the heat delivered to the zone by its heating input")
    direct: str = Field("", description="Terms added to the indoor air balance (cooling, tank losses)")


class SystemView(BaseModel):
    """Flattened view of a system: prefixed parameters, inputs, states and equations."""

    name: str
    prefix: str
    kind: str
    title: str
    parameters: list[ModelicaParameter]
    inputs: list[InputView]
    states: list[StateView]
    equations: list[str]


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
    systems: list[SystemView]


class OrientationView(BaseModel):
    name: str
    irradiance: str
    azimuth: float
    tilt: float
    description: str


class OccupancyView(BaseModel):
    model: str
    parameters: str
    values: dict[str, str | float] = {}
    data_column: str | None = None


class RunnableZoneView(BaseModel):
    name: str
    prefix: str
    floor_area: float
    occupancy: OccupancyView | None = None


class RunnableControlView(BaseModel):
    """A control input of the runnable model: a top-level input or a replayed data column."""

    name: str
    description: str
    column: int | None = None


class RunnableTableView(BaseModel):
    """A daily schedule (periodic table) feeding a disturbance input of ``building_mpc``."""

    name: str
    input: str
    rows: str
    description: str


class RunnablePVView(BaseModel):
    name: str
    input: str
    orientation: str
    gain: float


class ExternalDataView(BaseModel):
    table: str
    columns: list[str]


class RunnableView(BaseModel):
    weather: str
    weather_name: str
    weather_file: str | None = None
    orientations: list[OrientationView]
    zones: list[RunnableZoneView]
    external_data: ExternalDataView | None = None
    controls: list[RunnableControlView] = []
    tables: list[RunnableTableView] = []
    photovoltaics: list[RunnablePVView] = []


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


def _supply_temperature(zone: "RCZone", heat_pump: HeatPump) -> str:
    """Supply temperature seen by the heat pump: the emitter state plus an offset, or the design value."""
    if zone.parameters.model_type == RCModelType.r4c3:
        return f"({zone.prefix}Th + {heat_pump.prefix}dTSup)"
    return f"{heat_pump.prefix}TSup"


def _zone_inputs(building: "RCBuilding", zone: "RCZone") -> tuple[list[InputView], str, str]:
    """Inputs of the zone, the expression of its heating and the extra terms of its air balance."""
    p = zone.prefix
    inputs = [
        InputView(
            name=f"{p}QInt",
            unit="W",
            description="Internal heat gains (occupants, appliances, lighting)",
            role=InputRole.disturbance,
        )
    ]
    heating_system = building.heating_system(zone.name)
    if isinstance(heating_system, HeatPump):
        inputs.append(
            InputView(
                name=f"{p}PHea",
                unit="W",
                description=f"Electrical power of heat pump {heating_system.name} for the zone, control input",
                carrier="electricity",
                system=heating_system.name,
            )
        )
        heating = f"{p}PHea*{heating_system.cop_expression('TOut', _supply_temperature(zone, heating_system))}"
    else:
        carrier: Carrier = "gas" if isinstance(heating_system, GasBoiler) else "heat"
        inputs.append(
            InputView(
                name=f"{p}QHea",
                unit="W",
                description="Heating (> 0) or cooling (< 0) power, control input",
                carrier=carrier,
                system=heating_system.name if heating_system else None,
            )
        )
        heating = f"{p}QHea"
    direct = ""
    chiller = building.cooling_system(zone.name)
    if chiller is not None:
        inputs.append(
            InputView(
                name=f"{p}PCoo",
                unit="W",
                description=f"Electrical power of chiller {chiller.name} for the zone, control input",
                carrier="electricity",
                system=chiller.name,
            )
        )
        direct += f" - {p}PCoo*{chiller.eer_expression('TOut')}"
    for tank in building.tanks_in(zone.name):
        direct += f" + {tank.prefix}UA*({tank.prefix}T - {p}Ti)"
    return inputs, heating, direct


def _zone_view(building: "RCBuilding", zone: "RCZone") -> ZoneView:
    couplings = []
    for coupling in building.couplings:
        if zone.name in (coupling.zone_a, coupling.zone_b):
            neighbour = coupling.zone_b if coupling.zone_a == zone.name else coupling.zone_a
            couplings.append({"parameter": coupling.parameter, "neighbour": f"{neighbour}_"})
    solar_parameters, solar = _solar_terms(zone)
    inputs, heating, direct = _zone_inputs(building, zone)
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
        inputs=inputs,
        heating=heating,
        direct=direct,
    )


SYSTEM_TITLES = {
    SystemKind.heat_pump: "Heat pump (electrical power per zone, bilinear COP)",
    SystemKind.boiler: "Boiler (thermal power per zone, fuel costed with the efficiency)",
    SystemKind.chiller: "Chiller (electrical power per zone, linear EER)",
    SystemKind.dhw_tank: "Domestic hot water tank (lumped temperature)",
    SystemKind.battery: "Battery (energy state, charging and discharging power)",
    SystemKind.ev_charger: "Electric vehicle charger (vehicle energy state, driving discharge)",
    SystemKind.photovoltaic: "Photovoltaic array (generated power as disturbance)",
}


def _system_view(building: "RCBuilding", system: AnySystem) -> SystemView:
    p = system.prefix
    parameters = [
        parameter.model_copy(update={"name": f"{p}{parameter.name}"}) for parameter in system.modelica_parameters()
    ]
    inputs: list[InputView] = []
    states: list[StateView] = []
    equations: list[str] = []
    if isinstance(system, DHWTank):
        inputs = [
            InputView(
                name=f"{p}PHea",
                unit="W",
                description="Electrical power heating the tank, control input",
                carrier="electricity",
                system=system.name,
            ),
            InputView(
                name=f"{p}QDraw",
                unit="W",
                description="Heat drawn by the hot water taps",
                role=InputRole.disturbance,
                system=system.name,
                source="draw_off",
            ),
        ]
        states = [StateView(name=f"{p}T", unit="K", start=system.temperature_initial, description="Tank temperature")]
        if system.heat_pump is not None:
            heat_pump = building.heat_pump(system.heat_pump)
            heat = f"{p}PHea*{heat_pump.cop_expression('TOut', f'({p}T + {p}dTSup)')}"
        else:
            heat = f"{p}PHea"
        equations = [f"der({p}T) = ({heat} - {p}QDraw - {p}UA*({p}T - {system.zone}_Ti))/{p}C;"]
    elif isinstance(system, Battery):
        inputs = [
            InputView(
                name=f"{p}PCha",
                unit="W",
                description="Charging power, control input",
                carrier="electricity",
                system=system.name,
            ),
            InputView(
                name=f"{p}PDis",
                unit="W",
                description="Discharging power, control input",
                carrier="electricity",
                system=system.name,
            ),
        ]
        states = [StateView(name=f"{p}E", unit="J", start=system.initial_energy, description="Stored energy")]
        equations = [f"der({p}E) = {p}etaCha*{p}PCha - {p}PDis/{p}etaDis;"]
    elif isinstance(system, EVCharger):
        inputs = [
            InputView(
                name=f"{p}PCha",
                unit="W",
                description="Charging power, control input",
                carrier="electricity",
                system=system.name,
            ),
            InputView(
                name=f"{p}PDri",
                unit="W",
                description="Discharge of the vehicle battery by driving",
                role=InputRole.disturbance,
                system=system.name,
                source="ev_driving",
            ),
        ]
        states = [StateView(name=f"{p}E", unit="J", start=system.initial_energy, description="Vehicle battery energy")]
        equations = [f"der({p}E) = {p}etaCha*{p}PCha - {p}PDri;"]
    elif isinstance(system, Photovoltaic):
        inputs = [
            InputView(
                name=f"{p}P",
                unit="W",
                description="Generated electrical power",
                role=InputRole.disturbance,
                carrier="electricity",
                system=system.name,
                source="photovoltaic",
            )
        ]
    return SystemView(
        name=system.name,
        prefix=p,
        kind=system.kind.value,
        title=SYSTEM_TITLES[system.kind],
        parameters=parameters,
        inputs=inputs,
        states=states,
        equations=equations,
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
        systems=[_system_view(building, system) for system in building.systems],
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
            inputs=[
                InputView(
                    name="QInt",
                    unit="W",
                    description="Internal heat gains (occupants, appliances, lighting)",
                    role=InputRole.disturbance,
                ),
                InputView(name="QHea", unit="W", description="Heating (> 0) or cooling (< 0) power, control input"),
            ],
            heating="QHea",
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
        return OccupancyView(
            model="SimpleOccupancy", parameters=_render_parameters(parameters), values=dict(parameters)
        )
    variable = data_sources[0].variable
    column = _column(external_data, variable)
    if column is None:
        raise ValueError(
            f"Occupancy {occupancy.name} reads '{variable}' but the external data does not contain this column."
        )
    parameters["AFlo"] = floor_area
    values = dict(parameters)
    # The measured CO2 concentration is bound to the (non-connector) input of the occupancy estimator.
    parameters["co2"] = f"(u=externalData.y[{column}])"
    return OccupancyView(
        model="OccupancyCo2", parameters=_render_parameters(parameters), values=values, data_column=variable
    )


def _render_parameters(parameters: dict[str, Any]) -> str:
    return ", ".join(
        f"{key}{value}" if str(value).startswith("(") else f"{key}={value}"
        for key, value in parameters.items()
        if value is not None
    )


def _table_rows(hourly_values: list[float]) -> str:
    """Rows of a periodic daily ``CombiTimeTable`` held constant over each hour."""
    rows = [f"{hour * 3600}, {value:g}" for hour, value in enumerate(hourly_values)]
    rows.append(f"{24 * 3600}, {hourly_values[0]:g}")
    return "; ".join(rows)


def _runnable_systems(
    building: "RCBuilding", view: BuildingView, external_data: ExternalDataView | None
) -> tuple[list[RunnableControlView], list[RunnableTableView], list[RunnablePVView], list["Orientation"]]:
    tables, photovoltaics, orientations = [], [], []
    controls = [
        RunnableControlView(
            name=signal.name, description=signal.description, column=_column(external_data, signal.name)
        )
        for zone in view.zones
        for signal in zone.inputs
        if signal.role == InputRole.control
    ]
    systems = {system.name: system for system in building.systems}
    for system_view in view.systems:
        system = systems[system_view.name]
        for signal in system_view.inputs:
            if signal.role == InputRole.control:
                controls.append(
                    RunnableControlView(
                        name=signal.name, description=signal.description, column=_column(external_data, signal.name)
                    )
                )
            elif signal.source == "draw_off" and isinstance(system, DHWTank):
                tables.append(
                    RunnableTableView(
                        name=f"{system.prefix}drawOff",
                        input=signal.name,
                        rows=_table_rows(system.draw_off.hourly_power()),
                        description=f"Hot water draw-off of {system.name} [W], daily profile",
                    )
                )
            elif signal.source == "ev_driving" and isinstance(system, EVCharger):
                tables.append(
                    RunnableTableView(
                        name=f"{system.prefix}driving",
                        input=signal.name,
                        rows=_table_rows(system.sessions.hourly_driving_power()),
                        description=f"Driving discharge of {system.name} [W], daily profile",
                    )
                )
            elif signal.source == "photovoltaic" and isinstance(system, Photovoltaic):
                from trano.mpc.building import Orientation

                orientation = Orientation(azimuth=system.azimuth, tilt=system.tilt)
                if orientation not in orientations:
                    orientations.append(orientation)
                photovoltaics.append(
                    RunnablePVView(
                        name=f"{system.prefix}gain",
                        input=signal.name,
                        orientation=orientation.name,
                        gain=system.area * system.efficiency,
                    )
                )
    return controls, tables, photovoltaics, orientations


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
            RunnableZoneView(name=zone.name, prefix=zone.prefix, floor_area=zone.floor_area, occupancy=occupancy)
        )
    view = _building_view(building)
    controls, tables, photovoltaics, pv_orientations = _runnable_systems(building, view, external_data)
    orientations = list(building.orientations)
    orientations += [orientation for orientation in pv_orientations if orientation not in orientations]
    return RunnableView(
        weather=_render_element(weather, network),
        weather_name=weather.name,
        weather_file=_weather_file(weather, network),
        orientations=[
            OrientationView(
                name=orientation.name,
                irradiance=orientation.irradiance,
                azimuth=orientation.azimuth_radians,
                tilt=orientation.tilt_radians,
                description=_describe(orientation),
            )
            for orientation in orientations
        ],
        zones=zones,
        external_data=external_data,
        controls=controls,
        tables=tables,
        photovoltaics=photovoltaics,
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


def _weather_file(weather: "BaseElement", network: "Network") -> str | None:
    path = weather.processed_parameters(network.library).get("filNam")
    return str(path).strip('"') if path else None


def _prepare(network: "Network", data_bus: "DataBus | None") -> tuple["RCBuilding", RunnableView]:
    from trano.mpc.estimation import rc_building_from_network

    model_type = network.library.rc_model_type or RCModelType.r3c2
    building = rc_building_from_network(network, model_type=model_type)
    return building, _runnable_view(network, building, data_bus)


def render_network(network: "Network", data_bus: "DataBus | None" = None) -> str:
    """Model of a network generated with the ``mpc`` library: flat RC model and runnable model."""
    building, runnable = _prepare(network, data_bus)
    return _render(building, network.name, runnable)


def network_interface(network: "Network", data_bus: "DataBus | None" = None) -> "MPCModelInterface":
    """Interface of the model generated for a network with the ``mpc`` library."""
    building, runnable = _prepare(network, data_bus)
    return build_interface(building, network.name, runnable)


def build_interface(  # noqa: C901, PLR0912
    building: "RCBuilding", package_name: str, runnable: RunnableView | None = None
) -> "MPCModelInterface":
    """Describe ``building_mpc`` in the declaration order, i.e. the order of the CasADi vectors."""
    from trano.mpc.interface import (
        BatterySpec,
        BoilerSpec,
        ChillerSpec,
        DrawOffSource,
        DrivingSource,
        EVChargerSpec,
        HeatPumpSpec,
        InputSpec,
        IrradianceSource,
        MPCModelInterface,
        OccupancySource,
        OutdoorTemperatureSource,
        ParameterSpec,
        PhotovoltaicSource,
        PhotovoltaicSpec,
        StateSpec,
        StorageTankSpec,
        ZoneSpec,
    )

    view = _building_view(building)
    runnable_zones = {zone.name: zone for zone in runnable.zones} if runnable else {}
    external_columns = runnable.external_data.columns if runnable and runnable.external_data else []
    parameters = [
        ParameterSpec(
            name="TGro", value=building.ground_temperature, unit="K", description="Ground temperature below the slab"
        ),
        *(
            ParameterSpec(
                name=coupling.parameter,
                value=coupling.conductance,
                unit="W/K",
                description=f"Conductance between {coupling.zone_a} and {coupling.zone_b}",
            )
            for coupling in building.couplings
        ),
    ]
    inputs = [
        InputSpec(
            name="TOut",
            role=InputRole.disturbance,
            unit="K",
            description="Outdoor dry-bulb air temperature",
            source=OutdoorTemperatureSource(),
        ),
        *(
            InputSpec(
                name=orientation.irradiance,
                role=InputRole.disturbance,
                unit="W/m2",
                description=f"Total solar irradiance, {_describe(orientation)}",
                source=IrradianceSource(azimuth=orientation.azimuth, tilt=orientation.tilt),
            )
            for orientation in building.orientations
        ),
    ]
    states, zones = [], []
    for zone, zone_view in zip(building.zones, view.zones, strict=True):
        parameters += [
            ParameterSpec(
                name=f"{zone.prefix}{parameter.name}",
                value=parameter.value,
                unit=parameter.unit,
                description=parameter.description,
                zone=zone.name,
            )
            for parameter in zone_view.parameters
        ]
        runnable_zone = runnable_zones.get(zone.name)
        occupancy = runnable_zone.occupancy if runnable_zone else None
        heating = next(signal for signal in zone_view.inputs if signal.name.endswith(("_QHea", "_PHea")))
        cooling = next((signal for signal in zone_view.inputs if signal.name.endswith("_PCoo")), None)
        for signal in zone_view.inputs:
            source: OccupancySource | DrawOffSource | DrivingSource | PhotovoltaicSource | None = None
            if signal.name == f"{zone.prefix}QInt":
                source = OccupancySource(
                    zone=zone.name,
                    floor_area=zone.floor_area,
                    model=occupancy.model if occupancy else None,
                    parameters=occupancy.values if occupancy else {},
                    data_column=occupancy.data_column if occupancy else None,
                )
            inputs.append(
                InputSpec(
                    name=signal.name,
                    role=signal.role,
                    unit=signal.unit,
                    description=signal.description,
                    zone=zone.name,
                    source=source,
                    data_column=signal.name if signal.name in external_columns else None,
                    carrier=signal.carrier,
                    system=signal.system,
                )
            )
        states += [
            StateSpec(
                name=f"{zone.prefix}{state.name}",
                zone=zone.name,
                node=state.name,
                initial_value=zone.temperature_initial,
                description=state.description,
            )
            for state in zone.states
        ]
        zones.append(
            ZoneSpec(
                name=zone.name,
                model_type=zone.parameters.model_type.value,
                indoor_temperature=f"{zone.prefix}Ti",
                heating_input=heating.name,
                internal_gains_input=f"{zone.prefix}QInt",
                floor_area=zone.floor_area,
                design_heating_power=zone.design_heating_power,
                heating_carrier=heating.carrier,
                heating_system=heating.system,
                cooling_input=cooling.name if cooling else None,
                cooling_system=cooling.system if cooling else None,
            )
        )
    systems: list[Any] = []
    for system, system_view in zip(building.systems, view.systems, strict=True):
        parameters += [
            ParameterSpec(
                name=parameter.name,
                value=parameter.value,
                unit=parameter.unit,
                description=parameter.description,
                system=system.name,
            )
            for parameter in system_view.parameters
        ]
        for signal in system_view.inputs:
            source = None
            if isinstance(system, DHWTank) and signal.source == "draw_off":
                source = DrawOffSource(
                    daily_energy_kwh=system.draw_off.daily_energy_kwh,
                    hourly_fractions=list(system.draw_off.hourly_fractions),
                )
            elif isinstance(system, EVCharger) and signal.source == "ev_driving":
                source = DrivingSource(
                    arrival_hour=system.sessions.arrival_hour,
                    departure_hour=system.sessions.departure_hour,
                    energy_per_day_kwh=system.sessions.energy_per_day_kwh,
                    weekend_present=system.sessions.weekend_present,
                )
            elif isinstance(system, Photovoltaic) and signal.source == "photovoltaic":
                source = PhotovoltaicSource(
                    azimuth=system.azimuth, tilt=system.tilt, area=system.area, efficiency=system.efficiency
                )
            inputs.append(
                InputSpec(
                    name=signal.name,
                    role=signal.role,
                    unit=signal.unit,
                    description=signal.description,
                    source=source,
                    data_column=signal.name if signal.name in external_columns else None,
                    carrier=signal.carrier,
                    system=signal.system,
                )
            )
        states += [
            StateSpec(
                name=state.name,
                node=state.name.removeprefix(system.prefix),
                unit=state.unit,
                initial_value=state.start,
                description=state.description,
                system=system.name,
            )
            for state in system_view.states
        ]
        controls = [signal.name for signal in system_view.inputs if signal.role == InputRole.control]
        state_names = [state.name for state in system_view.states]
        if isinstance(system, HeatPump):
            zone_inputs = [f"{zone}_PHea" for zone in system.zones]
            tank_inputs = [
                f"{tank.prefix}PHea"
                for tank in building.systems
                if isinstance(tank, DHWTank) and tank.heat_pump == system.name
            ]
            systems.append(
                HeatPumpSpec(
                    name=system.name,
                    inputs=zone_inputs,
                    zones=list(system.zones),
                    max_electrical_power=system.max_electrical_power,
                    cop_nominal=system.cop_nominal,
                    cop_outdoor_slope=system.cop_outdoor_slope,
                    cop_supply_slope=system.cop_supply_slope,
                    cop_cross_term=system.cop_cross_term,
                    supply_temperature=system.supply_temperature,
                    supply_offset=system.supply_offset,
                    tank_inputs=tank_inputs,
                )
            )
        elif isinstance(system, GasBoiler):
            systems.append(
                BoilerSpec(
                    name=system.name,
                    inputs=[f"{zone}_QHea" for zone in system.zones],
                    zones=list(system.zones),
                    efficiency=system.efficiency,
                    max_heating_power=system.max_heating_power,
                )
            )
        elif isinstance(system, Chiller):
            systems.append(
                ChillerSpec(
                    name=system.name,
                    inputs=[f"{zone}_PCoo" for zone in system.zones],
                    zones=list(system.zones),
                    max_electrical_power=system.max_electrical_power,
                    eer_nominal=system.eer_nominal,
                    eer_outdoor_slope=system.eer_outdoor_slope,
                )
            )
        elif isinstance(system, DHWTank):
            systems.append(
                StorageTankSpec(
                    name=system.name,
                    inputs=controls,
                    states=state_names,
                    zone=system.zone,
                    heat_pump=system.heat_pump,
                    state=f"{system.prefix}T",
                    heating_input=f"{system.prefix}PHea",
                    draw_off_input=f"{system.prefix}QDraw",
                    capacitance=system.capacitance,
                    min_temperature=system.min_temperature,
                    set_temperature=system.set_temperature,
                )
            )
        elif isinstance(system, Battery):
            systems.append(
                BatterySpec(
                    name=system.name,
                    inputs=controls,
                    states=state_names,
                    state=f"{system.prefix}E",
                    charge_input=f"{system.prefix}PCha",
                    discharge_input=f"{system.prefix}PDis",
                    capacity=system.capacity,
                    max_charge_power=system.max_charge_power,
                    max_discharge_power=system.max_discharge_power,
                    charge_efficiency=system.charge_efficiency,
                    discharge_efficiency=system.discharge_efficiency,
                    min_soc=system.min_soc,
                    max_soc=system.max_soc,
                )
            )
        elif isinstance(system, EVCharger):
            systems.append(
                EVChargerSpec(
                    name=system.name,
                    inputs=controls,
                    states=state_names,
                    state=f"{system.prefix}E",
                    charge_input=f"{system.prefix}PCha",
                    driving_input=f"{system.prefix}PDri",
                    capacity=system.capacity,
                    max_charge_power=system.max_charge_power,
                    charge_efficiency=system.charge_efficiency,
                    arrival_hour=system.sessions.arrival_hour,
                    departure_hour=system.sessions.departure_hour,
                    target_soc=system.sessions.target_soc,
                    energy_per_day_kwh=system.sessions.energy_per_day_kwh,
                    weekend_present=system.sessions.weekend_present,
                )
            )
        elif isinstance(system, Photovoltaic):
            systems.append(
                PhotovoltaicSpec(
                    name=system.name,
                    generation_input=f"{system.prefix}P",
                    area=system.area,
                    efficiency=system.efficiency,
                    azimuth=system.azimuth,
                    tilt=system.tilt,
                    peak_power=system.peak_power,
                )
            )
    return MPCModelInterface(
        package=package_name,
        model=f"{package_name}.{building.name}",
        runnable_model=f"{package_name}.building" if runnable else None,
        weather_file=runnable.weather_file if runnable else None,
        external_data_columns=external_columns,
        states=states,
        inputs=inputs,
        parameters=parameters,
        zones=zones,
        systems=systems,
    )
