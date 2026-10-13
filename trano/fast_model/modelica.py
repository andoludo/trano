"""Modelica package and interface of a network generated with the ``FastModel`` library.

The package holds the Trano library (occupancy models, ...), the ``FastModel`` components, ``building_mpc``
assembled from them with ``connect()`` statements, and the runnable ``building`` that feeds it the weather and
gains of the YAML. The zones, their weather and gains, and the RC parameters of the zones are read with the
helpers of the ``mpc`` library, which is not changed by this one.
"""

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

from trano.fast_model.components import ConnectedModel, package_source
from trano.fast_model.interface import FastModelInterface, InputSpec, IrradianceSource
from trano.mpc.interface import InputRole, OutdoorTemperatureSource, ParameterSpec, StateSpec
from trano.mpc.modelica import (
    OrientationView,
    RunnableControlView,
    RunnableView,
    _building_view,
    _column,
    _prepare,
    build_interface,
)

if TYPE_CHECKING:
    from trano.elements.bus import DataBus
    from trano.mpc.building import RCBuilding, RCZone
    from trano.topology import Network

WEATHER_INPUTS = {
    "TOut": ("TDryBul", "K"),
    "TBlaSky": ("TBlaSky", "K"),
    "winSpe": ("winSpe", "m/s"),
    "winDir": ("winDir", "rad"),
}
IRRADIANCE_COMPONENTS = {"HDir": "direct", "HDif": "diffuse_perez", "cosInc": "incidence_cosine"}


@dataclass(frozen=True)
class FastRunnable:
    """What the runnable model of ``FastModel`` adds to the one of the ``mpc`` library."""

    multilayer: bool
    indoor_temperatures: dict[str, str] = field(default_factory=dict)  # zone -> variable of building_mpc
    control_units: dict[str, str] = field(default_factory=dict)  # control input -> unit (W by default)


def is_multilayer(network: "Network") -> bool:
    return network.library.envelope == "multilayer"


def _indoor_temperature(network: "Network", zone: "RCZone") -> str:
    if is_multilayer(network):
        return f"{zone.name}_air.T"
    from trano.fast_model.components.rc import indoor_temperature

    return indoor_temperature(zone)


def connected_model(network: "Network", building: "RCBuilding") -> ConnectedModel:
    """``building_mpc`` of a network: multilayer zones with their systems, or the RC zones, connected."""
    if is_multilayer(network):
        from trano.fast_model.components.envelope import multilayer_model

        return multilayer_model(network)
    from trano.fast_model.components.rc import connected_rc_model

    return connected_rc_model(building)


def _multilayer_runnable(network: "Network", runnable: RunnableView, model: ConnectedModel) -> RunnableView:
    """The runnable view of multilayer zones: their orientations, in radians, named as their inputs, and the
    controls of the model (zone heat inputs, hydronic controls), replayed from the external data when it has them."""
    from trano.fast_model.components.envelope import zone_orientations

    orientations = [
        OrientationView(
            name=orientation.name,
            irradiance=orientation.direct,
            azimuth=orientation.azimuth,
            tilt=orientation.tilt,
            description=(
                f"azimuth {math.degrees(orientation.azimuth):g} deg, tilt {math.degrees(orientation.tilt):g} deg"
            ),
        )
        for orientation in zone_orientations(network)
    ]
    controls = [
        RunnableControlView(
            name=top.name, description=top.description, column=_column(runnable.external_data, top.name)
        )
        for top in model.inputs
        if top.control
    ]
    return runnable.model_copy(
        update={"orientations": orientations, "controls": controls, "tables": [], "photovoltaics": []}
    )


def _runnable(
    network: "Network", building: "RCBuilding", runnable: RunnableView, model: ConnectedModel
) -> tuple[RunnableView, FastRunnable]:
    multilayer = is_multilayer(network)
    if multilayer:
        runnable = _multilayer_runnable(network, runnable, model)
    fast = FastRunnable(
        multilayer=multilayer,
        indoor_temperatures={zone.name: _indoor_temperature(network, zone) for zone in building.zones},
        control_units={top.name: top.unit for top in model.inputs if top.control and top.unit},
    )
    return runnable, fast


def render_network(network: "Network", data_bus: "DataBus | None" = None) -> str:
    """The Modelica package of a network generated with the ``FastModel`` library."""
    from trano.elements.jinja import ENVIRONMENT

    building, runnable = _prepare(network, data_bus)
    model = connected_model(network, building)
    runnable, fast = _runnable(network, building, runnable, model)
    return ENVIRONMENT.get_template("fast_model_base.jinja2").render(
        package_name=network.name,
        fast_model_package=package_source(),
        building_model=model.render(),
        building=_building_view(building),
        runnable=runnable,
        fast=fast,
    )


def network_interface(network: "Network", data_bus: "DataBus | None" = None) -> FastModelInterface:
    """Interface of the model generated for a network with the ``FastModel`` library."""
    building, runnable = _prepare(network, data_bus)
    model = connected_model(network, building)
    runnable, _ = _runnable(network, building, runnable, model)
    interface = FastModelInterface.model_validate(build_interface(building, network.name, runnable).model_dump())
    if is_multilayer(network):
        return multilayer_interface(interface, model)
    return connected_interface(interface, model)


def _state_and_parameter_specs(model: ConnectedModel) -> tuple[list[StateSpec], list[ParameterSpec]]:
    """States and parameters of a connected model, described from the component catalogue."""
    instances = {instance.name: instance for instance in model.instances}
    states = []
    for name, initial in zip(model.state_names, model.initial_state(), strict=True):
        instance_name, variable = name.split(".", 1)
        instance = instances[instance_name]
        state = next(state for state in instance.component_class.states if state.name == variable.split("[")[0])
        states.append(
            StateSpec(
                name=name,
                zone=instance.zone,
                system=instance.system,
                node=instance_name.removeprefix(f"{instance.zone}_") if instance.zone else instance_name,
                unit=state.unit or "K",
                initial_value=initial,
                description=instance.description or state.description,
            )
        )
    parameters = []
    for name, value in model.parameters:
        instance_name, variable = name.split(".", 1)
        instance = instances[instance_name]
        parameter = instance.component_class.parameter(variable.split("[", 1)[0])
        description = (
            f"{instance.description}: {parameter.description}" if instance.description else parameter.description
        )
        parameters.append(
            ParameterSpec(
                name=name,
                value=value,
                unit=parameter.unit,
                description=description,
                zone=instance.zone,
                system=instance.system,
            )
        )
    return states, parameters


def _irradiance_input(name: str, unit: str, description: str) -> InputSpec:
    """``<HDir|HDif|cosInc>_azi<azimuth>_til<tilt>``: a part of the irradiance on an orientation."""
    prefix, orientation = name.split("_", 1)
    azimuth, tilt = (float(part[3:].replace("m", "-")) for part in orientation.split("_"))
    return InputSpec(
        name=name,
        role=InputRole.disturbance,
        unit=unit,
        description=description,
        source=IrradianceSource(azimuth=azimuth, tilt=tilt, component=IRRADIANCE_COMPONENTS[prefix]),
    )


def multilayer_interface(interface: FastModelInterface, model: ConnectedModel) -> FastModelInterface:
    """The interface of a multilayer model: weather and irradiance parts as inputs, zone inputs as in the RC model."""
    zone_inputs = {signal.name: signal for signal in interface.inputs if signal.zone}
    inputs = []
    for top in model.inputs:
        if top.name in zone_inputs:
            inputs.append(zone_inputs[top.name])
        elif top.control:
            inputs.append(
                InputSpec(
                    name=top.name,
                    role=InputRole.control,
                    unit=top.unit,
                    description=top.description,
                    zone=top.zone,
                    system=top.system,
                    minimum=top.minimum,
                    maximum=top.maximum,
                )
            )
        elif top.name in WEATHER_INPUTS:
            variable, unit = WEATHER_INPUTS[top.name]
            inputs.append(
                InputSpec(
                    name=top.name,
                    role=InputRole.disturbance,
                    unit=unit,
                    description=top.description,
                    source=OutdoorTemperatureSource(variable=variable, unit=unit),
                )
            )
        else:
            inputs.append(_irradiance_input(top.name, top.unit, top.description))
    states, parameters = _state_and_parameter_specs(model)
    names = set(model.input_names)
    zones = []
    for zone in interface.zones:
        update: dict[str, Any] = {"indoor_temperature": f"{zone.name}_air.T", "model_type": "Multilayer"}
        if zone.heating_input not in names:  # heated by a hydronic circuit: its branch flows heat it
            branch = next((top for top in model.inputs if top.control and top.zone == zone.name), None)
            update |= {
                "heating_input": branch.name if branch else "",
                "heating_system": branch.system if branch else None,
                "heating_carrier": "heat",
            }
        zones.append(zone.model_copy(update=update))
    return interface.model_copy(
        update={"inputs": inputs, "states": states, "parameters": parameters, "zones": zones, "systems": []}
    )


def connected_interface(interface: FastModelInterface, model: ConnectedModel) -> FastModelInterface:
    """The interface of the RC model rewritten for its connected version (same inputs, other states and parameters)."""
    if model.input_names != interface.input_names:
        raise ValueError("The connected model must keep the inputs of the flat model, in the same order.")
    states, parameters = _state_and_parameter_specs(model)
    # The capacitor of the indoor air of zone z is the instance z_Ti (trano.fast_model.components.rc.state_instance).
    zones = [zone.model_copy(update={"indoor_temperature": f"{zone.name}_Ti.T"}) for zone in interface.zones]
    return interface.model_copy(update={"states": states, "parameters": parameters, "zones": zones})
