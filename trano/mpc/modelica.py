"""Rendering of the CasADi-compatible RC models into Modelica.

The zone equations live in ``templates/mpc.jinja2``. They are rendered twice:

* as the ``Trano.MPC.Zones`` library embedded in the ``Trano`` package of every generated model,
* as the flat multi-zone ``building`` model generated when the ``mpc`` library is selected.
"""

from functools import cache
from typing import TYPE_CHECKING

from pydantic import BaseModel

from trano.mpc.parameters import ZONE_PARAMETERS, ModelicaParameter, ModelicaState, RCModelType

if TYPE_CHECKING:
    from trano.mpc.building import RCBuilding, RCZone
    from trano.topology import Network

LIBRARY_TEMPERATURE_INITIAL = 294.15
LIBRARY_GROUND_TEMPERATURE = 283.15


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


class BuildingView(BaseModel):
    name: str
    model_type: str
    ground_temperature: float
    zones: list[ZoneView]
    couplings: list[dict[str, str | float]]


def _zone_view(building: "RCBuilding", zone: "RCZone") -> ZoneView:
    couplings = []
    for coupling in building.couplings:
        if zone.name in (coupling.zone_a, coupling.zone_b):
            neighbour = coupling.zone_b if coupling.zone_a == zone.name else coupling.zone_a
            couplings.append({"parameter": coupling.parameter, "neighbour": f"{neighbour}_"})
    return ZoneView(
        name=zone.name,
        prefix=zone.prefix,
        model_type=zone.parameters.model_type.value,
        title=zone.parameters.title,
        documentation=zone.parameters.documentation,
        temperature_initial=zone.temperature_initial,
        parameters=zone.parameters.modelica_parameters(),
        states=list(zone.states),
        couplings=couplings,
    )


def _building_view(building: "RCBuilding") -> BuildingView:
    return BuildingView(
        name=building.name,
        model_type=building.model_type,
        ground_temperature=building.ground_temperature,
        zones=[_zone_view(building, zone) for zone in building.zones],
        couplings=[{**coupling.model_dump(), "parameter": coupling.parameter} for coupling in building.couplings],
    )


@cache
def render_mpc_library() -> str:
    """The ``MPC`` package of the Trano library: one model per RC structure, for a reference zone."""
    from trano.elements.jinja import ENVIRONMENT
    from trano.mpc.estimation import reference_zone_parameters

    library_zones = [
        ZoneView(
            name=model_type.value,
            prefix="",
            model_type=model_type.value,
            title=parameters_class.title,
            documentation=parameters_class.documentation,
            temperature_initial=LIBRARY_TEMPERATURE_INITIAL,
            parameters=reference_zone_parameters(model_type).modelica_parameters(),
            states=list(parameters_class.states),
            couplings=[],
        )
        for model_type, parameters_class in ZONE_PARAMETERS.items()
    ]
    template = ENVIRONMENT.from_string(
        "{% import 'mpc.jinja2' as mpc %}{{ mpc.zones_library(library_zones, ground_temperature) }}"
    )
    return template.render(library_zones=library_zones, ground_temperature=LIBRARY_GROUND_TEMPERATURE)


def render_building(building: "RCBuilding", package_name: str) -> str:
    """Modelica package with the Trano library (including ``Trano.MPC``) and the flat building model."""
    from trano.elements.jinja import ENVIRONMENT

    return ENVIRONMENT.get_template("mpc_base.jinja2").render(
        package_name=package_name, building=_building_view(building)
    )


def render_network(network: "Network") -> str:
    """Model of a network generated with the ``mpc`` library."""
    from trano.mpc.estimation import rc_building_from_network

    model_type = network.library.rc_model_type or RCModelType.r3c2
    building = rc_building_from_network(network, model_type=model_type)
    return render_building(building, network.name)
