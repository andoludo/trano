import itertools
from pathlib import Path
from typing import Any, TYPE_CHECKING, cast, get_args

import pandas as pd
from pydantic import BaseModel, Field, computed_field
from networkx.classes.reportviews import NodeView

from trano.elements.base import BaseElement, Control
from trano.elements.data_bus.inputs import BaseInputOutput, DataSource, ModelInput
from trano.elements.jinja import compile_template
from trano.elements.types import ContainerTypes

if TYPE_CHECKING:
    from trano.topology import Network


class ValidationData(BaseModel):
    data: str | None = None
    columns: list[str] = Field([])


class DataBus(BaseElement):
    name: str = "data_bus"
    spaces: list[str] = Field(default=[])
    non_connected_ports: list[BaseInputOutput] = Field(default=[])
    model_inputs: list[BaseInputOutput] = Field(
        default=[], description="Signals that no element writes, inputs of the model (rendered ``model_input``)"
    )
    power_ports: list[BaseInputOutput] = Field(default=[])
    external_data: Path | None = None
    container_type: ContainerTypes = "bus"

    @computed_field
    def validation_data(self) -> ValidationData:
        if not self.external_data:
            return ValidationData()
        return transform_csv_to_table(self.external_data)

    def add_to_network(self, network: "Network") -> None:
        from trano.elements import Space, AirHandlingUnit

        spaces = sorted(
            [node for node in network.graph.nodes if isinstance(node, Space)],
            key=lambda x: x.name,
        )
        controls = sorted(
            [node for node in network.graph.nodes if isinstance(node, Control)],
            key=lambda x: x.name,  # type: ignore
        )
        ahus = sorted(
            [node for node in network.graph.nodes if isinstance(node, AirHandlingUnit)],
            key=lambda x: x.name,  # type: ignore
        )
        if not spaces:
            raise ValueError("No spaces in the network")
        self.spaces = [space.name for space in spaces]
        self.external_data = self.external_data
        self.position.set_container(0, 0)
        network.add_node(self)
        for space in spaces:
            network.graph.add_edge(space, self)
            if space.occupancy:
                network.graph.add_edge(space.occupancy, self)
        for control in controls:
            network.graph.add_edge(control, self)
        for ahu in ahus:
            network.graph.add_edge(ahu, self)
        for system in get_bus_only_systems(network.graph.nodes):
            network.graph.add_edge(system, self)
        for element in get_bus_targeted_elements(network.graph.nodes, self):
            network.graph.add_edge(element, self)

    def configure(self, network: "Network") -> None:
        ports = get_non_connected_ports(network.graph.nodes)
        self.non_connected_ports = [port for port in ports if port.model_input is None]
        self.model_inputs = sorted(
            (_render_model_input(port, network) for port in ports if port.model_input is not None),
            key=lambda port: port.bus_member,
        )
        self.power_ports = get_power_ports(network.graph.nodes)


def _render_model_input(port: BaseInputOutput, network: "Network") -> BaseInputOutput:
    """The declaration and the variable of a model input, rendered for the element reading the signal."""
    element = next(node for node in network.graph.nodes if node.name == port.target.evaluated_element)
    context = {"element": element, "parameters": element.processed_parameters(network.library)}
    model_input = cast(ModelInput, port.model_input)
    rendered = ModelInput(
        declaration=compile_template(model_input.declaration).render(**context),
        port=compile_template(model_input.port).render(**context),
    )
    return port.model_copy(update={"model_input": rendered})


def get_bus_only_systems(nodes: list[NodeView]) -> list[Any]:
    """Systems with a port that only a direct edge to the data bus can satisfy.

    A port whose single target is the data bus cannot be reached through any other
    element, so such a system needs its own edge to the bus. Systems that also target
    a control reach the bus through that control instead and are left out here.
    """
    from trano.elements.system import System

    return sorted(
        [
            node
            for node in nodes
            if isinstance(node, System) and any(list(port.targets) == [DataBus] for port in node.ports)
        ],
        key=lambda node: node.name,
    )


def get_bus_targeted_elements(nodes: list[NodeView], data_bus: DataBus) -> list[Any]:
    """Elements, other than systems and controls, that target the data bus and that the data bus targets.

    The ports of both say that they are connected: a PV array whose library reads the weather on the data bus.
    """
    from trano.elements.system import System

    targeted = tuple(target for port in data_bus.ports for target in port.targets)
    return sorted(
        [
            node
            for node in nodes
            if not isinstance(node, (System, Control, DataBus))
            and isinstance(node, targeted)
            and any(DataBus in port.targets for port in node.ports)
        ],
        key=lambda node: node.name,
    )


def get_power_ports(nodes: list[NodeView]) -> list[BaseInputOutput]:
    power_ports = []
    for node in nodes:
        if not (hasattr(node, "component_template") and hasattr(node.component_template, "bus")):
            continue
        if node.component_template and node.component_template.bus:
            node_ports = node.component_template.bus.list_ports(node)
            # the data bus draws the electrical load of the building from these; fuel is not on the grid
            power_ports += [
                port
                for port in node_ports["RealOutput"]
                if port.power is not None and port.power.energy == "Electricity"
            ]
    return power_ports


class BusMember(BaseModel):
    """A signal of the data bus: what an element connects as ``dataBus.<name>``."""

    name: str
    type: str


def get_bus_members(nodes: list[BaseElement]) -> list[BusMember]:
    """The signals of the data bus of a network, for the data bus templates declaring them."""
    members: dict[str, BusMember] = {}
    for node in nodes:
        if not (node.component_template and node.component_template.bus):
            continue
        for port_type, ports in node.component_template.bus.list_ports(node).items():
            for port in ports:
                if port.name in get_args(DataSource):
                    continue
                type_ = port_type.removesuffix("Input").removesuffix("Output")
                members.setdefault(port.bus_member, BusMember(name=port.bus_member, type=type_))
    return sorted(members.values(), key=lambda member: member.name)


def get_non_connected_ports(nodes: list[NodeView]) -> list[BaseInputOutput]:
    port_types = ["Real", "Integer", "Boolean"]
    ports: dict[str, list[BaseInputOutput]] = {
        f"{port_type}{direction}": [] for port_type in port_types for direction in ["Output", "Input"]
    }

    for node in nodes:
        if not (hasattr(node, "component_template") and hasattr(node.component_template, "bus")):
            continue
        if node.component_template and node.component_template.bus:
            node_ports = node.component_template.bus.list_ports(node)
            for port_type in port_types:
                ports[f"{port_type}Output"] += node_ports[f"{port_type}Output"]
                ports[f"{port_type}Input"] += node_ports[f"{port_type}Input"]

    for port_type in port_types:
        ports[f"{port_type}Output"] = list(set(ports[f"{port_type}Output"]))
        ports[f"{port_type}Input"] = list(set(ports[f"{port_type}Input"]))

    inputs = list(
        itertools.chain(
            *[
                _get_non_connected_ports_intersection(ports[f"{port_type}Input"], ports[f"{port_type}Output"])
                for port_type in port_types
            ]
        )
    )
    return [in_ for in_ in inputs if in_.name not in get_args(DataSource)]


def _get_non_connected_ports_intersection(
    input_ports: list[BaseInputOutput], output_ports: list[BaseInputOutput]
) -> list[BaseInputOutput]:
    return list(set(input_ports) - set(output_ports).intersection(set(input_ports)))


def transform_csv_to_table(file_path: Path, total_second: bool = True) -> ValidationData:
    data = pd.read_csv(file_path, index_col=0, infer_datetime_format=True)  # type: ignore
    data = data.ffill().bfill()
    data = data.dropna(axis=1)
    data.index = pd.to_datetime(data.index)
    if total_second:
        data.index = (data.index - data.first_valid_index()).total_seconds()
    else:
        data.index = data.index.astype(int) // 10**9
    data_str = data.to_csv(sep=",", header=False, lineterminator=";")
    data_str = data_str.removesuffix(";")
    return ValidationData(data=data_str, columns=data.columns.tolist())
