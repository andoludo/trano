import itertools
from pathlib import Path
from typing import List, Optional, TYPE_CHECKING, Dict

import pandas as pd
from pydantic import BaseModel, Field, computed_field
from networkx.classes.reportviews import NodeView

from trano.elements.base import BaseElement, Control
from trano.elements.data_bus.inputs import BaseInputOutput
from trano.elements.types import ContainerTypes

if TYPE_CHECKING:
    from trano.topology import Network


class ValidationData(BaseModel):
    data: Optional[str] = None
    columns: List[str] = Field([])


class DataBus(BaseElement):
    name: str = "data_bus"
    spaces: List[str] = Field(default=[])
    non_connected_ports: List[BaseInputOutput] = Field(default=[])
    power_ports: List[BaseInputOutput] = Field(default=[])
    external_data: Optional[Path] = None
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

    def configure(self, network: "Network") -> None:
        self.non_connected_ports = get_non_connected_ports(network.graph.nodes)
        self.power_ports = get_power_ports(network.graph.nodes)


def get_power_ports(nodes: List[NodeView]) -> List[BaseInputOutput]:
    power_ports = []
    for node in nodes:
        if not (
            hasattr(node, "component_template")
            and hasattr(node.component_template, "bus")
        ):
            continue
        if node.component_template and node.component_template.bus:
            node_ports = node.component_template.bus.list_ports(node)
            power_ports += [p for p in node_ports["RealOutput"] if p.power is not None]
    return power_ports


def get_non_connected_ports(nodes: List[NodeView]) -> List[BaseInputOutput]:
    port_types = ["Real", "Integer", "Boolean"]
    ports: Dict[str, List[BaseInputOutput]] = {
        f"{port_type}{direction}": []
        for port_type in port_types
        for direction in ["Output", "Input"]
    }

    for node in nodes:
        if not (
            hasattr(node, "component_template")
            and hasattr(node.component_template, "bus")
        ):
            continue
        if node.component_template and node.component_template.bus:
            node_ports = node.component_template.bus.list_ports(node)
            for port_type in port_types:
                ports[f"{port_type}Output"] += node_ports[f"{port_type}Output"]
                ports[f"{port_type}Input"] += node_ports[f"{port_type}Input"]

    for port_type in port_types:
        ports[f"{port_type}Output"] = list(set(ports[f"{port_type}Output"]))
        ports[f"{port_type}Input"] = list(set(ports[f"{port_type}Input"]))

    return list(
        itertools.chain(
            *[
                _get_non_connected_ports_intersection(
                    ports[f"{port_type}Input"], ports[f"{port_type}Output"]
                )
                for port_type in port_types
            ]
        )
    )


def _get_non_connected_ports_intersection(
    input_ports: List[BaseInputOutput], output_ports: List[BaseInputOutput]
) -> List[BaseInputOutput]:
    return list(set(input_ports) - set(output_ports).intersection(set(input_ports)))


def transform_csv_to_table(
    file_path: Path, total_second: bool = True
) -> ValidationData:
    data = pd.read_csv(file_path, index_col=0, infer_datetime_format=True)
    data = data.ffill().bfill()
    data = data.dropna(axis=1)
    data.index = pd.to_datetime(data.index)
    if total_second:
        data.index = (data.index - data.first_valid_index()).total_seconds()  # type: ignore
    else:
        data.index = data.index.astype(int) // 10**9
    data_str = data.to_csv(sep=",", header=False, lineterminator=";")
    if data_str.endswith(";"):
        data_str = data_str[:-1]
    return ValidationData(data=data_str, columns=data.columns.tolist())
