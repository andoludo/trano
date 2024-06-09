import itertools
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Type, Union

import matplotlib.pyplot as plt
import networkx as nx
from jinja2 import Environment, FileSystemLoader
from networkx import DiGraph, shortest_path
from networkx.classes.reportviews import NodeView
from pyvis.network import Network as PyvisNetwork  # type: ignore

from neosim.construction import Constructions
from neosim.controller.parser import BaseInput
from neosim.documentation.json import generate_html_documentation
from neosim.library.library import Buildings, Libraries
from neosim.models.constants import Tilt
from neosim.models.elements.ahu import AirHandlingUnit
from neosim.models.elements.base import (
    BaseElement,
    Connection,
    DynamicTemplateCategories,
    connect,
)
from neosim.models.elements.bus import DataBus
from neosim.models.elements.controls.base import Control
from neosim.models.elements.controls.collector import CollectorControl
from neosim.models.elements.damper import VAV
from neosim.models.elements.envelope.internal_element import InternalElement
from neosim.models.elements.materials.properties import extract_properties
from neosim.models.elements.pump import Pump
from neosim.models.elements.space import Space, _get_controllable_element
from neosim.models.elements.system import System, Ventilation
from neosim.models.elements.valve import Valve
from neosim.models.elements.weather import Weather


class Network:
    def __init__(
        self,
        name: str,
        library: Optional[Libraries] = None,
    ) -> None:
        self.graph: DiGraph = DiGraph()
        self.edge_attributes: List[Connection] = []
        self.name: str = name
        self._system_controls: List[Control] = []
        self.library = library or Buildings()
        self.dynamic_components: Dict[DynamicTemplateCategories, List[str]] = {
            "ventilation": [],
            "control": [],
            "boiler": [],
        }

    def add_node(self, node: BaseElement) -> None:

        if not node.libraries_data:
            raise Exception(
                f"No library data defined for NOde of type {type(node).__name__}"
            )
        found_library = node.assign_library_property(self.library)
        if not found_library:
            return

        if node not in self.graph.nodes:
            self.graph.add_node(node)
        if (isinstance(node, System) and node.control) and (
            node.control not in self.graph.nodes
        ):
            node_control = node.control
            if not node_control.libraries_data:
                raise Exception(
                    f"No library data defined for NOde of type {type(node).__name__}"
                )
            node_control.assign_library_property(self.library)
            self.graph.add_node(node_control)
            self.graph.add_edge(node, node_control)
            node_control.controllable_element = node

    def add_space(self, space: "Space") -> None:
        self.add_node(space)
        if self.library.merged_external_boundaries:
            external_boundaries = space.merged_external_boundaries
        else:
            external_boundaries = space.external_boundaries  # type: ignore
        for boundary in external_boundaries:
            self.add_node(boundary)
            self.graph.add_edge(
                space,
                boundary,
            )
        self._build_space_emission(space)  # TODO: perhaps move to space
        self._build_occupancy(space)
        self._build_space_ventilation(space)
        space.assign_position()

    def _add_subsequent_systems(self, systems: List[System]) -> None:
        for system1, system2 in zip(systems[:-1], systems[1:]):
            if not self.graph.has_node(system1):  # type: ignore
                self.add_node(system1)
            if not self.graph.has_node(system2):  # type: ignore
                self.add_node(system2)
            self.graph.add_edge(
                system1,
                system2,
            )

    def _build_space_ventilation(self, space: "Space") -> None:
        # Assumption: first element always the one connected to the space.
        if space.get_ventilation_inlet():
            self.add_node(space.get_ventilation_inlet())  # type: ignore
            self.graph.add_edge(space.get_ventilation_inlet(), space)
        if space.get_ventilation_outlet():
            self.add_node(space.get_ventilation_outlet())  # type: ignore
            self.graph.add_edge(space, space.get_ventilation_outlet())
        # The rest is connected to each other
        self._add_subsequent_systems(space.ventilation_outlets)
        self._add_subsequent_systems(space.ventilation_inlets)

    def _build_space_emission(self, space: "Space") -> None:
        emission = space.find_emission()
        if emission:
            self.add_node(emission)
            self.graph.add_edge(
                space,
                emission,
            )
            self._add_subsequent_systems(space.emissions)

    def _build_data_bus(self) -> DataBus:

        spaces = sorted(
            [node for node in self.graph.nodes if isinstance(node, Space)],
            key=lambda x: x.name,
        )
        controls = sorted(
            [node for node in self.graph.nodes if isinstance(node, Control)],
            key=lambda x: x.name,
        )
        ahus = sorted(
            [node for node in self.graph.nodes if isinstance(node, AirHandlingUnit)],
            key=lambda x: x.name,
        )
        data_bus = DataBus(
            name="data_bus",
            spaces=[space.name for space in spaces],
        )
        self.add_node(data_bus)
        for space in spaces:
            self.graph.add_edge(space, data_bus)
        for control in controls:
            self.graph.add_edge(control, data_bus)
        for ahu in ahus:
            self.graph.add_edge(ahu, data_bus)
        return data_bus

    def _build_full_space_control(self) -> None:
        spaces = [node for node in self.graph.nodes if isinstance(node, Space)]

        for space in spaces:
            _neighbors = []
            if space.get_last_ventilation_inlet():
                _neighbors += list(
                    self.graph.predecessors(space.get_last_ventilation_inlet())  # type: ignore
                )
            if space.get_last_ventilation_outlet():
                _neighbors += list(
                    self.graph.predecessors(space.get_last_ventilation_outlet())  # type: ignore
                )
            neighbors = list(set(_neighbors))
            controllable_ventilation_elements = list(
                filter(
                    None,
                    [
                        _get_controllable_element(space.ventilation_inlets),
                        _get_controllable_element(space.ventilation_outlets),
                    ],
                )
            )
            for controllable_element in controllable_ventilation_elements:
                if controllable_element.control:
                    controllable_element.control.ahu = next(
                        (n for n in neighbors if isinstance(n, AirHandlingUnit)), None
                    )

    def _build_occupancy(self, space: "Space") -> None:
        if space.occupancy:
            self.add_node(space.occupancy)
            self.connect_system(space, space.occupancy)

    def connect_spaces(
        self,
        space_1: "Space",
        space_2: "Space",
        internal_element: Optional[
            "InternalElement"
        ] = None,  # TODO: this should not be optional
    ) -> None:
        internal_element = internal_element or InternalElement(
            name=f"internal_{space_1.name}_{space_2.name}",
            surface=10,
            azimuth=10,
            construction=Constructions.internal_wall,
            tilt=Tilt.wall,
        )
        internal_element.position = [
            space_1.position[0] + (space_2.position[0] - space_1.position[0]) / 2,
            space_1.position[1],
        ]  # TODO: this is to be moved somewher
        self.add_node(internal_element)
        self.graph.add_edge(
            space_1,
            internal_element,
        )
        self.graph.add_edge(
            space_2,
            internal_element,
        )
        space_1.internal_elements.append(internal_element)
        space_2.internal_elements.append(internal_element)

    def connect_system(self, space: "Space", system: "System") -> None:
        self.graph.add_edge(
            space,
            system,
        )

    def _assign_position(
        self, system_1: System, system_2: System  # noqa :  PLR6301
    ) -> None:
        # TODO: change position to object
        if system_1.position and not system_2.position:
            system_2.position = [system_1.position[0] + 100, system_1.position[1] - 100]
            if hasattr(system_2, "control") and system_2.control:
                system_2.control.position = [
                    system_2.position[0] - 50,
                    system_2.position[1],
                ]
        if system_2.position and not system_1.position:
            system_1.position = [system_2.position[0] - 100, system_2.position[1] - 100]
            if hasattr(system_1, "control") and system_1.control:
                system_1.control.position = [
                    system_1.position[0] - 50,
                    system_1.position[1],
                ]

    def connect_elements(self, element_1: BaseElement, element_2: BaseElement) -> None:
        for element in [element_1, element_2]:
            if element not in self.graph.nodes:
                self.add_node(element)
        self.graph.add_edge(element_1, element_2)
        self._assign_position(element_1, element_2)  # type: ignore

    def connect_systems(self, system_1: System, system_2: System) -> None:

        if system_1 not in self.graph.nodes:
            self.add_node(system_1)
            if system_1.control:
                if system_1.control not in self.graph.nodes:
                    self.add_node(system_1.control)
                    self._system_controls.append(system_1.control)
                self.graph.add_edge(system_1, system_1.control)
                # TODO: check if it is controllable the system

        if system_2 not in self.graph.nodes:
            self.add_node(system_2)
            if system_2.control:
                if system_2.control not in self.graph.nodes:
                    self.add_node(system_2.control)
                    self._system_controls.append(system_2.control)
                self.graph.add_edge(system_2, system_2.control)
        self.graph.add_edge(system_1, system_2)
        self._assign_position(system_1, system_2)

    def connect_edges(
        self, edge: Tuple[BaseElement, BaseElement]  # noqa :  PLR6301
    ) -> list[Connection]:
        return connect(edge)

    def merge_spaces(self, space_1: "Space", space_2: "Space") -> None:
        internal_elements = nx.shortest_path(self.graph, space_1, space_2)[1:-1]
        merged_space = space_1 + space_2
        merged_space.internal_elements = internal_elements
        self.graph = nx.contracted_nodes(self.graph, merged_space, space_2)

    def generate_layout(self) -> Dict[Any, Any]:
        # nodes = [n for n in self.graph.nodes if isinstance(n, Space)] # noqa : E800
        # for i, n in enumerate(nodes): # : E800
        #     n.assign_position([200*i, 50]) # noqa : E800

        return nx.spring_layout(self.graph, k=10, dim=2, scale=200)  # type: ignore

    def generate_graphs(self) -> None:
        layout = self.generate_layout()
        for node in self.graph.nodes:
            node.get_position(layout)
            if isinstance(node, Space):
                node.get_neighhors(self.graph)
        for edge in self.graph.edges:
            self.edge_attributes += self.connect_edges(edge)

    def _connect_space_controls(self) -> None:
        undirected_graph = self.graph.to_undirected()
        space_controls = [
            node for node in undirected_graph.nodes if isinstance(node, Space)
        ]
        for space_control in space_controls:
            for system_control in self._system_controls:
                shortest_path(undirected_graph, system_control, space_control)

    def get_ahu_elements(
        self, ahu: AirHandlingUnit, element_type: Type[Union[VAV, Space]]
    ) -> List[Union[VAV, Space]]:
        elements_: List[Union[VAV, Space]] = []
        elements = [node for node in self.graph.nodes if isinstance(node, element_type)]
        for element in elements:
            paths = nx.shortest_path(self.graph, ahu, element)
            p = paths[1:-1]
            if p and all(isinstance(p_, Ventilation) for p_ in p):
                elements_.append(element)
        return elements_

    def configure_ahu_control(self) -> None:
        ahus = [node for node in self.graph.nodes if isinstance(node, AirHandlingUnit)]
        for ahu in ahus:
            if ahu.control:
                ahu.control.spaces = self.get_ahu_elements(ahu, Space)
                ahu.control.vavs = self.get_ahu_elements(ahu, VAV)

    def get_linked_valves(self, pump_collector: BaseElement) -> List[Valve]:
        valves_: List[Valve] = []
        valves = [node for node in self.graph.nodes if isinstance(node, Valve)]
        for valve in valves:
            paths = nx.shortest_path(self.graph, pump_collector, valve)
            p = paths[1:-1]
            if p and all(isinstance(p_, System) for p_ in p):
                valves_.append(valve)
        return valves_

    def configure_collector_control(self) -> None:
        pump_collectors = [
            node
            for node in self.graph.nodes
            if isinstance(node, Pump) and isinstance(node.control, CollectorControl)
        ]
        for pump_collector in pump_collectors:
            pump_collector.control.valves = self.get_linked_valves(pump_collector)

    def model(self) -> str:
        Space.counter = 0
        self._build_full_space_control()
        data_bus = self._build_data_bus()
        self.configure_ahu_control()
        self.configure_collector_control()

        data_bus.non_connected_ports = get_non_connected_ports(self.graph.nodes)

        self.generate_graphs()

        self._connect_space_controls()

        element_models = self.build_element_models()
        environment = Environment(
            trim_blocks=True,
            lstrip_blocks=True,
            loader=FileSystemLoader(str(Path(__file__).parent.joinpath("templates"))),
            autoescape=True,
        )
        environment.filters["frozenset"] = frozenset
        environment.filters["enumerate"] = enumerate

        template = environment.get_template("base.jinja2")

        data = extract_properties(self.library, self.name, self.graph.nodes)
        return template.render(
            network=self,
            data=data,
            element_models=element_models,
            library=self.library,
            databus=data_bus,
            dynamic_components=self.dynamic_components,
            documentation=generate_html_documentation(self),
        )

    def build_dynamic_component_template(self, node: BaseElement) -> None:
        if node.component_template:
            component = node.component_template.render(
                self.name, node, node.processed_parameters(self.library)
            )
            if node.component_template.category:
                self.dynamic_components[node.component_template.category].append(
                    component
                )

    def build_element_models(self) -> List[str]:
        environment = Environment(
            trim_blocks=True,
            lstrip_blocks=True,
            loader=FileSystemLoader(str(Path(__file__).parent.joinpath("templates"))),
            autoescape=True,
        )
        environment.filters["enumerate"] = enumerate
        models = []
        for node in self.graph.nodes:
            if not node.template:
                continue
            environment.globals.update(self.library.functions)
            rtemplate = environment.from_string(
                "{% import 'macros.jinja2' as macros %}"
                + node.template
                + " "
                + node.annotation_template
            )
            if node.component_template:
                self.build_dynamic_component_template(node)
            model = rtemplate.render(
                element=node,
                package_name=self.name,
                library_name=self.library.name,
                parameters=node.processed_parameters(self.library),
            )
            models.append(model)
        return models

    def add_boiler_plate_spaces(self, spaces: list[Space]) -> None:
        for space in spaces:
            self.add_space(space)
        for combination in itertools.combinations(spaces, 2):
            self.connect_spaces(*combination)
        weather = Weather(name="weather")
        weather.position = [-100, 200]  # TODO: move somewhere else
        self.add_node(weather)
        for space in spaces:
            self.connect_system(space, weather)

    def plot(self, use_pyvis: bool = True) -> None:
        if use_pyvis:
            net = PyvisNetwork(notebook=True)
            plot_graph = DiGraph()
            for node in self.graph.nodes:
                plot_graph.add_node(node.name)
            for edge in self.graph.edges:
                plot_graph.add_edge(edge[0].name, edge[1].name)
            net.from_nx(plot_graph)
            net.toggle_physics(True)
            net.show("example.html")
        else:
            nx.draw(self.graph)
            plt.draw()
            plt.show()


def get_non_connected_ports(nodes: List[NodeView]) -> List[BaseInput]:
    port_types = ["Real", "Integer", "Boolean"]
    ports: Dict[str, List[BaseInput]] = {
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
    input_ports: List[BaseInput], output_ports: List[BaseInput]
) -> List[BaseInput]:
    return list(set(input_ports) - set(output_ports).intersection(set(input_ports)))
