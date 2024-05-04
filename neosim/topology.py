import itertools
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import networkx as nx
from jinja2 import Environment, FileSystemLoader
from networkx import DiGraph, shortest_path
from pyvis.network import Network as PyvisNetwork  # type: ignore

from neosim.construction import Constructions
from neosim.library.base import DefaultLibrary
from neosim.library.buildings.buildings import BuildingsLibrary
from neosim.models.constants import Tilt
from neosim.models.elements.base import BaseElement, Connection, connect
from neosim.models.elements.control import Control
from neosim.models.elements.space import Space, _get_controllable_element
from neosim.models.elements.system import System, Weather
from neosim.models.elements.wall import InternalElement


class Network:
    def __init__(
        self,
        name: str,
        library: Optional[DefaultLibrary] = None,
    ) -> None:
        self.graph: DiGraph = DiGraph()
        self.edge_attributes: List[Connection] = []
        self.name: str = name
        self._system_controls: List[Control] = []
        self.library = library or BuildingsLibrary()

    def add_node(self, node: BaseElement) -> None:
        node = self.library.assign_properties(node)
        self.graph.add_node(node)

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
        self._build_control(space)  # TODO: perhaps move to space
        self._build_occupancy(space)
        self._build_space_ventilation(space)
        self._build_ventilation_control(space)
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

    def _build_control(self, space: "Space") -> None:
        if space.control:
            self.add_node(space.control)
            self.graph.add_edge(
                space.control,
                space,
            )
            for space_emission in space.emissions:
                self.library.assign_properties(space_emission)
            controllable_emission = space.get_controllable_emission()
            if controllable_emission is None:
                raise Exception(
                    f"Space {space.name} is controllable but is "
                    f"not linked to controllable emission."
                )
            self.graph.add_edge(space.control, controllable_emission)

    def _build_ventilation_control(self, space: "Space") -> None:
        if space.ventilation_control:
            self.add_node(space.ventilation_control)
            self.graph.add_edge(
                space.ventilation_control,
                space,
            )
            for ventilation_element in (
                space.ventilation_inlets + space.ventilation_outlets
            ):
                self.library.assign_properties(ventilation_element)
            controllable_ventilation_elements = list(
                filter(
                    None,
                    [
                        _get_controllable_element(space.ventilation_inlets),
                        _get_controllable_element(space.ventilation_outlets),
                    ],
                )
            )
            if not controllable_ventilation_elements:
                raise Exception(
                    f"Space {space.name} is controllable but is "
                    f"not linked to controllable emission."
                )
            for controllable_ventilation_element in controllable_ventilation_elements:
                self.graph.add_edge(
                    space.ventilation_control, controllable_ventilation_element
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
            if system_2.control:
                system_2.control.position = [
                    system_2.position[0] - 50,
                    system_2.position[1],
                ]
        if system_2.position and not system_1.position:
            system_1.position = [system_2.position[0] - 100, system_2.position[1] - 100]
            if system_1.control:
                system_1.control.position = [
                    system_1.position[0] - 50,
                    system_1.position[1],
                ]

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

    def model(self) -> str:
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

        template = environment.get_template("base.jinja2")

        data = self.library.extract_data(self.name, self.graph.nodes)
        return template.render(
            network=self, data=data, element_models=element_models, library=self.library
        )

    def build_element_models(self) -> List[str]:
        environment = Environment(
            trim_blocks=True,
            lstrip_blocks=True,
            loader=FileSystemLoader(str(Path(__file__).parent.joinpath("templates"))),
            autoescape=True,
        )
        models = []
        for node in self.graph.nodes:
            environment.globals.update(self.library.functions)
            rtemplate = environment.from_string(
                "{% import 'macros.jinja2' as macros %}" + node.template
            )
            model = rtemplate.render(
                element=node,
                package_name=self.name,
                library_name=self.library.library_name,
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
