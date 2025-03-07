import itertools
import logging
import shutil
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Type, cast

import networkx as nx
from jinja2 import Environment, FileSystemLoader
from networkx import DiGraph

from trano.elements import (
    BaseElement,
    Connection,
    Control,
    DynamicTemplateCategories,
    InternalElement,
    connect,
)
from trano.elements.base import ElementPort
from trano.elements.bus import DataBus
from trano.elements.construction import (
    extract_properties,
    Construction,
    Layer,
    Material,
)
from trano.elements.containers import containers_factory, ContainerInput
from trano.elements.library.library import Library
from trano.elements.space import Space
from trano.elements.system import (
    System,
    TemperatureSensor,
    ThreeWayValve,
    Weather,
)
from trano.elements.types import Tilt
from trano.elements.utils import generate_normalized_layout
from trano.exceptions import SystemsNotConnectedError

logger = logging.getLogger(__name__)


def all_subclasses(cls: Type[BaseElement]) -> List[Type[BaseElement]]:
    return cls.__subclasses__() + [
        g for s in cls.__subclasses__() for g in all_subclasses(s)
    ]


def reset_element_names() -> None:
    from trano.data_models.conversion import COUNTER

    COUNTER.clear()
    for subclass in all_subclasses(BaseElement):
        subclass.name_counter = 0


def default_internal_wall_construction() -> Construction:
    return Construction(
        name="internal_wall",
        layers=[
            Layer(
                material=Material(
                    name="brick",
                    thermal_conductivity=0.89,
                    density=1920,
                    specific_heat_capacity=790,
                ),
                thickness=0.2,
            ),
        ],
    )


class Network:  # : PLR0904, #TODO: fix this
    def __init__(
        self,
        name: str,
        library: Optional[Library] = None,
        external_data: Optional[Path] = None,
        diagram_scale: Optional[int] = None,
    ) -> None:
        reset_element_names()
        self.graph: DiGraph = DiGraph()
        self.edge_attributes: List[Connection] = []
        self.name: str = name
        self._system_controls: List[Control] = []
        self.library = library or Library.load_default()
        self.external_data = external_data
        self.containers = containers_factory()
        self.diagram_scale = diagram_scale or 1000
        self.dynamic_components: Dict[DynamicTemplateCategories, List[str]] = {
            "ventilation": [],
            "control": [],
            "boiler": [],
        }

    @property
    def diagram_size(self) -> str:
        return f"{{{{{-50},{-50}}},{{{self.diagram_scale},{self.diagram_scale}}}}}"

    def get_node(self, element: Type[BaseElement]) -> Optional[BaseElement]:
        element_node = [node for node in self.graph.nodes if isinstance(node, element)]
        if element_node:
            return element_node[0]
        return None

    def get_edge(
        self, first_edge: Type[BaseElement], second_edge: Type[BaseElement]
    ) -> Connection:
        return cast(
            Connection,
            next(
                edge
                for edge in self.graph.edges
                if (
                    (
                        isinstance(edge[0], first_edge)
                        and isinstance(edge[1], second_edge)
                    )
                    or (
                        isinstance(edge[1], first_edge)
                        and isinstance(edge[0], second_edge)
                    )
                )
            ),
        )

    def add_node(self, node: BaseElement) -> None:

        if not node.libraries_data:
            return
            # TODO: check better option here!!
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
            construction=default_internal_wall_construction(),
            tilt=Tilt.wall,
        )
        if space_1.position.is_global_empty() or space_2.position.is_global_empty():
            raise Exception("Position not assigned to spaces")
        internal_element.position.between_two_objects(
            space_1.position.global_, space_2.position.global_
        )

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

    def connect_elements(self, element_1: BaseElement, element_2: BaseElement) -> None:
        for element in [element_1, element_2]:
            if element not in self.graph.nodes:
                self.add_node(element)
        self.graph.add_edge(element_1, element_2)

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
        if (
            isinstance(system_2, ThreeWayValve)
            and isinstance(system_1, TemperatureSensor)
        ) or (
            isinstance(system_1, ThreeWayValve)
            and isinstance(system_2, TemperatureSensor)
        ):
            if system_2.control:
                self.graph.add_edge(system_2.control, system_1)
            if system_1.control:
                self.graph.add_edge(system_1.control, system_2)
        self.graph.add_edge(system_1, system_2)

    def connect_edges(
        self, edge: Tuple[BaseElement, BaseElement]  # :  PLR6301
    ) -> list[Connection]:
        connection = connect(
            ElementPort.from_element(edge[0]), ElementPort.from_element(edge[1])
        )
        if not connection:
            logger.warning(
                f"Connection not possible between {edge[0].name} and {edge[1].name}"
            )
        return connection

    def merge_spaces(self, space_1: "Space", space_2: "Space") -> None:
        internal_elements = nx.shortest_path(self.graph, space_1, space_2)[1:-1]
        merged_space = space_1 + space_2
        merged_space.internal_elements = internal_elements
        self.graph = nx.contracted_nodes(self.graph, merged_space, space_2)

    def assign_nodes_position(self) -> None:
        global_position = generate_normalized_layout(self, scale=self.diagram_scale)
        container_positions = {
            container.name: generate_normalized_layout(
                self,
                scale=container.layout.scale,
                origin=container.layout.bottom_left,
                container_type=container.name,
            )
            for container in self.containers.containers
        }
        for node in self.graph.nodes:
            node.set_position(global_position)
            if node.container_type and container_positions[node.container_type]:
                node.set_position(
                    container_positions[node.container_type], global_=False
                )

    def connect(self) -> None:
        self.assign_nodes_position()
        data_buses = [bus for bus in list(self.graph.nodes) if isinstance(bus, DataBus)]

        data_bus = data_buses[0] if data_buses else None
        new_edges = [edge for edge in self.graph.edges if data_bus not in edge]
        edge_with_databus = [edge for edge in self.graph.edges if data_bus in edge]
        edges_with_bus_without_space = [
            edge
            for edge in edge_with_databus
            if not any(isinstance(e, Space) for e in edge)
        ]
        edges_with_bus_with_space = sorted(
            [
                edge
                for edge in edge_with_databus
                if any(isinstance(e, Space) for e in edge)
            ],
            key=lambda e_: next(e for e in e_ if isinstance(e, Space)).name,
        )
        # Sorting is necessary here since we need to keep the
        # same index for the same space indatabus
        # TODO: not sure where to put this!!!!
        for edge in (
            new_edges + edges_with_bus_without_space + edges_with_bus_with_space
        ):
            self.edge_attributes += self.connect_edges(edge)

    def set_weather_path_to_container_path(self, project_path: Path) -> None:
        for node in self.graph.nodes:
            if (
                isinstance(node, Weather)
                and hasattr(node.parameters, "path")
                and node.parameters.path is not None  # type: ignore
            ):
                # TODO: type ognore needs to be fixed
                old_path = Path(node.parameters.path).resolve()  # type: ignore
                if not old_path.exists():
                    parents = [Path.cwd(), *Path.cwd().parents]
                    for parent in parents:
                        old_path = next(parent.rglob(old_path.name), None)  # type: ignore
                        if old_path and old_path.exists():
                            break
                    if not old_path or not old_path.exists():
                        raise FileNotFoundError(f"File {old_path} not found")
                new_path = project_path.joinpath(old_path.name)
                shutil.copy(old_path, new_path)
                # TODO: this is not correct
                node.parameters.path = f'"/simulation/{old_path.name}"'  # type: ignore

    def model(
        self, include_container: bool = True, data_bus: Optional[DataBus] = None
    ) -> str:
        Space.counter = 0
        for node in self.graph.nodes:
            node.assign_container_type(self)
            node.processing(self)
        if not self.get_node(DataBus):
            data_bus = data_bus or DataBus()
            data_bus.add_to_network(self)
        for node in self.graph.nodes:
            node.configure(self)
        self.connect()
        data = extract_properties(self.library, self.name, self.graph.nodes)
        component_models = []
        for node in self.graph.nodes:
            model = node.model(self)
            if model:
                component_models.append(model)

        container_input = ContainerInput(
            nodes=list(self.graph.nodes),
            connections=self.edge_attributes,
            data=data,
            medium=self.library.medium,
        )
        container_model = self.containers.build(container_input)
        element_models = [c.model for c in component_models]
        environment = Environment(
            trim_blocks=True,
            lstrip_blocks=True,
            loader=FileSystemLoader(str(Path(__file__).parent.joinpath("templates"))),
            autoescape=True,
        )
        environment.filters["frozenset"] = frozenset
        environment.filters["enumerate"] = enumerate
        template = environment.get_template("base.jinja2")
        if not all(n.system_ports_connected() for n in self.graph.nodes):
            raise SystemsNotConnectedError(
                f"""Not all system ports are connected. 
            The following are not connected: {[nm for n in self.graph.nodes for nm in n.non_connected_ports_name() 
                                               if not n.system_ports_connected() and nm]}"""
            )
        return template.render(
            network=self,
            data=data,
            element_models=element_models,
            library=self.library,
            databus=data_bus,
            dynamic_components=self.dynamic_components,
            diagram_size=self.diagram_size,
            containers=container_model if include_container else [],
            main=self.containers.main if include_container else "",
            include_container=include_container,
        )

    def add_boiler_plate_spaces(
        self,
        spaces: list[Space],
        create_internal: bool = True,
        weather: Optional[Weather] = None,
    ) -> None:
        for space in spaces:
            space.add_to_network(self)
        if create_internal:
            for combination in itertools.combinations(spaces, 2):
                self.connect_spaces(*combination)
        weather = weather or Weather()
        weather.position.set_global(-100, 200)
        self.add_node(weather)
        for space in spaces:
            self.connect_system(space, weather)
