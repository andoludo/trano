import itertools
import logging
import shutil

from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Type, Union, get_args

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from jinja2 import Environment, FileSystemLoader
from networkx import DiGraph, shortest_path
from networkx.classes.reportviews import NodeView
from pyvis.network import Network as PyvisNetwork  # type: ignore

from tests.constructions.constructions import Constructions
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
from trano.elements.common_base import BasePosition
from trano.elements.construction import extract_properties
from trano.elements.containers import containers_factory
from trano.elements.control import AhuControl, CollectorControl, VAVControl
from trano.elements.inputs import BaseInputOutput
from trano.elements.space import Space, _get_controllable_element
from trano.elements.system import (
    VAV,
    AirHandlingUnit,
    Boiler,
    Pump,
    System,
    TemperatureSensor,
    ThreeWayValve,
    Valve,
    Ventilation,
    Weather,
)
from trano.elements.types import Tilt, ContainerTypes
from trano.exceptions import WrongSystemFlowError
from trano.library.library import Library
from pydantic import BaseModel
logger = logging.getLogger(__name__)
class ComponentModel(BaseModel):
    id: int
    model: str
    container: str

class Network:  # : PLR0904, #TODO: fix this
    def __init__(
        self,
        name: str,
        library: Optional[Library] = None,
        external_data: Optional[Path] = None,
    ) -> None:
        self.graph: DiGraph = DiGraph()
        self.edge_attributes: List[Connection] = []
        self.name: str = name
        self._system_controls: List[Control] = []
        self.library = library or Library.load_default()
        self.external_data = external_data
        self.containers = containers_factory()
        self.dynamic_components: Dict[DynamicTemplateCategories, List[str]] = {
            "ventilation": [],
            "control": [],
            "boiler": [],
        }

    def get_edge(self, first_edge: Type[BaseElement], second_edge: Type[BaseElement]) -> Connection:
        return next(edge for edge in self.graph.edges if ((isinstance(edge[0], first_edge) and isinstance(edge[1], second_edge)) or (isinstance(edge[1], first_edge) and isinstance(edge[0], second_edge))))
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
        # TODO: this feels like it does not belong here!!!!

        spaces = sorted(
            [node for node in self.graph.nodes if isinstance(node, Space)],
            key=lambda x: x.name,
        )
        controls = sorted(
            [node for node in self.graph.nodes if isinstance(node, Control)],
            key=lambda x: x.name,  # type: ignore
        )
        ahus = sorted(
            [node for node in self.graph.nodes if isinstance(node, AirHandlingUnit)],
            key=lambda x: x.name,  # type: ignore
        )
        data_bus = DataBus(
            name="data_bus",
            spaces=[space.name for space in spaces],
            external_data=self.external_data,
        )
        self.add_node(data_bus)
        for space in spaces:
            self.graph.add_edge(space, data_bus)
            if space.occupancy:
                self.graph.add_edge(space.occupancy, data_bus)
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
                if controllable_element.control and isinstance(
                    controllable_element.control, VAVControl
                ):
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
        if space_1.position.is_empty() or space_2.position.is_empty():
            raise Exception("Position not assigned to spaces")
        internal_element.position.between_two_objects(space_1.position.global_, space_2.position.global_)

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
        self, system_1: System, system_2: System  # :  PLR6301
    ) -> None:
        # TODO: change position to object
        if not system_1.position.is_empty() and not system_2.position:
            position = BasePosition()
            position.set(system_1.position.global_.location.x + 100, system_1.position.global_.location.y - 100)
            system_2.position = position
            if hasattr(system_2, "control") and system_2.control:
                system_2.control.position.set(
                    system_2.position.global_.location.x - 50,
                    system_2.position.global_.location.y
                )
        if not system_1.position.is_empty()  and not system_1.position:
            position = BasePosition()
            position.set_global(system_2.position.global_.location.x - 100, system_2.position.global_.location.y - 100)
            system_1.position = position
            if hasattr(system_1, "control") and system_1.control:
                system_1.control.position.set(
                    system_2.position.global_.location.x - 50,
                    system_2.position.global_.location.y
                )

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
        self._assign_position(system_1, system_2)

    def connect_edges(
        self, edge: Tuple[BaseElement, BaseElement]  # :  PLR6301
    ) -> list[Connection]:
        connection =  connect(ElementPort.from_element(edge[0]), ElementPort.from_element(edge[1]))
        if not connection:
            logger.warning(f"Connection not possible between {edge[0].name} and {edge[1].name}")
        return connection

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
        # Sorting is necessary here since we need to keep the same
        # index for the same space indatabus
        # TODO: not sure where to put this!!!!
        data_bus = next(
            bus for bus in list(self.graph.nodes) if isinstance(bus, DataBus)
        )
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

    def _connect_space_controls(self) -> None:
        undirected_graph = self.graph.to_undirected()
        space_controls = [
            node for node in undirected_graph.nodes if isinstance(node, Space)
        ]
        for space_control in space_controls:
            for system_control in self._system_controls:
                shortest_path(undirected_graph, system_control, space_control)

    def get_ahu_space_elements(self, ahu: AirHandlingUnit) -> List[Space]:
        return [x for x in self._get_ahu_elements(ahu, Space) if isinstance(x, Space)]

    def get_ahu_vav_elements(self, ahu: AirHandlingUnit) -> List[VAV]:
        return [x for x in self._get_ahu_elements(ahu, VAV) if isinstance(x, VAV)]

    def _get_ahu_elements(
        self, ahu: AirHandlingUnit, element_type: Type[Union[VAV, Space]]
    ) -> List[Union[VAV, Space]]:
        elements_: List[Union[VAV, Space]] = []
        elements = [node for node in self.graph.nodes if isinstance(node, element_type)]
        for element in elements:
            try:
                paths = nx.shortest_path(self.graph, ahu, element)
            except Exception as e:
                raise WrongSystemFlowError(
                    "Wrong AHU system configuration flow."
                ) from e
            p = paths[1:-1]
            if p and all(isinstance(p_, Ventilation) for p_ in p):
                elements_.append(element)
        return elements_

    def configure_ahu_control(self) -> None:
        ahus = [node for node in self.graph.nodes if isinstance(node, AirHandlingUnit)]
        for ahu in ahus:
            if ahu.control and isinstance(ahu.control, AhuControl):
                ahu.control.spaces = self.get_ahu_space_elements(ahu)
                ahu.control.vavs = self.get_ahu_vav_elements(ahu)

    def get_linked_valves(self, pump_collector: BaseElement) -> List[Valve]:
        valves_: List[Valve] = []
        valves = [node for node in self.graph.nodes if isinstance(node, Valve)]
        for valve in valves:
            paths = list(nx.all_simple_paths(self.graph, pump_collector, valve))
            for path in paths:
                p = path[1:-1]
                if p and all(isinstance(p_, System) for p_ in p):
                    valves_.append(valve)
                    break
        return valves_

    def configure_collector_control(self) -> None:
        pump_collectors = [
            node
            for node in self.graph.nodes
            if isinstance(node, (Pump, Boiler))
            and isinstance(node.control, CollectorControl)
        ]
        for pump_collector in pump_collectors:
            if isinstance(pump_collector.control, CollectorControl):
                pump_collector.control.valves = self.get_linked_valves(pump_collector)

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

    def assign_container_type(self):
        for node in list(self.graph.nodes):
            if node.container_type is None :
                self.graph.neighbors(node)
                container_types = {s.container_type for s in  list(self.graph.predecessors(node)) + list(self.graph.successors(node)) if s.container_type}
                node.container_type = next(iter(sorted(container_types, key={v: i for i, v in enumerate(get_args(ContainerTypes))}.get)))

    def model(self, include_container=False) -> str:
        Space.counter = 0
        self.assign_container_type()
        self._build_full_space_control()
        data_bus = self._build_data_bus()
        self.configure_ahu_control()
        self.configure_collector_control()

        data_bus.non_connected_ports = get_non_connected_ports(self.graph.nodes)
        self.containers.assign_nodes(self.graph.nodes, self.graph)
        self.generate_graphs()

        self._connect_space_controls() #TODO: What is this for?

        component_models = self.build_element_models()
        self.containers.assign_models(component_models)
        container_model = self.containers.build()
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

        data = extract_properties(self.library, self.name, self.graph.nodes)
        diagram_size = self._get_diagram_size()
        if include_container:
            return template.render(
                network=self,
                data=data,
                element_models=element_models,
                library=self.library,
                databus=data_bus,
                dynamic_components=self.dynamic_components,
                diagram_size=diagram_size,
                containers = container_model,
                main = self.containers.main
            )
        return template.render(
            network=self,
            data=data,
            element_models=element_models,
            library=self.library,
            databus=data_bus,
            dynamic_components=self.dynamic_components,
            diagram_size=diagram_size,
            containers = []
        )

    def _get_diagram_size(self) -> str:
        x = [n.position.global_.location.x for n in list(self.graph.nodes)]
        y = [n.position.global_.location.y for n in list(self.graph.nodes)]
        return f"{{{{{min(x) - 50},{min(y) - 50}}},{{{max(x) + 50},{max(y) + 50}}}}}"

    def build_dynamic_component_template(self, node: BaseElement) -> None:

        if node.component_template:
            component = node.component_template.render(
                self.name, node, node.processed_parameters(self.library)
            )
            if node.component_template.category:
                self.dynamic_components[node.component_template.category].append(
                    component
                )

    def build_element_models(self) -> List[ComponentModel]:
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
            if node.component_template:
                self.build_dynamic_component_template(node)
            package_name = self.name
            library_name = self.library.name
            parameters = node.processed_parameters(self.library)
            rtemplate = environment.from_string(
                "{% import 'macros.jinja2' as macros %}"
                + node.template
                + " "
                + node.position.global_.annotation
            )


            model = rtemplate.render(
                element=node,
                package_name=package_name,
                library_name=library_name,
                parameters=parameters,
            )
            rtemplate = environment.from_string(
                "{% import 'macros.jinja2' as macros %}"
                + node.template
                + " "
                + node.position.container.annotation
            )

            container = rtemplate.render(
                element=node,
                package_name=package_name,
                library_name=library_name,
                parameters=parameters,
            )
            models.append(ComponentModel(id=hash(node), model=model, container=container))
        return models

    def add_boiler_plate_spaces(
        self,
        spaces: list[Space],
        create_internal: bool = True,
        weather: Optional[Weather] = None,
    ) -> None:
        for space in spaces:
            self.add_space(space)
        if create_internal:
            for combination in itertools.combinations(spaces, 2):
                self.connect_spaces(*combination)
        weather = weather or Weather()
        weather.position.set(-100, 200)
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
