"""Containers of the FastModel library: ``building_mpc`` made of one sub-model per container (envelope, emission,
distribution, production, ventilation, solar), as the containers of the Buildings models.

The FastModel templates render ``building_mpc`` as a flat list of declarations; this module groups them:

* the inputs of ``building_mpc`` (its ``FastModel.Interfaces`` connectors: weather data, internal gains, the controls
  of FastModelMPC) and the weather stay at the top of ``building_mpc``: rumoca takes its inputs there, and the
  runnable model keeps its connections and modifiers;
* every other component goes to the container of its element;
* a connection between two containers, or between a container and the top, goes through a port of the container,
  a connector of the type of the component port it stands for (``FastModel.mo`` gives the types);
* a component referring to a variable of another container in its modifiers (``TZon=space_001.TAir``) reads it
  from an input of its container, which the container of the variable gives through an output.
"""

import re
from collections import defaultdict
from functools import cache, partial
from pathlib import Path
from typing import TYPE_CHECKING, cast

from pydantic import BaseModel, Field

from trano.elements.connection import Connection, ContainerConnection
from trano.elements.system import Weather

if TYPE_CHECKING:
    from trano.topology import Network

FAST_MODEL = Path(__file__).parents[1].joinpath("library", "FastModel.mo")
TOP = "top"
CONTAINER_ORDER = ["envelope", "emission", "distribution", "production", "ventilation", "solar", "bus"]
CONTAINER_SIZE = 40  # side of a container in the diagram of building_mpc
CONTAINER_SPACING = 80
PORT_EDGE = 110  # ports on the border of the container diagrams, which hold the components within -100..100
SIGNAL_TYPES = ("FastModel.Interfaces.RealInput", "FastModel.Interfaces.RealOutput")
DESCRIPTIONS = {
    "envelope": "Zones, walls, windows and floors",
    "emission": "Emitters and their controls",
    "distribution": "Water distribution: pumps, valves, junctions and their controls",
    "production": "Heat production and its controls",
    "ventilation": "Ventilation units and ducts",
    "solar": "Photovoltaic production",
    "bus": "Controls",
}


class _Classes(BaseModel):
    """The classes of FastModel.mo: body and enclosing package of each, and the names of the connectors."""

    bodies: dict[str, tuple[str, str]] = Field(default_factory=dict)
    connectors: set[str] = Field(default_factory=set)

    @classmethod
    def parse(cls, text: str) -> "_Classes":
        classes = cls()
        stack: list[tuple[str, int]] = []
        pattern = re.compile(
            r"^ *(?:partial |expandable )?(package|model|block|connector|record|function) (\w+)\b( =)?"
            r"|^ *end (?!(?:if|for|when|while);)(\w+);",
            re.MULTILINE,
        )
        for match in pattern.finditer(text):
            kind, name, short, end = match.groups()
            enclosing = [n for n, _ in stack]
            if name and kind == "connector":
                classes.connectors.add(".".join([*enclosing, name]))  # connector RealInput = input Real too
            if name and not short:
                stack.append((name, match.end()))
            elif end:
                name, start = stack.pop()
                if name != end:
                    raise ValueError(f"FastModel.mo: end {end} closes {name}")
                classes.bodies[".".join([*enclosing[:-1], name])] = (
                    text[start : match.start()],
                    ".".join(enclosing[:-1]),
                )
        return classes

    def resolve(self, name: str, package: str) -> str | None:
        """Qualified name of a class referred to as ``name`` from ``package`` (Modelica lookup, outwards)."""
        parts = package.split(".")
        for depth in range(len(parts), 0, -1):
            candidate = ".".join([*parts[:depth], name])
            if candidate in self.bodies or candidate in self.connectors:
                return candidate
        return None

    def ports(self, qualified: str) -> dict[str, str]:
        """Connector type of every port of a class, inherited ports included."""
        body, package = self.bodies[qualified]
        result: dict[str, str] = {}
        for base in re.findall(r"^\s*extends ([\w.]+)", body, re.MULTILINE):
            if (base_name := self.resolve(base, package)) and base_name in self.bodies:
                result |= self.ports(base_name)
        for type_, name in re.findall(r"^\s*([A-Za-z][\w.]*) (\w+)\b", body, re.MULTILINE):
            if (type_name := self.resolve(type_, package)) and type_name in self.connectors:
                result[name] = type_name
        return result


@cache
def connector_types() -> dict[str, dict[str, str]]:
    """Connector type of every port of the FastModel classes (qualified names), inherited ports included."""
    classes = _Classes.parse(FAST_MODEL.read_text())
    return {qualified: classes.ports(qualified) for qualified in classes.bodies if qualified not in classes.connectors}


def split_declarations(text: str) -> list[str]:
    """The declarations of a rendered element, split at the semicolons outside parentheses and strings."""
    declarations, depth, string, start = [], 0, False, 0
    for index, character in enumerate(text):
        if string:
            if character == '"' and text[index - 1] != "\\":
                string = False
        elif character == '"':
            string = True
        elif character in "({":
            depth += 1
        elif character in ")}":
            depth -= 1
        elif character == ";" and depth == 0:
            declarations.append(text[start:index].strip())
            start = index + 1
    if text[start:].strip():
        declarations.append(text[start:].strip())
    return [declaration for declaration in declarations if declaration]


def _type_and_name(declaration: str) -> tuple[str, str]:
    match = re.match(r"\s*([A-Za-z][\w.]*)\s+(\w+)", declaration)
    if not match:
        raise ValueError(f"Not a component declaration: {declaration[:80]}")
    return match.group(1), match.group(2)


def _identifier(path: str) -> str:
    return re.sub(r"\W+", "_", path).strip("_")


class Declaration(BaseModel):
    text: str
    type_: str
    name: str
    owner: str  # container name or TOP
    location: tuple[float, float] = (0.0, 0.0)  # of its element in the diagram of its container


class BoundaryPort(BaseModel):
    """A port of a container: a connector of the type of the component port, or a signal."""

    name: str
    type_: str
    size: int | None = None
    binding: str | None = None
    side: str = "left"
    position: tuple[float, float] = (0.0, 0.0)

    def declaration(self) -> str:
        size = f"[{self.size}]" if self.size else ""
        binding = f" = {self.binding}" if self.binding else ""
        x, y = self.position
        # as small as the components in the diagram of the container, readable on its icon
        diagram = f"{{{{{x - 5:g},{y - 5:g}}},{{{x + 5:g},{y + 5:g}}}}}"
        icon = f"{{{{{x - 10:g},{y - 10:g}}},{{{x + 10:g},{y + 10:g}}}}}"
        return (
            f"{self.type_} {self.name}{size}{binding}\n"
            f"    annotation (Placement(transformation(extent={diagram}), iconTransformation(extent={icon})));"
        )


class FastModelContainer(BaseModel):
    name: str
    declarations: list[str] = Field(default_factory=list)
    ports: dict[str, BoundaryPort] = Field(default_factory=dict)
    equations: list[str] = Field(default_factory=list)
    position: tuple[float, float] = (0.0, 0.0)

    def add_port(self, port: BoundaryPort) -> BoundaryPort:
        existing = self.ports.setdefault(port.name, port)
        if existing.type_ != port.type_ or existing.size != port.size or existing.binding != port.binding:
            raise ValueError(f"Port {port.name} of container {self.name} declared twice with different types.")
        return existing

    def port_location(self, port: BoundaryPort) -> tuple[float, float]:
        """Location of a port in the diagram of building_mpc."""
        scale = CONTAINER_SIZE / 2 / (PORT_EDGE - 10)
        return self.position[0] + port.position[0] * scale, self.position[1] + port.position[1] * scale

    def model(self, package_name: str) -> str:
        lines = [
            f'model {self.name} "{DESCRIPTIONS.get(self.name, self.name)}"',
            f"  extends {package_name}.Trano.BaseClasses.Containers.{self.name};",
        ]
        lines += [f"  {port.declaration()}" for port in self.ports.values()]
        lines += [f"  {declaration};" for declaration in self.declarations]
        lines.append("equation")
        lines += [f"  {equation}" for equation in self.equations]
        extent = f"{{{{{-PORT_EDGE - 10},{-PORT_EDGE - 10}}},{{{PORT_EDGE + 10},{PORT_EDGE + 10}}}}}"
        lines.append(
            f"  annotation (Diagram(coordinateSystem(extent={extent})), Icon(coordinateSystem(extent={extent})));"
        )
        lines.append(f"end {self.name};")
        return "\n".join(lines)

    def placement(self) -> str:
        x, y = self.position
        half = CONTAINER_SIZE / 2
        return (
            f"Components.Containers.{self.name} {self.name}1 annotation (Placement(transformation("
            f"extent={{{{{x - half:g},{y - half:g}}},{{{x + half:g},{y + half:g}}}}})));"
        )


class FastModelContainers(BaseModel):
    """building_mpc made of containers: the container models, and the declarations and equations of its top."""

    package_name: str
    containers: dict[str, FastModelContainer] = Field(default_factory=dict)
    top_declarations: list[str] = Field(default_factory=list)
    top_equations: list[str] = Field(default_factory=list)
    diagram_extent: str = "{{-200,-100},{200,100}}"

    def models(self) -> list[str]:
        return [container.model(self.package_name) for container in self.containers.values()]

    def declarations(self) -> list[str]:
        return [f"{declaration};" for declaration in self.top_declarations] + [
            container.placement() for container in self.containers.values()
        ]


def _line(start: tuple[float, float], end: tuple[float, float], color: str) -> str:
    middle = (start[0] + end[0]) / 2
    points = [start, (middle, start[1]), (middle, end[1]), end]
    path = ",".join(f"{{{x:g},{y:g}}}" for x, y in points)
    return f"annotation (Line(points={{{path}}}, color={color}))"


def _color(type_: str) -> str:
    if type_.endswith(("WaterInlet", "WaterOutlet", "AirInlet", "AirOutlet")):
        return "{0,127,255}"
    if type_.endswith("HeatPort"):
        return "{191,0,0}"
    if type_.endswith(("Bus", "BusIn", "Inputs", "Gains")):
        return "{255,204,51}"
    return "{0,0,127}"


class _Builder:
    def __init__(self, network: "Network") -> None:
        self.network = network
        self.types = connector_types()
        self.declarations: list[Declaration] = []
        self.containers: dict[str, FastModelContainer] = {}
        self.top_equations: list[str] = []
        self.top_inputs: list[str] = []
        self.owners: dict[str, Declaration] = {}
        self.positions: dict[str, tuple[float, float]] = {}
        # connections of a component to a port of its container: container, equation, component, port
        self.boundaries: list[tuple[str, str, str, str]] = []

    def container_of(self, node: object) -> str:
        container_type = getattr(node, "container_type", None)
        if isinstance(node, Weather) or container_type is None:
            return TOP
        return str(container_type)

    def split(self) -> None:
        for node in self.network.graph.nodes:
            model = getattr(node, "component_model", None)
            if model is None:
                continue
            owner = self.container_of(node)
            text = model.model if owner == TOP else model.container
            position = getattr(node, "position", None)
            location = position.container.coordinate() if position and not position.is_container_empty() else (0, 0)
            for declaration in split_declarations(text):
                type_, name = _type_and_name(declaration)
                input_ = type_.startswith("FastModel.Interfaces.")
                self.declarations.append(
                    Declaration(
                        text=declaration,
                        type_=type_,
                        name=name,
                        owner=TOP if input_ else owner,
                        location=location,
                    )
                )
                if input_:
                    self.top_inputs.append(name)
        self.owners = {declaration.name: declaration for declaration in self.declarations}
        order = {name: index for index, name in enumerate(CONTAINER_ORDER)}
        owners = sorted({d.owner for d in self.declarations if d.owner != TOP}, key=lambda n: order.get(n, 99))
        for index, name in enumerate(owners):
            position = (index * CONTAINER_SPACING, 0.0)
            self.containers[name] = FastModelContainer(name=name, position=position)

    def port_type(self, reference: str) -> tuple[str, int | None]:
        """Connector type of a port of a component (``space_001.propsBus[1]``), and its size for a slice."""
        component, port = re.match(r"(\w+)(?:\[[^\]]*\])?\.(\w+)", reference).groups()  # type: ignore[union-attr]
        type_ = self.owners[component].type_
        try:
            connector = self.types[type_][port]
        except KeyError as error:
            raise ValueError(f"Unknown port {port} of {type_} ({reference})") from error
        size = None
        if slice_ := re.search(r"\[(\d+):(\d+)\]", reference):
            size = int(slice_.group(2)) - int(slice_.group(1)) + 1
        return connector, size

    def connect(self, connection: Connection) -> None:
        left, right = connection.left.equation, connection.right.equation
        owner_left = self.owners[re.match(r"\w+", left).group()].owner  # type: ignore[union-attr]
        owner_right = self.owners[re.match(r"\w+", right).group()].owner  # type: ignore[union-attr]
        if owner_left == owner_right == TOP:
            self.top_equations.append(f"connect({left}, {right});")
            return
        if owner_left == owner_right:
            container_connection = ContainerConnection.model_validate(
                connection.model_dump() | {"source": tuple(sorted([left, right]))}
            )
            self.containers[owner_left].equations.append(cast(str, container_connection.equation).strip())
            return
        ends = []
        for reference, owner, other, other_owner in [
            (left, owner_left, right, owner_right),
            (right, owner_right, left, owner_left),
        ]:
            if owner == TOP:
                ends.append(reference)
                continue
            type_, size = self.port_type(reference)
            # a port of the top (the weather, an input) is shared by every component of the container that uses it
            name = _identifier(other if other_owner == TOP else reference)
            container = self.containers[owner]
            container.add_port(BoundaryPort(name=name, type_=type_, size=size, side=self._side(owner, other_owner)))
            boundary = (owner, f"connect({reference}, {name})", re.match(r"\w+", reference).group(), name)  # type: ignore[union-attr]
            if boundary not in self.boundaries:
                self.boundaries.append(boundary)
            ends.append(f"{owner}1.{name}")
        equation = f"connect({ends[0]}, {ends[1]})"  # its line follows, once the ports are placed
        if equation not in self.top_equations:
            self.top_equations.append(equation)

    def _side(self, owner: str, other_owner: str) -> str:
        if other_owner == TOP:
            return "top"
        order = list(self.containers)
        return "left" if order.index(other_owner) < order.index(owner) else "right"

    def references(self) -> None:
        """Variables of another container, or of the top, in the modifiers of the components."""
        owners = self.owners
        for declaration in self.declarations:
            foreign = {name for name, d in owners.items() if d.owner != declaration.owner and name != declaration.name}
            if not foreign:
                continue
            pattern = re.compile(
                r'"[^"]*"|(?<![\w.])(' + "|".join(map(re.escape, sorted(foreign, key=len, reverse=True))) + r")"
                r"((?:\.\w+(?:\[\d+\])?)*)(?![\w(])(?!\s*=[^=])"
            )
            declaration.text = pattern.sub(partial(self._signal, declaration=declaration), declaration.text)

    def _signal(self, match: re.Match[str], declaration: Declaration) -> str:
        if match.group(1) is None:  # a string
            return match.group(0)
        source, path = match.group(1), match.group(2)
        reference = source + path
        name = _identifier(reference)
        source_owner = self.owners[source].owner
        if declaration.owner == TOP:  # the top reads an output of the container of the variable
            container = self.containers[source_owner]
            container.add_port(BoundaryPort(name=name, type_=SIGNAL_TYPES[1], binding=reference, side="top"))
            return f"{source_owner}1.{name}"
        target = self.containers[declaration.owner]
        target.add_port(
            BoundaryPort(name=name, type_=SIGNAL_TYPES[0], side=self._side(declaration.owner, source_owner))
        )
        if source_owner == TOP:
            source_end = reference
        else:
            container = self.containers[source_owner]
            container.add_port(
                BoundaryPort(
                    name=name,
                    type_=SIGNAL_TYPES[1],
                    binding=reference,
                    side=self._side(source_owner, declaration.owner),
                )
            )
            source_end = f"{source_owner}1.{name}"
        equation = f"connect({source_end}, {declaration.owner}1.{name})"
        if equation not in self.top_equations:
            self.top_equations.append(equation)
        return name

    def place_ports(self) -> None:
        for container in self.containers.values():
            sides: dict[str, list[BoundaryPort]] = defaultdict(list)
            for port in container.ports.values():
                sides[port.side].append(port)
            for side, ports in sides.items():
                step = 2 * (PORT_EDGE - 20) / max(len(ports), 1)
                for index, port in enumerate(ports):
                    offset = PORT_EDGE - 20 - step * (index + 0.5)
                    port.position = {
                        "left": (-PORT_EDGE, offset),
                        "right": (PORT_EDGE, offset),
                        "top": (-offset, PORT_EDGE),
                    }[side]

    def place_top(self) -> list[str]:
        """Declarations of the top: the inputs in a column on the left, the weather above the containers."""
        top = [d for d in self.declarations if d.owner == TOP]
        inputs = [d for d in top if d.name in self.top_inputs]
        others = [d for d in top if d.name not in self.top_inputs]
        left = float(-CONTAINER_SPACING)
        declarations = []
        for index, declaration in enumerate(inputs):
            y = 60 - 20 * index
            declarations.append(
                self._placed(declaration.text, f"{{{{{left - 10},{y - 10}}},{{{left + 10},{y + 10}}}}}")
            )
        for index, declaration in enumerate(others):
            x = index * CONTAINER_SPACING
            declarations.append(self._placed(declaration.text, f"{{{{{x - 10},{50}}},{{{x + 10},{70}}}}}"))
        self.positions = {d.name: (left, 60.0 - 20 * i) for i, d in enumerate(inputs)} | {
            d.name: (float(i * CONTAINER_SPACING), 60.0) for i, d in enumerate(others)
        }
        return declarations

    @staticmethod
    def _placed(declaration: str, extent: str) -> str:
        declaration = re.sub(r"\s*annotation\s*\(.*\)\s*$", "", declaration, flags=re.DOTALL)
        return f"{declaration}\n    annotation (Placement(transformation(extent={extent})))"

    def lines(self) -> list[str]:
        """The top equations with the lines between the ports of the containers."""
        ports = {
            f"{container.name}1.{port.name}": (container, port)
            for container in self.containers.values()
            for port in container.ports.values()
        }
        equations = []
        for equation in self.top_equations:
            if not equation.startswith("connect(") or equation.endswith(";"):
                equations.append(equation)
                continue
            ends = [end.strip() for end in equation[len("connect(") : -1].split(",")]
            locations, color = [], "{0,0,127}"
            for end in ends:
                if end in ports:
                    container, port = ports[end]
                    locations.append(container.port_location(port))
                    color = _color(port.type_)
                else:
                    locations.append(self.positions.get(re.match(r"\w+", end).group(), (0.0, 0.0)))  # type: ignore[union-attr]
            equations.append(f"{equation}\n    {_line(locations[0], locations[1], color)};")
        return equations

    def build(self) -> FastModelContainers:
        self.split()
        for connection in self.network.edge_attributes:
            self.connect(connection)
        self.references()
        for declaration in self.declarations:
            if declaration.owner == TOP:
                continue
            text = declaration.text
            if "annotation" not in text:  # a second component of an element (its controller): placed below it
                x, y = declaration.location
                text = self._placed(text, f"{{{{{x - 5:g},{y - 17:g}}},{{{x + 5:g},{y - 7:g}}}}}")
            self.containers[declaration.owner].declarations.append(text)
        self.place_ports()
        for owner, equation, component, port in self.boundaries:
            container = self.containers[owner]
            start = self.owners[component].location
            container.equations.append(
                f"{equation}\n    {_line(start, container.ports[port].position, _color(container.ports[port].type_))};"
            )
        top_declarations = self.place_top()
        width = CONTAINER_SPACING * max(len(self.containers), 1)
        return FastModelContainers(
            package_name=self.network.name,
            containers=self.containers,
            top_declarations=top_declarations,
            top_equations=self.lines(),
            diagram_extent=f"{{{{{-CONTAINER_SPACING - 20},{-40}}},{{{width},{80}}}}}",
        )


def build_containers(network: "Network") -> FastModelContainers:
    """Group the rendered components of a FastModel network into containers (after ``node.model`` ran)."""
    return _Builder(network).build()
