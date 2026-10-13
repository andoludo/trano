"""Multilayer zones of the ``mpc`` library: the physics of ``Buildings.ThermalZones.Detailed.MixedAir``,
assembled from ``Trano.MPC.Components`` so that it translates into an explicit CasADi ODE.

Every construction the Buildings zone receives (``datConExt``, ``datConExtWin``, ``datConBou``, from the
same grouping of the trano envelope) becomes an element of the zone:

* conduction through the layers: each layer split into ``cells_per_layer`` cells, with a capacitive node
  at every cell boundary (states at both surfaces, as ``stateAtSurface`` in Buildings); the outer node of
  a construction whose outer surface is held at a temperature (floor on ground) is dropped;
* exterior surface: natural and wind-driven convection (``TemperatureWind``), long-wave exchange with sky
  and ground, absorbed solar irradiance (direct + Perez diffuse) of the orientation;
* interior surface: natural convection (``Temperature`` correlations of walls, floors and ceilings) and
  long-wave exchange through a radiant star node. The star is the exact equivalent of Buildings' linearized
  radiosity exchange with view factors ``A_j/sum(A)``: conductance ``4*sigma*T0^3*epsilon_i*A_i`` per surface;
  radiative gains injected in the star reach the surfaces in proportion to ``epsilon_i*A_i``, as in Buildings;
* windows: panes as capacitive nodes with the EN 673 gap conductances of trano's glazing model, the solar
  radiation transmitted and absorbed from the optics of trano (Buildings algorithm), fitted in the cosine of
  incidence; the transmitted radiation reaches the surfaces with the factors of Buildings'
  ``SolarRadiationExchange`` (direct radiation on the floors first); the frame is a conductance;
* the air: one capacity ``rho*cp*V``, infiltration of outdoor air at the air change rate of the
  ``infiltration`` variant, internal and emitter gains split between air and star.
"""

import math
import re
import warnings
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from pydantic import BaseModel, ConfigDict, Field

from trano.mpc.components.model import ConnectedModel, Instance, TopInput
from trano.mpc.estimation import sanitize_name

if TYPE_CHECKING:
    from trano.elements.construction import Construction, Glass
    from trano.mpc.components.hydronics import HydronicSettings
    from trano.elements.envelope import BaseInternalElement, BaseSimpleWall
    from trano.elements.space import Space
    from trano.topology import Network

STEFAN_BOLTZMANN = 5.670374419e-8
T0 = 293.15  # [K] linearization temperature of the interior long-wave exchange (Buildings)
AIR_DENSITY = 1.2  # [kg/m3]
AIR_HEAT_CAPACITY = 1006.0  # [J/(kg.K)]
DEFAULT_RADIANT_FRACTION = 0.3  # radiators (fraction_radiant_heat_transfer)
DEFAULT_INFILTRATION = 0.15  # [1/h] ACH of Trano.ThermalZones.MixedAirInf
OUTDOOR = "outdoor"
WEATHER_INPUTS = (
    TopInput(name="TOut", unit="K", description="Outdoor dry-bulb air temperature"),
    TopInput(name="TBlaSky", unit="K", description="Black-body sky temperature"),
    TopInput(name="winSpe", unit="m/s", description="Wind speed"),
    TopInput(name="winDir", unit="rad", description="Wind direction, 0 from the north"),
)


class Orientation(BaseModel):
    """Orientation of a surface [rad], with the names of its irradiance inputs."""

    model_config = ConfigDict(frozen=True)

    azimuth: float
    tilt: float

    @property
    def name(self) -> str:
        azimuth = round(math.degrees(self.azimuth)) % 360
        return f"azi{azimuth}_til{round(math.degrees(self.tilt))}".replace("-", "m")

    @property
    def direct(self) -> str:
        return f"HDir_{self.name}"

    @property
    def diffuse(self) -> str:
        return f"HDif_{self.name}"

    @property
    def cosine(self) -> str:
        return f"cosInc_{self.name}"


class Convection(BaseModel):
    """Coefficients of the natural convection laws of Buildings for a construction tilt."""

    c_pos: float
    c_neg: float

    @classmethod
    def for_tilt(cls, tilt: float) -> "Convection":
        if abs(math.sin(tilt)) < 1e-9 and math.cos(tilt) > 0:  # ceiling (roof)
            return cls(c_pos=0.76, c_neg=1.51)
        if abs(math.sin(tilt)) < 1e-9 and math.cos(tilt) < 0:  # floor
            return cls(c_pos=1.51, c_neg=0.76)
        return cls(c_pos=1.3, c_neg=1.3)


class MultilayerSettings(BaseModel):
    model_config = ConfigDict(frozen=True)

    cells_per_layer: int = Field(1, ge=1, description="Cells of each construction layer")
    frame_exterior_coefficient: float = Field(25.0, gt=0, description="Frame exterior film coefficient")
    frame_interior_coefficient: float = Field(7.7, gt=0, description="Frame interior film coefficient")
    glass_specific_heat: float = Field(720.0, gt=0, description="Glass heat capacity when the material has none")


# --------------------------------------------------------------------------------------------- geometry
@dataclass(frozen=True)
class OpaqueElement:
    """An opaque construction of a zone (Buildings datConExt, the opaque part of datConExtWin, datConBou)."""

    name: str
    construction: "Construction"
    area: float
    azimuth: float
    tilt: float
    boundary: str  # exterior, ground, outdoor_air, adiabatic or internal
    ground_temperature: float | None = None
    side: str = "b"  # internal elements: side of the zone, a (outer layer first) or b (the room side of others)


INTERNAL_EMISSIVITY = 0.84  # Buildings OpaqueSurface absIR and absSol, which trano leaves at their defaults


@dataclass(frozen=True)
class WindowElement:
    name: str
    glazing: "Glass"
    area: float  # window area, glass and frame
    frame_fraction: float
    azimuth: float
    tilt: float


def _tilt(element: "BaseSimpleWall", space_name: str) -> float:
    from trano.elements.types import TILT_MAPPING

    return math.radians(TILT_MAPPING[element.get_tilt(space_name).value])


def _construction(element: "BaseSimpleWall") -> "Construction":
    from trano.elements.construction import Construction

    if not isinstance(element.construction, Construction):
        raise TypeError(f"{element.name} needs an opaque construction, not {type(element.construction).__name__}")
    return element.construction


def internal_side(element: "BaseInternalElement", space: "Space", network: "Network") -> str:
    """Side of ``space`` on an internal element: ``a`` for the first of its two zones in the network.

    Buildings connects the ``port_a`` of the ``MultiLayer`` conduction to the zone met first in the graph, so the
    layers run from that zone to the other one.
    """
    from trano.elements.space import Space

    first = next(
        node for node in network.graph.nodes if isinstance(node, Space) and element in network.graph.succ[node]
    )
    return "a" if first is space else "b"


def internal_name(element: "BaseInternalElement") -> str:
    return sanitize_name(str(element.name))


def zone_elements(space: "Space", network: "Network") -> tuple[list[OpaqueElement], list[WindowElement]]:
    """The constructions of the Buildings zone of ``space``, grouped as trano groups them for MixedAir."""
    from trano.elements.envelope import (
        AdiabaticWall,
        BaseInternalElement,
        ExternalWall,
        FloorOnGround,
        _group,
        hosted_windows,
    )

    neighbors = list(network.graph.neighbors(space))
    opaque: list[OpaqueElement] = []
    windows: list[WindowElement] = []
    host_names: set[str] = set()
    for index, group in enumerate(hosted_windows(neighbors), start=1):
        host = group.host_walls[0]
        host_names |= {wall.name for wall in group.host_walls if wall.name}
        for glazing_index, glazing_windows in enumerate(
            _group(group.windows, lambda a, b: a.construction == b.construction), start=1
        ):
            window_area = sum(window.surface for window in glazing_windows)
            gross_area = group.gross_area * window_area / group.window_area
            name = f"winWal{index}_{glazing_index}"
            opaque.append(
                OpaqueElement(
                    name=name,
                    construction=_construction(host),
                    area=gross_area - window_area,
                    azimuth=host.azimuth,
                    tilt=_tilt(host, space.name),
                    boundary="exterior",
                )
            )
            windows.append(
                WindowElement(
                    name=f"win{index}_{glazing_index}",
                    glazing=glazing_windows[0].construction,
                    area=window_area,
                    frame_fraction=sum(w.surface * w.frame_fraction for w in glazing_windows) / window_area,
                    azimuth=host.azimuth,
                    tilt=_tilt(host, space.name),
                )
            )
    walls = [n for n in neighbors if isinstance(n, ExternalWall) and n.name not in host_names]
    for index, wall in enumerate(walls, start=1):
        opaque.append(
            OpaqueElement(
                name=f"ext{index}",
                construction=_construction(wall),
                area=float(wall.surface),
                azimuth=wall.azimuth,
                tilt=_tilt(wall, space.name),
                boundary="exterior",
            )
        )
    floors = [n for n in neighbors if isinstance(n, FloorOnGround)]
    for index, floor in enumerate(floors, start=1):
        outdoor = getattr(floor, "variant", "default") == "outdoor_air"
        opaque.append(
            OpaqueElement(
                name=f"flo{index}",
                construction=_construction(floor),
                area=float(floor.surface),
                azimuth=floor.azimuth,
                tilt=_tilt(floor, space.name),
                boundary="outdoor_air" if outdoor else "ground",
                ground_temperature=None if outdoor else floor.ground_temperature,
            )
        )
    adiabatic = [n for n in neighbors if isinstance(n, AdiabaticWall)]
    for index, adiabatic_wall in enumerate(adiabatic, start=1):
        opaque.append(
            OpaqueElement(
                name=f"adi{index}",
                construction=_construction(adiabatic_wall),
                area=float(adiabatic_wall.surface),
                azimuth=adiabatic_wall.azimuth,
                tilt=_tilt(adiabatic_wall, space.name),
                boundary="adiabatic",
            )
        )
    internal = [n for n in neighbors if isinstance(n, BaseInternalElement)]
    opaque.extend(
        OpaqueElement(
            name=internal_name(element),
            construction=_construction(element),
            area=float(element.surface),
            azimuth=element.azimuth,
            tilt=_tilt(element, space.name),
            boundary="internal",
            side=internal_side(element, space, network),
        )
        for element in internal
    )
    return opaque, windows


# ------------------------------------------------------------------------------------------- the layers
class LayerChain(BaseModel):
    """Nodes of a construction from the outer surface (side a) to the inner surface (side b)."""

    capacities: list[float]  # [J/K] per node
    resistances: list[float]  # [K/W] between consecutive nodes


def layer_chain(construction: "Construction", area: float, cells_per_layer: int) -> LayerChain:
    capacities = [0.0]
    resistances: list[float] = []
    for layer in construction.layers:
        material = layer.material
        cell = layer.thickness / cells_per_layer
        cell_capacity = material.density * material.specific_heat_capacity * cell * area
        cell_resistance = cell / material.thermal_conductivity / area
        for _ in range(cells_per_layer):
            capacities[-1] += cell_capacity / 2
            capacities.append(cell_capacity / 2)
            resistances.append(cell_resistance)
    return LayerChain(capacities=capacities, resistances=resistances)


# ------------------------------------------------------------------------------------------- windows
def _polynomial(values: np.ndarray) -> list[float]:
    """Coefficients a0..a4 of a polynomial in the cosine of incidence through the angular values."""
    from trano.elements.glazing import INCIDENCE_ANGLES

    cosines = np.cos(INCIDENCE_ANGLES)
    return [float(value) for value in np.polynomial.polynomial.polyfit(cosines, values, 4)]


def window_optics(glazing: "Glass") -> dict[str, float | list[float]]:
    """Parameters of ``Envelope.WindowSolar``: polynomial fits of the angular optics of trano (Buildings)."""
    from trano.elements.glazing import N_ANGLES

    optics = glazing.properties.optics
    transmittance = np.asarray(optics.transmittance)
    absorptances = np.asarray(optics.absorptances)  # one row per pane, outside first
    tau = _polynomial(transmittance[:N_ANGLES])
    panes = [_polynomial(row[:N_ANGLES]) for row in absorptances]
    return {
        **{f"tau{i}": value for i, value in enumerate(tau)},
        "tauDif": float(transmittance[N_ANGLES]),
        **{f"a{i}": [pane[i] for pane in panes] for i in range(5)},
        "aDif": [float(row[N_ANGLES]) for row in absorptances],
    }


# ------------------------------------------------------------------------------------------- assembly
class _Surface(BaseModel):
    """A surface seen from the room: its node, its area and its emissivity and solar absorptance."""

    port: str
    area: float
    emissivity: float
    absorptance: float
    transmittance: float = 0.0
    is_floor: bool = False


class _Zone:
    def __init__(
        self, space: "Space", network: "Network", settings: MultilayerSettings, hydronic: bool = False
    ) -> None:
        self.space = space
        self.hydronic = hydronic  # heated by the radiators of a hydronic circuit instead of the QHea input
        self.name = sanitize_name(space.name)
        self.network = network
        self.settings = settings
        self.instances: list[Instance] = []
        self.connections: list[tuple[str, str]] = []
        self.surfaces: list[_Surface] = []
        self.orientations: set[Orientation] = set()
        self.air = ""

    def n(self, suffix: str) -> str:
        return f"{self.name}_{suffix}"

    def add(self, name: str, component: str, description: str = "", **kwargs: object) -> str:
        self.instances.append(
            Instance(name=name, component=component, description=description, zone=self.name, **kwargs)
        )
        return name

    def connect(self, a: str, b: str) -> None:
        self.connections.append((a, b))

    # ---------------------------------------------------------------- pieces
    def capacitor(self, name: str, capacity: float, description: str) -> str:
        return (
            self.add(
                name,
                "Thermal.HeatCapacitor",
                description,
                parameters={"C": capacity, "T0": float(self.space.parameters.temperature_initial)},  # type: ignore[union-attr]
            )
            + ".port"
        )

    def resistor(self, name: str, resistance: float, a: str, b: str) -> None:
        self.add(name, "Thermal.ThermalResistor", parameters={"R": resistance})
        self.connect(a, f"{name}.port_a")
        self.connect(f"{name}.port_b", b)

    def conductor(self, name: str, conductance: float, a: str, b: str) -> None:
        self.add(name, "Thermal.ThermalConductor", parameters={"G": conductance})
        self.connect(a, f"{name}.port_a")
        self.connect(f"{name}.port_b", b)

    def interior_surface(self, element: str, port: str, area: float, tilt: float) -> None:
        coefficient = getattr(self.space.parameters, "interior_convection_coefficient", None)
        name = self.n(f"{element}_conInt")
        if coefficient is not None:
            self.conductor(name, coefficient * area, port, self.air)
            return
        convection = Convection.for_tilt(tilt)
        self.add(
            name,
            "Envelope.InteriorConvection",
            parameters={"A": area, "cPos": convection.c_pos, "cNeg": convection.c_neg},
        )
        self.connect(f"{name}.solid", port)
        self.connect(f"{name}.fluid", self.air)

    def exterior_surface(self, element: OpaqueElement, port: str, emissivity: float) -> None:
        """Outer opaque surface: Buildings ``TemperatureWind`` convection and long-wave exchange with sky and ground."""
        coefficient = getattr(self.space.parameters, "exterior_convection_coefficient", None)
        name = self.n(f"{element.name}_conExt")
        if coefficient is not None:
            self.conductor(name, coefficient * element.area, port, f"{OUTDOOR}.port")
        else:
            # Buildings evaluates the exterior surface with the tilt of its outer face, pi - tilt:
            # the outside of a roof is an upward-facing (floor) surface.
            convection = Convection.for_tilt(math.pi - element.tilt)
            self.add(
                name,
                "Envelope.ExteriorConvection",
                parameters={
                    "A": element.area,
                    "cPos": convection.c_pos,
                    "cNeg": convection.c_neg,
                    "azi": element.azimuth,
                },
                inputs={"v": "winSpe", "dir": "winDir"},
            )
            self.connect(f"{name}.solid", port)
            self.connect(f"{name}.fluid", f"{OUTDOOR}.port")
        self.sky(element.name, port, element.area, element.tilt, emissivity)

    def window_exterior(self, element: str, port: str, area: float, tilt: float, emissivity: float) -> None:
        """Outer glass surface: Buildings window convection A*(4 + 4*|v|) and long-wave exchange with sky and ground."""
        name = self.n(f"{element}_conExt")
        self.add(name, "Envelope.WindowExteriorConvection", parameters={"A": area}, inputs={"v": "winSpe"})
        self.connect(f"{name}.solid", port)
        self.connect(f"{name}.fluid", f"{OUTDOOR}.port")
        self.sky(element, port, area, tilt, emissivity)

    def sky(self, element: str, port: str, area: float, tilt: float, emissivity: float) -> None:
        name = self.add(
            self.n(f"{element}_sky"),
            "Envelope.SkyRadiation",
            parameters={"k": 4 * STEFAN_BOLTZMANN * emissivity * area, "vieFacSky": (math.pi - tilt) / math.pi},
            inputs={"TOut": "TOut", "TBlaSky": "TBlaSky"},
        )
        self.connect(f"{name}.port", port)

    # ---------------------------------------------------------------- elements
    def opaque(self, element: OpaqueElement) -> None:
        if element.boundary == "internal":
            self.internal(element)
            return
        chain = layer_chain(element.construction, element.area, self.settings.cells_per_layer)
        layers = element.construction.layers
        nodes: list[str] = []
        for index, capacity in enumerate(chain.capacities):
            if index == 0 and element.boundary in ("ground", "outdoor_air"):
                continue  # outer surface held at a temperature: no state there
            nodes.append(self.capacitor(self.n(f"{element.name}_T{index}"), capacity, f"{element.name}, node {index}"))
        if element.boundary == "ground":
            outer = self.add(
                self.n(f"{element.name}_gro"),
                "Sources.FixedTemperature",
                "Ground under the floor",
                parameters={"T": float(element.ground_temperature or 283.15)},
            )
            ports = [f"{outer}.port", *nodes]
        elif element.boundary == "outdoor_air":
            ports = [f"{OUTDOOR}.port", *nodes]
        else:
            ports = nodes
        for index, resistance in enumerate(chain.resistances):
            self.resistor(self.n(f"{element.name}_R{index}"), resistance, ports[index], ports[index + 1])
        if element.boundary == "exterior":
            orientation = Orientation(azimuth=element.azimuth, tilt=element.tilt)
            self.orientations.add(orientation)
            outer_material = layers[0].material
            self.exterior_surface(element, nodes[0], float(outer_material.longwave_emissivity))
            solar = self.add(
                self.n(f"{element.name}_sol"),
                "Sources.ScaledHeatFlow",
                "Solar irradiance absorbed by the exterior surface",
                parameters={"k": outer_material.shortwave_emissivity * element.area},
                inputs={"u": f"{orientation.direct} + {orientation.diffuse}"},
            )
            self.connect(f"{solar}.port", nodes[0])
        inner_material = layers[-1].material
        self.room_surface(
            element,
            nodes[-1],
            float(inner_material.longwave_emissivity),
            float(inner_material.shortwave_emissivity),
        )

    def internal(self, element: OpaqueElement) -> None:
        """One face of a wall or floor between two zones (Buildings ``surBou`` with a ``MultiLayer`` conduction).

        The zone on side ``a`` builds the conduction, named after the element so that both zones share it.
        """
        chain = layer_chain(element.construction, element.area, self.settings.cells_per_layer)
        prefix = element.name
        if element.side == "a":
            nodes = [
                self.capacitor(f"{prefix}_T{index}", capacity, f"{element.name}, node {index}")
                for index, capacity in enumerate(chain.capacities)
            ]
            for index, resistance in enumerate(chain.resistances):
                self.resistor(f"{prefix}_R{index}", resistance, nodes[index], nodes[index + 1])
        port = f"{prefix}_T{0 if element.side == 'a' else len(chain.capacities) - 1}.port"
        self.room_surface(element, port, INTERNAL_EMISSIVITY, INTERNAL_EMISSIVITY, name=f"{prefix}_{element.side}")

    def room_surface(
        self, element: OpaqueElement, port: str, emissivity: float, absorptance: float, name: str | None = None
    ) -> None:
        """Interior convection of a surface, and its part in the long-wave and solar exchanges of the room."""
        self.interior_surface(name or element.name, port, element.area, element.tilt)
        self.surfaces.append(
            _Surface(
                port=port,
                area=element.area,
                emissivity=emissivity,
                absorptance=absorptance,
                is_floor=abs(math.sin(element.tilt)) < 1e-9 and math.cos(element.tilt) < 0,
            )
        )

    def window(self, window: WindowElement) -> None:
        from trano.elements.construction import GasLayer, GlassLayer

        glazing = window.glazing
        properties = glazing.properties
        panes = [layer for layer in glazing.layers if isinstance(layer, GlassLayer)]
        glass_area = window.area * (1 - window.frame_fraction)
        frame_area = window.area * window.frame_fraction
        orientation = Orientation(azimuth=window.azimuth, tilt=window.tilt)
        self.orientations.add(orientation)
        nodes = []
        for index, pane in enumerate(panes, start=1):
            specific_heat = pane.material.specific_heat_capacity or self.settings.glass_specific_heat
            capacity = pane.material.density * specific_heat * pane.thickness * glass_area
            nodes.append(self.capacitor(self.n(f"{window.name}_pan{index}"), capacity, f"{window.name}, pane {index}"))
        gaps = [layer for layer in glazing.layers if isinstance(layer, GasLayer)]
        for index in range(len(gaps)):
            resistance = (
                properties.pane_resistances[index] / 2
                + properties.gap_resistances[index]
                + properties.pane_resistances[index + 1] / 2
            ) / glass_area
            self.resistor(self.n(f"{window.name}_gap{index + 1}"), resistance, nodes[index], nodes[index + 1])
        solar = self.n(f"{window.name}_sol")
        self.add(
            solar,
            "Envelope.WindowSolar",
            "Solar radiation transmitted and absorbed by the glazing",
            parameters={"nPan": len(panes), "AGla": glass_area, **window_optics(glazing)},
            inputs={"HDir": orientation.direct, "HDif": orientation.diffuse, "cosInc": orientation.cosine},
        )
        for index, node in enumerate(nodes, start=1):
            self.connect(f"{solar}.pane[{index}]", node)
        self.window_exterior(
            window.name, nodes[0], glass_area, window.tilt, panes[0].material.infrared_absorptivity_outside_facing
        )
        self.interior_surface(window.name, nodes[-1], glass_area, window.tilt)
        self.surfaces.append(
            _Surface(
                port=nodes[-1],
                area=glass_area,
                emissivity=panes[-1].material.infrared_absorptivity_room_facing,
                absorptance=float(np.asarray(properties.optics.absorptances)[-1, -1]),
                transmittance=float(np.asarray(properties.optics.transmittance)[-1]),
            )
        )
        if frame_area > 0:
            frame_resistance = (
                1 / self.settings.frame_exterior_coefficient
                + 1 / glazing.u_value_frame
                + 1 / self.settings.frame_interior_coefficient
            )
            self.conductor(self.n(f"{window.name}_fra"), frame_area / frame_resistance, f"{OUTDOOR}.port", self.air)

    # ---------------------------------------------------------------- the room
    def build(self) -> None:
        parameters = self.space.parameters
        volume = float(parameters.volume)  # type: ignore[union-attr]
        self.air = self.capacitor(self.n("air"), AIR_DENSITY * AIR_HEAT_CAPACITY * volume, "Indoor air")
        opaque, windows = zone_elements(self.space, self.network)
        for element in opaque:
            self.opaque(element)
        for window in windows:
            self.window(window)
        ach = infiltration_rate(self.space)
        if ach:
            self.conductor(
                self.n("inf"), AIR_DENSITY * AIR_HEAT_CAPACITY * volume * ach / 3600, self.air, f"{OUTDOOR}.port"
            )
        star = self.add(
            self.n("rad"),
            "Thermal.StarNode",
            "Long-wave radiation between the room surfaces (radiant star)",
            parameters={
                "n": len(self.surfaces),
                "G": [4 * STEFAN_BOLTZMANN * T0**3 * surface.emissivity * surface.area for surface in self.surfaces],
            },
        )
        for index, surface in enumerate(self.surfaces, start=1):
            self.connect(f"{star}.port[{index}]", surface.port)
        self._gains(star)
        self._solar_distribution(windows)

    def _gains(self, star: str) -> None:
        internal_radiant = occupancy_radiant_fraction(self.space, self.network)
        self._split_input(f"{self.name}_QInt", "intGai", internal_radiant, star)
        if not self.hydronic:
            self._split_input(f"{self.name}_QHea", "hea", emitter_radiant_fraction(self.space), star)

    def _split_input(self, signal: str, name: str, radiant: float, star: str) -> None:
        if radiant < 1:
            self.add(
                self.n(f"{name}Con"),
                "Sources.ScaledHeatFlow",
                f"Convective part of {signal}",
                parameters={"k": 1 - radiant},
                inputs={"u": signal},
            )
            self.connect(f"{self.n(name)}Con.port", self.air)
        if radiant > 0:
            self.add(
                self.n(f"{name}Rad"),
                "Sources.ScaledHeatFlow",
                f"Radiant part of {signal}",
                parameters={"k": radiant},
                inputs={"u": signal},
            )
            self.connect(f"{self.n(name)}Rad.port", f"{star}.node")

    def _solar_distribution(self, windows: list[WindowElement]) -> None:
        if not windows:
            return
        k_dir, k_dif = solar_distribution(self.surfaces)
        transmitted_direct = " + ".join(f"{self.n(window.name)}_sol.QTraDir" for window in windows)
        transmitted_diffuse = " + ".join(f"{self.n(window.name)}_sol.QTraDif" for window in windows)
        name = self.add(
            self.n("solDis"),
            "Envelope.SolarDistribution",
            "Transmitted solar radiation absorbed by the room surfaces",
            parameters={"n": len(self.surfaces), "kDir": k_dir, "kDif": k_dif},
            inputs={"QDir": transmitted_direct, "QDif": transmitted_diffuse},
        )
        for index, surface in enumerate(self.surfaces, start=1):
            self.connect(f"{name}.surface[{index}]", surface.port)


def solar_distribution(surfaces: list[_Surface]) -> tuple[list[float], list[float]]:
    """Absorbed shares of the transmitted direct and diffuse radiation (Buildings SolarRadiationExchange).

    The share of the windows (transmittance) leaves the room again; their absorptance heats the inner pane.
    """
    eps_tau_area = [(s.absorptance + s.transmittance) * s.area for s in surfaces]
    total = sum(eps_tau_area)
    k_dif = [s.absorptance * s.area / total for s in surfaces]
    floor_area = sum(s.area for s in surfaces if s.is_floor)
    if floor_area == 0:
        return k_dif, k_dif
    reflected = sum(s.area * (1 - s.absorptance - s.transmittance) for s in surfaces if s.is_floor) / floor_area
    others = sum(value for value, s in zip(eps_tau_area, surfaces, strict=True) if not s.is_floor)
    k_dir = [
        s.absorptance * s.area / floor_area if s.is_floor else reflected / others * s.absorptance * s.area
        for s in surfaces
    ]
    return k_dir, k_dif


def infiltration_rate(space: "Space") -> float:
    """Air change rate [1/h] of the ``infiltration`` variant, as ``Trano.ThermalZones.MixedAirInf`` (the zone of the
    Buildings library) takes it: the ``ach`` of the space, 0.15 by default; none for the other variants."""
    if getattr(space, "variant", "default") != "infiltration":
        return 0.0
    ach = getattr(space.parameters, "ach", None)
    return DEFAULT_INFILTRATION if ach is None else float(ach)


def occupancy_radiant_fraction(space: "Space", network: "Network") -> float:
    """Radiant share of the sensible occupancy gains, from the gain vector (radiant; convective; latent)."""
    if not space.occupancy:
        return 0.0
    gain = space.occupancy.processed_parameters(network.library).get("gain", "[40; 75; 40]")
    values = [float(value) for value in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", str(gain))]
    if len(values) < 2 or values[0] + values[1] <= 0:
        return 0.0
    return values[0] / (values[0] + values[1])


def emitter_radiant_fraction(space: "Space") -> float:
    """Radiant share of the heat of the emitters of a zone (radiators 0.3 by default, ideal systems 0)."""
    from trano.elements.system import Radiator

    fractions = [
        getattr(emission.parameters, "fraction_radiant_heat_transfer", None) or DEFAULT_RADIANT_FRACTION
        for emission in space.emissions
        if isinstance(emission, Radiator)
    ]
    return float(fractions[0]) if fractions else 0.0


def zone_orientations(network: "Network") -> list[Orientation]:
    """Orientations of the exterior surfaces of the multilayer zones and of the PV arrays, in the order of their
    inputs."""
    from trano.elements.space import Space
    from trano.mpc.components.photovoltaics import array_orientation, photovoltaic_arrays

    orientations: set[Orientation] = {array_orientation(array) for array in photovoltaic_arrays(network)}
    for space in network.graph.nodes:
        if isinstance(space, Space):
            opaque, windows = zone_elements(space, network)
            orientations |= {Orientation(azimuth=e.azimuth, tilt=e.tilt) for e in opaque if e.boundary == "exterior"}
            orientations |= {Orientation(azimuth=w.azimuth, tilt=w.tilt) for w in windows}
    return sorted(orientations, key=lambda o: (o.tilt, o.azimuth))


class UnmodelledSystemWarning(UserWarning):
    """A system of the YAML that the multilayer models of the ``mpc`` library leave out."""


def _warn_unmodelled(network: "Network", modelled: set[object]) -> None:
    from trano.elements.system import BaseOccupancy, Emission, Sensor, System, Weather
    from trano.mpc.components.hydronics import hydronic_graph

    covered = (Weather, BaseOccupancy, Sensor)
    left_out = sorted(
        str(node.name)
        for node in network.graph.nodes
        if isinstance(node, System)
        and not isinstance(node, covered)
        and node not in modelled
        # an emitter outside the water circuits is an ideal emitter: the heat input of its zone
        and not (isinstance(node, Emission) and node not in hydronic_graph(network))
    )
    if left_out:
        warnings.warn(
            f"The multilayer mpc model leaves out {', '.join(left_out)}: not modelled by Trano.MPC yet.",
            UnmodelledSystemWarning,
            stacklevel=3,
        )


def multilayer_model(
    network: "Network", settings: MultilayerSettings | None = None, hydronics: "HydronicSettings | None" = None
) -> ConnectedModel:
    """``building_mpc`` of a network with multilayer zones and, when it has them, its hydronic circuits, ventilation
    units and PV arrays. Systems it does not model raise an ``UnmodelledSystemWarning``."""
    from trano.elements.space import Space
    from trano.elements.system import Duct
    from trano.mpc.components.hydronics import hydronic_circuit, hydronic_emitters, hydronic_graph, zone_of
    from trano.mpc.components.photovoltaics import photovoltaic_arrays, photovoltaic_instance
    from trano.mpc.components.ventilation import ventilation_systems, ventilation_units

    settings = settings or MultilayerSettings()
    hydronic_zones = {
        sanitize_name(str(space.name))
        for radiator in hydronic_emitters(network)
        if (space := zone_of(radiator, network))
    }
    zones = [
        _Zone(space, network, settings, hydronic=sanitize_name(space.name) in hydronic_zones)
        for space in network.graph.nodes
        if isinstance(space, Space)
    ]
    instances = [
        Instance(
            name=OUTDOOR, component="Sources.PrescribedTemperature", inputs={"T": "TOut"}, description="Outdoor air"
        )
    ]
    connections: list[tuple[str, str]] = []
    for zone in zones:
        zone.build()
        instances += zone.instances
        connections += zone.connections
    circuit = hydronic_circuit(network, lambda zone: (f"{zone}_air.port", f"{zone}_rad.rad"), hydronics)
    ventilation = ventilation_systems(network)
    arrays = photovoltaic_arrays(network)
    instances += circuit.instances + ventilation.instances + [photovoltaic_instance(array) for array in arrays]
    connections += circuit.connections + ventilation.connections
    units = ventilation_units(network)
    ducts = {node for node in network.graph.nodes if isinstance(node, Duct)}
    _warn_unmodelled(network, set(hydronic_graph(network).nodes) | set(units) | ducts | set(arrays))

    orientations = zone_orientations(network)
    inputs = list(WEATHER_INPUTS)
    for orientation in orientations:
        label = f"azimuth {math.degrees(orientation.azimuth):g} deg, tilt {math.degrees(orientation.tilt):g} deg"
        inputs += [
            TopInput(name=orientation.direct, unit="W/m2", description=f"Direct irradiance, {label}"),
            TopInput(name=orientation.diffuse, unit="W/m2", description=f"Diffuse irradiance (Perez), {label}"),
            TopInput(name=orientation.cosine, unit="1", description=f"Cosine of the incidence angle, {label}"),
        ]
    for zone in zones:
        inputs.append(TopInput(name=f"{zone.name}_QInt", unit="W", description="Sensible internal heat gains"))
        if not zone.hydronic:
            inputs.append(
                TopInput(
                    name=f"{zone.name}_QHea",
                    unit="W",
                    description="Heating (> 0) or cooling (< 0) power, control input",
                    control=True,
                    zone=zone.name,
                )
            )
    inputs += circuit.inputs + ventilation.inputs
    parts = [f"{len(circuit.inputs)} hydronic controls"] if circuit.inputs else []
    parts += [f"{len(ventilation.instances)} ventilation units"] if ventilation.instances else []
    parts += [f"{len(arrays)} PV arrays"] if arrays else []
    systems = f" with {', '.join(parts)}" if parts else ""
    return ConnectedModel(
        name="building_mpc",
        description=f"{len(zones)}-zone multilayer building model{systems}, connected components",
        inputs=inputs,
        instances=instances,
        connections=connections,
    )
