"""How the envelope of the YAML lands in the parameters of a Buildings model.

Buildings renders the envelope inside the zone (``Buildings.ThermalZones.Detailed.MixedAir`` or
trano's ``MixedAirInf``): the opaque constructions and the glazing systems are ``parameter``
records of the ``envelope`` container (``building.envelope1.external_wall_001.material[1].k``)
and each zone refers to them from its arrays of constructions:

- ``datConExt[i]``: opaque external walls without windows (``A``, ``azi``, ``til``, ``layers``);
- ``datConExtWin[i]``: one entry per orientation and glazing holding the host walls and their windows
  (``A`` the gross wall area, ``hWin``, ``wWin``, ``fFra``, ``glaSys``);
- ``datConBou[i]``: floors on ground, then adiabatic walls (``A``, ``azi``, ``til``);
- ``surBou[i]``: the internal walls shared with another zone (``A``, ``til``).

The index of a YAML boundary in these arrays follows the order in which the zone meets its
neighbours in the network graph, the order :meth:`trano.elements.space.BaseSpace.get_neighhors`
renders; it is recomputed here from the same graph. A window area becomes a square window
(``hWin = wWin = sqrt(A)``) or, with several windows of one glazing on one orientation, an
area-weighted height and a width: those are ``derived`` parameters. The temperature under a floor
on ground is a ``FixedTemperature`` component of its own (``flooronground_<n>.T``); the exposed
perimeter and the room temperature of the IDEAS ground model are not used.
"""

from collections.abc import Iterator
from typing import Any

from trano.elements.construction import (
    GAS_REFERENCE_PRESSURE,
    GAS_REFERENCE_TEMPERATURE,
    UNIVERSAL_GAS_CONSTANT,
    Construction,
    GasLayer,
    Glass,
    GlassLayer,
)
from trano.elements.envelope import (
    AdiabaticWall,
    BaseSimpleWall,
    BaseWindow,
    ExternalWall,
    FloorOnGround,
    InternalElement,
    WindowedWallParameters,
    hosted_windows,
)
from trano.elements.space import BaseSpace
from trano.exceptions import FmuError
from trano.fmu.inventory import (
    TOP_INSTANCE,
    Location,
    ResolvedParameter,
    Target,
    clean_name,
    find_element,
)
from trano.topology import Network

ENVELOPE = f"{TOP_INSTANCE}.envelope1"
MATERIAL_FIELDS = {"thermal_conductivity": "k", "specific_heat_capacity": "c", "density": "d"}
GLASS_FIELDS = {
    "thermal_conductivity": "k",
    "solar_transmittance": "tauSol",
    "solar_reflectance_outside_facing": "rhoSol_a",
    "solar_reflectance_room_facing": "rhoSol_b",
    "infrared_transmissivity": "tauIR",
    "infrared_absorptivity_outside_facing": "absIR_a",
    "infrared_absorptivity_room_facing": "absIR_b",
}
GLASS_NOT_USED = ("density", "specific_heat_capacity", "longwave_emissivity", "shortwave_emissivity")
GAS_FIELDS = {"thermal_conductivity": "a_k", "specific_heat_capacity": "a_c"}
GAS_NOT_USED = ("longwave_emissivity", "shortwave_emissivity")
FLOOR_NOT_USED = {
    "perimeter": "exposed perimeter of the IDEAS ground model (ISO 13370)",
    "average_indoor_temperature": "room temperature of the IDEAS ground model (ISO 13370)",
}
TILT_NOTE = "tilt word rendered as the constant Buildings.Types.Tilt.<Tilt> [rad]"
UNITS = {
    "thermal_conductivity": "W/(m.K)",
    "specific_heat_capacity": "J/(kg.K)",
    "density": "kg/m3",
    "thickness": "m",
    "surface": "m2",
    "azimuth": "rad",
    "u_value_frame": "W/(m2.K)",
    "ground_temperature": "K",
    "frame_fraction": "1",
}


class ZoneArrays:
    """Index of every boundary of a zone in its arrays of constructions, as the zone template renders them."""

    def __init__(self, space: BaseSpace, network: Network) -> None:
        neighbors = list(network.graph.neighbors(space))
        self.windowed = WindowedWallParameters.from_neighbors(neighbors)
        self.zone = f"{ENVELOPE}.{space.name}"
        self.entries: dict[int, tuple[str, int]] = {}
        self.window_entries: dict[int, int] = {}  # id(window) -> entry; several windows may share one
        hosted = [wall for wall in neighbors if wall.name in self.windowed.included_external_walls]
        external = [w for w in neighbors if isinstance(w, ExternalWall) and w not in hosted]
        floors = [w for w in neighbors if isinstance(w, FloorOnGround)]
        adiabatic = [w for w in neighbors if isinstance(w, AdiabaticWall)]
        internal = [w for w in neighbors if isinstance(w, InternalElement)]
        for array, walls in (("datConExt", external), ("datConBou", floors + adiabatic), ("surBou", internal)):
            for index, wall in enumerate(walls, start=1):
                self.entries[id(wall)] = (array, index)
        entry = 0
        for group in hosted_windows(neighbors):
            glazings: list[list[BaseWindow]] = []
            for window in group.windows:
                same = next((g for g in glazings if g[0].construction == window.construction), None)
                (same.append(window) if same else glazings.append([window]))
            for windows in glazings:
                entry += 1
                for window in windows:
                    self.entries[id(window)] = ("datConExtWin", entry)
                    self.window_entries[id(window)] = len(windows)
                for wall in group.host_walls:
                    self.entries.setdefault(id(wall), ("datConExtWin", entry))
        if space.boundaries is not None:
            rendered = {boundary.type: boundary.number for boundary in space.boundaries}
            counted = {
                "ExternalWall": len(external),
                "FloorOnGround": len(floors) + len(adiabatic),
                "InternalElement": len(internal),
                "WindowedWall": entry,
            }
            if any(rendered.get(kind, 0) != number for kind, number in counted.items()):
                raise FmuError(f"Zone {space.name}: boundary arrays {counted} differ from the rendered {rendered}")

    def path(self, wall: BaseSimpleWall, attribute: str) -> str:
        array, index = self.entries[id(wall)]
        return f"{self.zone}.{array}[{index}].{attribute}"

    def shared_entry(self, window: BaseWindow) -> bool:
        return self.window_entries.get(id(window), 1) > 1


class BuildingsEnvelopeResolver:
    """The :class:`trano.fmu.inventory.EnvelopeResolver` of the Buildings library."""

    def resolve(self, network: Network, data: dict[str, Any]) -> list[ResolvedParameter]:
        if network.library.base_library() != "Buildings":
            raise FmuError(f"The Buildings envelope resolver does not apply to the {network.library.name} library")
        constructions = self._constructions(network)
        glazings = self._glazings(network)
        resolved: list[ResolvedParameter] = []
        resolved += list(self._materials(data, constructions))
        resolved += list(self._glass_materials(data, glazings))
        resolved += list(self._gases(data, glazings))
        resolved += list(self._construction_layers(data, constructions))
        resolved += list(self._glazing_layers(data, glazings))
        resolved += list(self._spaces(network, data))
        resolved += list(self._internal_walls(network, data))
        return resolved

    @staticmethod
    def _constructions(network: Network) -> dict[str, Construction]:
        walls = (node for node in network.graph.nodes if isinstance(node, BaseSimpleWall))
        return {w.construction.name: w.construction for w in walls if isinstance(w.construction, Construction)}

    @staticmethod
    def _glazings(network: Network) -> dict[str, Glass]:
        windows = (node for node in network.graph.nodes if isinstance(node, BaseWindow))
        return {w.construction.name: w.construction for w in windows if isinstance(w.construction, Glass)}

    @staticmethod
    def _record(name: str) -> str:
        return f"{ENVELOPE}.{name}"

    def _materials(self, data: dict[str, Any], constructions: dict[str, Construction]) -> Iterator[ResolvedParameter]:
        for material in data.get("material") or []:
            name = clean_name(material["id"])
            uses = [
                (record, index)
                for record, construction in constructions.items()
                for index, layer in enumerate(construction.layers, start=1)
                if layer.material.name == name
            ]
            for field, attribute in MATERIAL_FIELDS.items():
                if field not in material:
                    continue
                targets = tuple(
                    Target(path=f"{self._record(record)}.material[{index}].{attribute}", value=material[field])
                    for record, index in uses
                )
                yield ResolvedParameter(
                    yaml_path=f"material[{material['id']}].{field}",
                    group="Opaque materials",
                    value=material[field],
                    unit=UNITS.get(field),
                    element=name,
                    targets=targets,
                    note=None if uses else "material not used by any construction of the model",
                )

    def _glass_materials(self, data: dict[str, Any], glazings: dict[str, Glass]) -> Iterator[ResolvedParameter]:
        for glass in data.get("glass_material") or []:
            name = clean_name(glass["id"])
            uses = [
                (record, index)
                for record, glazing in glazings.items()
                for index, layer in enumerate([lay for lay in glazing.layers if isinstance(lay, GlassLayer)], start=1)
                if layer.material.name == name
            ]
            for field, value in glass.items():
                if field == "id":
                    continue
                location = Location(yaml_path=f"glass_material[{glass['id']}].{field}", group="Glazing")
                if field in GLASS_NOT_USED:
                    yield ResolvedParameter(
                        yaml_path=location.yaml_path,
                        group=location.group,
                        value=value,
                        element=name,
                        note="Buildings glass panes carry no thermal mass: not in Buildings.HeatTransfer.Data.Glasses",
                    )
                    continue
                attribute = GLASS_FIELDS.get(field)
                if attribute is None:
                    continue
                values = value if isinstance(value, list) else [value]
                targets = tuple(
                    Target(
                        path=f"{self._record(record)}.glass[{index}].{attribute}"
                        + (f"[{position}]" if isinstance(value, list) else ""),
                        value=item,
                    )
                    for record, index in uses
                    for position, item in enumerate(values, start=1)
                )
                yield ResolvedParameter(
                    yaml_path=location.yaml_path,
                    group=location.group,
                    value=value,
                    unit="1" if attribute != "k" else UNITS["thermal_conductivity"],
                    element=name,
                    targets=targets,
                    note=None if uses else "glass not used by any glazing of the model",
                )

    def _gases(self, data: dict[str, Any], glazings: dict[str, Glass]) -> Iterator[ResolvedParameter]:
        for gas in data.get("gas") or []:
            name = clean_name(gas["id"])
            uses = [
                (record, index, layer.material)
                for record, glazing in glazings.items()
                for index, layer in enumerate([lay for lay in glazing.layers if isinstance(lay, GasLayer)], start=1)
                if layer.material.name == name
            ]
            for field, value in gas.items():
                if field == "id":
                    continue
                location = Location(yaml_path=f"gas[{gas['id']}].{field}", group="Glazing")
                targets: list[Target] = []
                note = None if uses else "gas not used by any glazing of the model"
                if field in GAS_FIELDS:
                    targets = [
                        Target(path=f"{self._record(record)}.gas[{index}].{GAS_FIELDS[field]}", value=value)
                        for record, index, _ in uses
                    ]
                elif field == "density":
                    for record, index, material in uses:
                        prefix = f"{self._record(record)}.gas[{index}]"
                        a_mu, b_mu = material.viscosity_coefficients
                        targets += [
                            Target(
                                path=f"{prefix}.MM",
                                value=material.molar_mass,
                                transformation=f"MM = density * R * {GAS_REFERENCE_TEMPERATURE} / "
                                f"{GAS_REFERENCE_PRESSURE} (ideal gas, R = {UNIVERSAL_GAS_CONSTANT})",
                            ),
                            Target(
                                path=f"{prefix}.a_mu",
                                value=a_mu,
                                transformation="viscosity of the standard gas"
                                " (air, argon, krypton, xenon) with the closest molar mass",
                            ),
                            Target(
                                path=f"{prefix}.b_mu",
                                value=b_mu,
                                transformation="viscosity of the standard gas with the closest molar mass",
                            ),
                        ]
                elif field in GAS_NOT_USED:
                    note = "emissivity of a gas layer is not a property of Buildings.HeatTransfer.Data.Gases"
                else:
                    continue
                yield ResolvedParameter(
                    yaml_path=location.yaml_path,
                    group=location.group,
                    value=value,
                    unit=UNITS.get(field),
                    element=name,
                    targets=tuple(targets),
                    note=note,
                )

    def _construction_layers(
        self, data: dict[str, Any], constructions: dict[str, Construction]
    ) -> Iterator[ResolvedParameter]:
        for construction in data.get("constructions") or []:
            name = clean_name(construction["id"])
            used = name in constructions
            for index, layer in enumerate(construction.get("layers") or [], start=1):
                if "thickness" not in layer:
                    continue
                yield ResolvedParameter(
                    yaml_path=f"constructions[{construction['id']}].layers[{index - 1}].thickness",
                    group="Layers",
                    value=layer["thickness"],
                    unit=UNITS["thickness"],
                    element=name,
                    targets=(Target(path=f"{self._record(name)}.material[{index}].x", value=layer["thickness"]),)
                    if used
                    else (),
                    note=None if used else "construction not used by any wall of the model",
                )

    def _glazing_layers(self, data: dict[str, Any], glazings: dict[str, Glass]) -> Iterator[ResolvedParameter]:
        for glazing in data.get("glazings") or []:
            name = clean_name(glazing["id"])
            used = name in glazings
            note = None if used else "glazing not used by any window of the model"
            if "u_value_frame" in glazing:
                yield ResolvedParameter(
                    yaml_path=f"glazings[{glazing['id']}].u_value_frame",
                    group="Glazing",
                    value=glazing["u_value_frame"],
                    unit=UNITS["u_value_frame"],
                    element=name,
                    targets=(Target(path=f"{self._record(name)}.UFra", value=glazing["u_value_frame"]),)
                    if used
                    else (),
                    note=note,
                )
            glass_index = gas_index = 0
            for position, layer in enumerate(glazing.get("layers") or []):
                if "glass" in layer:
                    glass_index += 1
                    array, index = "glass", glass_index
                else:
                    gas_index += 1
                    array, index = "gas", gas_index
                if "thickness" not in layer:
                    continue
                yield ResolvedParameter(
                    yaml_path=f"glazings[{glazing['id']}].layers[{position}].thickness",
                    group="Glazing",
                    value=layer["thickness"],
                    unit=UNITS["thickness"],
                    element=name,
                    targets=(Target(path=f"{self._record(name)}.{array}[{index}].x", value=layer["thickness"]),)
                    if used
                    else (),
                    note=note,
                )

    def _spaces(self, network: Network, data: dict[str, Any]) -> Iterator[ResolvedParameter]:
        for index, space_data in enumerate(data.get("spaces") or []):
            identifier = space_data.get("id", str(index))
            space = find_element(network, clean_name(identifier))
            if not isinstance(space, BaseSpace):
                raise FmuError(f"Space {identifier} not found in the network")
            arrays = ZoneArrays(space, network)
            boundaries = space_data.get("external_boundaries") or {}
            kinds: dict[str, type[BaseSimpleWall]] = {
                "external_walls": ExternalWall,
                "windows": BaseWindow,
                "floor_on_grounds": FloorOnGround,
                "adiabatic_walls": AdiabaticWall,
            }
            for section, kind in kinds.items():
                elements = [b for b in space.external_boundaries if isinstance(b, kind)]
                items = boundaries.get(section) or []
                if len(elements) != len(items):
                    raise FmuError(
                        f"Space {identifier}: {len(items)} {section} in the YAML, {len(elements)} in the network"
                    )
                for position, (item, element) in enumerate(zip(items, elements, strict=True)):
                    location = Location(
                        yaml_path=f"spaces[{identifier}].external_boundaries.{section}[{position}]", group="Geometry"
                    )
                    yield from self._boundary(item, element, arrays, location)

    def _boundary(
        self, item: dict[str, Any], element: BaseSimpleWall, arrays: ZoneArrays, location: Location
    ) -> Iterator[ResolvedParameter]:
        resolved = _BoundaryFields(item, element, location)
        if isinstance(element, BaseWindow):
            yield from self._window(item, element, arrays, resolved)
        else:
            yield from self._opaque(item, element, arrays, resolved)
        if "tilt" in item:
            yield resolved("tilt", [Target(path=arrays.path(element, "til"), value=None, transformation=TILT_NOTE)])
        if isinstance(element, FloorOnGround):
            if "ground_temperature" in item:
                yield resolved(
                    "ground_temperature",
                    [Target(path=f"{ENVELOPE}.{element.name}.T", value=item["ground_temperature"])],
                    group="Ground",
                )
            for field, reason in FLOOR_NOT_USED.items():
                if field in item:
                    yield resolved(field, [], group="Ground", note=f"IDEAS only: {reason}")

    @staticmethod
    def _window(
        item: dict[str, Any], window: BaseWindow, arrays: ZoneArrays, resolved: "_BoundaryFields"
    ) -> Iterator[ResolvedParameter]:
        shared = arrays.shared_entry(window)
        geometry = (
            "height and width area-weighted over the windows of the same orientation and glazing"
            if shared
            else "hWin = wWin = sqrt(surface) (a square window)"
        )
        yield resolved(
            "surface",
            [
                Target(path=arrays.path(window, "hWin"), value=window.height, transformation=geometry),
                Target(path=arrays.path(window, "wWin"), value=window.width, transformation=geometry),
            ],
        )
        if "frame_fraction" in item:
            weighted = "area-weighted over the windows of the entry" if shared else None
            yield resolved(
                "frame_fraction",
                [Target(path=arrays.path(window, "fFra"), value=item["frame_fraction"], transformation=weighted)],
                group="Glazing",
            )
        if "azimuth" in item:
            host = "azimuth of the host wall; the window must face as one of the walls"
            yield resolved(
                "azimuth", [Target(path=arrays.path(window, "azi"), value=item["azimuth"], transformation=host)]
            )

    @staticmethod
    def _opaque(
        item: dict[str, Any], wall: BaseSimpleWall, arrays: ZoneArrays, resolved: "_BoundaryFields"
    ) -> Iterator[ResolvedParameter]:
        if "surface" in item:
            gross = "gross area of the host walls of the orientation (windows included)"
            transformation = gross if arrays.entries[id(wall)][0] == "datConExtWin" else None
            yield resolved(
                "surface", [Target(path=arrays.path(wall, "A"), value=item["surface"], transformation=transformation)]
            )
        if "azimuth" in item:
            yield resolved("azimuth", [Target(path=arrays.path(wall, "azi"), value=item["azimuth"])])

    def _internal_walls(self, network: Network, data: dict[str, Any]) -> Iterator[ResolvedParameter]:
        arrays: dict[str, ZoneArrays] = {}
        seen: set[int] = set()
        internal = [node for node in network.graph.nodes if isinstance(node, InternalElement)]
        for position, wall in enumerate(data.get("internal_walls") or []):
            if "surface" not in wall:
                continue
            spaces = [find_element(network, clean_name(wall[key])) for key in ("space_1", "space_2")]
            prefix = f"internal_{clean_name(wall['space_1'])}_{clean_name(wall['space_2'])}_"
            element = next(
                (
                    node
                    for node in internal
                    if (node.name or "").startswith(prefix) and node.surface == wall["surface"] and id(node) not in seen
                ),
                None,
            )
            targets = []
            if element is not None:
                seen.add(id(element))
                for space in spaces:
                    if isinstance(space, BaseSpace):
                        zone = arrays.setdefault(space.name, ZoneArrays(space, network))
                        targets.append(Target(path=zone.path(element, "A"), value=wall["surface"]))
            yield ResolvedParameter(
                yaml_path=f"internal_walls[{position}].surface",
                group="Geometry",
                value=wall["surface"],
                unit=UNITS["surface"],
                element=element.name if element else None,
                targets=tuple(targets),
                note=None if element else "internal wall not found in the network",
            )


class _BoundaryFields:
    """Builds the resolved parameters of the fields of one boundary."""

    def __init__(self, item: dict[str, Any], element: BaseSimpleWall, location: Location) -> None:
        self.item, self.element, self.location = item, element, location

    def __call__(
        self, field: str, targets: list[Target], group: str = "Geometry", note: str | None = None
    ) -> ResolvedParameter:
        return ResolvedParameter(
            yaml_path=self.location.child(field).yaml_path,
            group=group,
            value=self.item.get(field),
            unit=UNITS.get(field),
            element=self.element.name,
            targets=tuple(targets),
            note=note,
        )
