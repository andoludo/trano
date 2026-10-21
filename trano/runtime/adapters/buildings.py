"""Run-time names of the models trano writes for the Buildings library.

Besides the element parameters (:class:`~trano.runtime.adapters.base.LibraryAdapter`), the Buildings
templates write the envelope as records and arrays (``library.json``, ``models/buildings/*.yaml``):

- each construction and glazing as a record of the envelope container
  (``envelope1.external_wall_001.material[1].k``, ``envelope1.window_001.glass[1].tauSol[1]``);
- each zone's constructions as arrays of records referring to them, with their areas, azimuths and window
  geometry (``living_001.datConExtWin[1].A``, ``.wWin``, ``.glaSys``, ``.layers``);
- partitions between zones as ``MultiLayer`` components with their area and construction;
- the ground under the floors as fixed temperatures.

OpenModelica copies the records a zone refers to into the zone with literal values: the zone's
``datConExtWin[1].layers.material[1].k`` is a free parameter of its own and does not follow
``external_wall_001.material[1].k``. The snapshot therefore holds every copy, which is what an override
has to set; the partitions' ``layers`` stay bound to the envelope record and follow it.
"""

import math
import re
from typing import ClassVar

from trano.elements.construction import Construction, GasLayer, Glass, GlassLayer, Layer
from trano.elements.envelope import BaseFloorOnGround, BaseInternalElement, WindowedWallParameters
from trano.elements.space import BaseSpace
from trano.runtime.adapters.base import LibraryAdapter, Snapshot, SnapshotEntry
from trano.topology import Network

FORMULAS = {
    "wWin": "sqrt(surface) unless width and height are given (envelope.BaseWindow)",
    "hWin": "sqrt(surface) unless width and height are given (envelope.BaseWindow)",
    "MM": "density * 8.314462618 * 293.15 / 101325 (construction.Gas.molar_mass)",
    "a_mu": "viscosity of the standard gas of closest molar mass (construction.Gas.viscosity_coefficients)",
    "b_mu": "viscosity of the standard gas of closest molar mass (construction.Gas.viscosity_coefficients)",
    "nSta": "max(1, ceil(nStaRef * x * sqrt(c * d / k) / 331.4)) (Buildings Material record)",
    "R": "x / k (Buildings Solids record, final R)",
    "piMat": "x * sqrt(c * d / k) (Buildings Material record)",
    "nStaReal": "nStaRef * x * sqrt(c * d / k) / 331.4 (Buildings Material record)",
}
"""How the envelope templates transform YAML fields, by record field."""
PI_REFERENCE = 331.4  # x / sqrt(alpha) of the 0.2 m concrete reference layer (Buildings Material record)


def _pi_material(layer: Layer) -> float:
    """``piMat`` of the Buildings Material record: x / sqrt(alpha), the reference value for a steady layer."""
    material = layer.material
    if material.specific_heat_capacity < 1e-15 or material.density < 1e-15:
        return PI_REFERENCE
    return layer.thickness * math.sqrt(
        material.specific_heat_capacity * material.density / material.thermal_conductivity
    )


def number_of_states(layer: Layer) -> int:
    """The discretisation Buildings derives from a layer (``nSta`` of ``HeatTransfer.Data.BaseClasses.Material``)."""
    return max(1, math.ceil(layer.material.number_of_states * _pi_material(layer) / PI_REFERENCE))


_MATERIAL_FIELD = re.compile(
    r"^material\[(?P<id>[^]]+)\]\.(?P<field>thermal_conductivity|density|specific_heat_capacity)$"
)
_LAYER_THICKNESS = re.compile(r"^constructions\[(?P<id>[^]]+)\]\.layers\[(?P<index>\d+)\]\.thickness$")
NSTA_EXPONENTS = {"thermal_conductivity": -0.5, "density": 0.5, "specific_heat_capacity": 0.5, "thickness": 1.0}
"""nStaReal is proportional to x * sqrt(c * d / k): the power of each field in it."""


def _clean(identifier: str) -> str:
    return identifier.lower().replace(":", "_")


def discretisation_range(layers: list[Layer], field: str, value: float) -> tuple[float, float]:
    """The values of a field (of all the layers given) for which each layer keeps its number of states.

    A layer keeps ``nSta = N`` while ``N - 1 < nStaReal <= N`` (``nStaReal <= 1`` for ``N = 1``), and
    ``nStaReal`` scales with the field to the power :data:`NSTA_EXPONENTS`.
    """
    exponent = NSTA_EXPONENTS[field]
    low, high = 0.0, math.inf
    for layer in layers:
        material = layer.material
        real = (
            material.number_of_states
            * layer.thickness
            * math.sqrt(material.specific_heat_capacity * material.density / material.thermal_conductivity)
            / PI_REFERENCE
        )
        states = number_of_states(layer)
        bounds = [states - 1 if states > 1 else 0.0, float(states)]  # of nStaReal: (lower, upper]
        values = [
            value * (bound / real) ** (1 / exponent) if bound > 0 else (0.0 if exponent > 0 else math.inf)
            for bound in bounds
        ]
        low, high = max(low, min(values)), min(high, max(values))
    return low, high


ZONE_RECORDS = {"ExternalWall": "datConExt", "FloorOnGround": "datConBou", "WindowedWall": "datConExtWin"}
"""Boundary types of the zone template and the record array each one writes (surBou has no layers)."""


def _record_values(prefix: str, construction: Construction | Glass) -> dict[str, float]:
    """The scalars of a construction or glazing record, as library.json renders them."""
    values: dict[str, float] = {}
    if isinstance(construction, Glass):
        glasses = [layer for layer in construction.layers if isinstance(layer, GlassLayer)]
        gases = [layer for layer in construction.layers if isinstance(layer, GasLayer)]
        for i, glass in enumerate(glasses, start=1):
            material = glass.material
            name = f"{prefix}.glass[{i}]"
            values |= {f"{name}.x": glass.thickness, f"{name}.k": material.thermal_conductivity}
            for field, data in (
                ("tauSol", material.solar_transmittance),
                ("rhoSol_a", material.solar_reflectance_outside_facing),
                ("rhoSol_b", material.solar_reflectance_room_facing),
            ):
                values |= {f"{name}.{field}[{j}]": value for j, value in enumerate(data, start=1)}
            values |= {
                f"{name}.tauIR": material.infrared_transmissivity,
                f"{name}.absIR_a": material.infrared_absorptivity_outside_facing,
                f"{name}.absIR_b": material.infrared_absorptivity_room_facing,
            }
        for i, gas in enumerate(gases, start=1):
            name = f"{prefix}.gas[{i}]"
            a_mu, b_mu = gas.material.viscosity_coefficients
            values |= {
                f"{name}.x": gas.thickness,
                f"{name}.a_k": gas.material.thermal_conductivity,
                f"{name}.a_mu": a_mu,
                f"{name}.b_mu": b_mu,
                f"{name}.a_c": gas.material.specific_heat_capacity,
                f"{name}.MM": gas.material.molar_mass,
            }
        values[f"{prefix}.UFra"] = construction.u_value_frame
        return values
    first, last = construction.layers[0].material, construction.layers[-1].material
    values |= {
        f"{prefix}.absIR_a": first.longwave_emissivity,
        f"{prefix}.absIR_b": last.longwave_emissivity,
        f"{prefix}.absSol_a": first.shortwave_emissivity,
        f"{prefix}.absSol_b": last.shortwave_emissivity,
    }
    for i, layer in enumerate(construction.layers, start=1):
        name = f"{prefix}.material[{i}]"
        pi_material = _pi_material(layer)
        values |= {
            f"{name}.x": layer.thickness,
            f"{name}.k": layer.material.thermal_conductivity,
            f"{name}.c": layer.material.specific_heat_capacity,
            f"{name}.d": layer.material.density,
            f"{name}.nStaRef": layer.material.number_of_states,
            # derived in the Buildings records; OpenModelica copies them into the zones as free literals
            f"{name}.R": layer.thickness / layer.material.thermal_conductivity,
            f"{name}.piMat": pi_material,
            f"{name}.nStaReal": layer.material.number_of_states * pi_material / PI_REFERENCE,
            f"{name}.nSta": number_of_states(layer),
            f"{prefix}.nSta[{i}]": number_of_states(layer),
        }
    values[f"{prefix}.R"] = sum(layer.thickness / layer.material.thermal_conductivity for layer in construction.layers)
    return values


class BuildingsAdapter(LibraryAdapter):
    library: ClassVar[str] = "Buildings"
    not_used_fields: ClassVar[dict[str, str]] = {
        r"\.floor_on_grounds\[[^]]+\]\.perimeter$": "IDEAS only (ISO 13370 ground model); Buildings takes "
        "ground_temperature",
        r"\.floor_on_grounds\[[^]]+\]\.average_indoor_temperature$": "IDEAS only (ISO 13370 ground model); "
        "Buildings takes ground_temperature",
    }

    frozen_mechanisms: ClassVar[dict[str, str]] = {
        r"\.radiator_[^.]+\.(T_a_nominal|T_b_nominal|TAir_nominal)$": "RadiatorEN442_2 derives m_flow_nominal "
        "and the start values of its states from the nominal temperatures; they are evaluated with them and "
        "inlined in UAEle",
        r"\.radiator_[^.]+\.(mDry|VWat)$": "RadiatorEN442_2: VWat sizes the water volumes and both enter the "
        "final mSenFac; trano computes them from Q_flow_nominal (parameters.yaml)",
        r"\.duct_[^.]+\.dp_nominal$": "PressureDrop: the final computeFlowResistance (Evaluate=true) depends on "
        "dp_nominal",
        r"\.ahu_[^.]+\.(m_flow_nominal|dp_nominal|dpSup_nominal|dpRet_nominal)$": "the system D template passes "
        "them to the fans' pressure curves (per records) and to the heat exchanger's dp1/dp2_nominal, whose "
        "flow-resistance flags are Evaluate=true",
        r"\.pump_[^.]+\.(m_flow_nominal|dp_nominal)$": "the pump template passes them to the mover's pressure "
        "curve (per record), evaluated with the mover's structural flags",
        r"\.nSta(\[\d+\])?$": "number of states of the layer, an array size with Evaluate=true (Buildings "
        "Material record): the change alters the discretisation",
        r"(\.layers|^building\.envelope1\.[a-z0-9_]+)\.R$": "final R = sum of the layers' R of the "
        "OpaqueConstructions record, evaluated with the discretisation",
        r"\.pv_[^.]+\.(til|azi)$": "Evaluate=true in Buildings.Electrical.Interfaces.PartialPVOriented",
        r"^building\.envelope1\.[a-z0-9_]+\.(glass|gas)\[": "trano writes the glazing record with final glass and "
        "gas modifiers (library.json); the zones use free copies of their own",
    }

    unused_frozen: ClassVar[tuple[str, ...]] = (
        r"(\.layers|^building\.envelope1\.[a-z0-9_]+)\.R$",  # construction total R (equivalence exact)
        r"^building\.envelope1\.[a-z0-9_]+\.(glass|gas)\[",  # envelope glazing record (equivalence exact)
    )

    def parameter_snapshot(self, network: Network) -> Snapshot:
        snapshot = self.element_parameters(network)
        constructions: dict[str, Construction | Glass] = {}
        envelope = f"{self.top_component}.envelope1"
        for element in network.graph.nodes:
            construction = getattr(element, "construction", None)
            if isinstance(construction, Construction | Glass):
                constructions[construction.name] = construction
            if isinstance(element, BaseFloorOnGround) and type(element).__name__ == "FloorOnGround":
                if element.variant == "default":
                    path = f"{self.element_prefix(element)}.T"
                    snapshot[path] = SnapshotEntry(element.ground_temperature, path)
            elif isinstance(element, BaseInternalElement) and type(element).__name__ == "InternalElement":
                path = f"{self.element_prefix(element)}.A"
                snapshot[path] = SnapshotEntry(float(element.surface), path)
        for construction in constructions.values():
            snapshot |= self._record(f"{envelope}.{construction.name}", construction)
        for element in network.graph.nodes:
            if isinstance(element, BaseSpace) and element.boundaries:
                snapshot |= self._zone(element, constructions)
        return snapshot

    def admissible_range(self, network: Network, path: str) -> tuple[float, float] | None:
        """For a material property or a layer thickness: the values keeping the discretisation of every layer."""
        constructions = {
            construction.name: construction
            for element in network.graph.nodes
            if isinstance(construction := getattr(element, "construction", None), Construction)
        }
        if match := _MATERIAL_FIELD.match(path):
            field = match.group("field")
            layers = [
                layer
                for construction in constructions.values()
                for layer in construction.layers
                if layer.material.name == _clean(match.group("id"))
            ]
            if not layers:
                return None
            return discretisation_range(layers, field, float(getattr(layers[0].material, field)))
        if match := _LAYER_THICKNESS.match(path):
            construction = constructions.get(_clean(match.group("id")))
            if construction is None:
                return None
            layer = construction.layers[int(match.group("index"))]
            return discretisation_range([layer], "thickness", layer.thickness)
        return None

    def impact_outputs(self, network: Network) -> dict[str, str]:
        """Zone air temperatures, radiator heat flows, heat pump heat and power, AHU supply air, PV power."""
        outputs: dict[str, str] = {}
        for element in sorted(network.graph.nodes, key=lambda node: str(node.name)):
            prefix = self.element_prefix(element) if getattr(element, "container_type", None) else None
            if prefix is None:
                continue
            match type(element).__name__, element.variant:
                case ("Space", _):
                    outputs[f"T_{element.name}"] = f"{prefix}.air.vol.T"
                case ("Radiator", _):
                    outputs[f"Q_{element.name}"] = f"{prefix}.Q_flow"
                case ("Boiler", "air_water_heat_pump"):
                    outputs[f"Q_{element.name}"] = f"{prefix}.airToWaterTableData2D.QCon_flow"
                    outputs[f"P_{element.name}"] = f"{prefix}.airToWaterTableData2D.P"
                case ("AirHandlingUnit", _):
                    outputs[f"TSup_{element.name}"] = f"{prefix}.TSup.T"
                case ("Photovoltaic", _):
                    outputs[f"P_{element.name}"] = f"{prefix}.P"
        return outputs

    @staticmethod
    def _record(prefix: str, construction: Construction | Glass) -> Snapshot:
        return {
            name: SnapshotEntry(float(value), prefix, FORMULAS.get(name.rsplit(".", 1)[-1]))
            for name, value in _record_values(prefix, construction).items()
        }

    def _zone(self, space: BaseSpace, constructions: dict[str, Construction | Glass]) -> Snapshot:  # noqa: C901
        """The arrays of the zone template (space.yaml), element by element."""
        zone = self.element_prefix(space)
        snapshot: Snapshot = {}
        for boundary in space.boundaries or []:
            if not boundary.number:
                continue
            if boundary.type == "InternalElement":
                for j, surface in enumerate(boundary.surfaces, start=1):
                    snapshot[f"{zone}.surBou[{j}].A"] = SnapshotEntry(float(surface), f"{zone}.surBou.A")
                continue
            record = ZONE_RECORDS[boundary.type]
            arrays: dict[str, list[float]] = {"A": boundary.surfaces, "azi": boundary.azimuths}
            if isinstance(boundary, WindowedWallParameters):
                arrays |= {
                    "wWin": boundary.window_width,
                    "hWin": boundary.window_height,
                    "fFra": boundary.window_frame_fraction,
                }
            if isinstance(boundary, WindowedWallParameters) and boundary.has_shading:
                arrays |= {
                    "ove.wL": boundary.overhang_width_left,
                    "ove.wR": boundary.overhang_width_right,
                    "ove.dep": boundary.overhang_depth,
                    "ove.gap": boundary.overhang_gap,
                    "sidFin.h": boundary.side_fin_height,
                    "sidFin.dep": boundary.side_fin_depth,
                    "sidFin.gap": boundary.side_fin_gap,
                }
            for field, values in arrays.items():
                for j, value in enumerate(values, start=1):
                    snapshot[f"{zone}.{record}[{j}].{field}"] = SnapshotEntry(
                        float(value), f"{zone}.{record}.{field}", FORMULAS.get(field)
                    )
            for j, layers in enumerate(boundary.layers, start=1):
                snapshot |= self._record(f"{zone}.{record}[{j}].layers", constructions[layers])
            if isinstance(boundary, WindowedWallParameters):
                for j, glazing in enumerate(boundary.window_layers, start=1):
                    snapshot |= self._record(f"{zone}.{record}[{j}].glaSys", constructions[glazing])
        return snapshot


__all__ = ["BuildingsAdapter"]
