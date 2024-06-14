from typing import TYPE_CHECKING

from networkx.classes.reportviews import NodeView

from neosim.construction import Construction
from neosim.glass import Glass
from neosim.models.elements.envelope.base import BaseSimpleWall, MergedBaseWall
from neosim.models.elements.materials.base import (
    BaseConstructionData,
    MaterialProperties,
)
from neosim.models.elements.materials.construction import (
    BuildingsConstruction,
    IdeasConstruction,
)
from neosim.models.elements.materials.glazing import BuildingsGlazing, IdeasGlazing
from neosim.models.elements.materials.material import BuildingsMaterial, IdeasMaterial

if TYPE_CHECKING:
    from neosim.library.library import Libraries


class BuildingsData(BaseConstructionData):
    template: str = """
{% for g in glazing %}
    {{ g|safe }}
{% endfor %}
{%- for c in construction -%}
    {{ c|safe}}
{%- endfor %}
"""
    construction: BuildingsConstruction
    material: BuildingsMaterial
    glazing: BuildingsGlazing


class IdeasData(BaseConstructionData):
    template: str = """package Data "Data for transient thermal building simulation"
extends Modelica.Icons.MaterialPropertiesPackage;

package Glazing "Library of building glazing systems"
extends Modelica.Icons.MaterialPropertiesPackage;
{% for g in glazing %}
    {{ g|safe }}
{% endfor %}
end Glazing;

package Materials "Library of construction materials"
extends Modelica.Icons.MaterialPropertiesPackage;
{%- for m in material -%}
    {{ m|safe }}
{%- endfor %}
end Materials;
package Constructions "Library of building envelope constructions"
{%- for c in construction -%}
    {{ c|safe}}
{%- endfor %}

end Constructions;
end Data;"""
    construction: IdeasConstruction
    material: IdeasMaterial
    glazing: IdeasGlazing


def extract_buildings_data(package_name: str, nodes: NodeView) -> MaterialProperties:

    constructions = {
        node.construction
        for node in [node_ for node_ in nodes if isinstance(node_, BaseSimpleWall)]
    }
    wall_constructions = sorted(
        [c for c in constructions if isinstance(c, Construction)], key=lambda x: x.name
    )
    glazing = sorted(
        [c for c in constructions if isinstance(c, Glass)], key=lambda x: x.name
    )
    buildings_data = BuildingsData(
        construction=BuildingsConstruction(constructions=wall_constructions),
        glazing=BuildingsGlazing(constructions=glazing),
        material=BuildingsMaterial(),
    )
    return MaterialProperties(
        data=buildings_data.generate_data(package_name), is_package=False
    )


def extract_ideas_data(package_name: str, nodes: NodeView) -> MaterialProperties:

    merged_constructions = {
        construction
        for node in [node_ for node_ in nodes if isinstance(node_, MergedBaseWall)]
        for construction in node.constructions
    }

    constructions = {
        node.construction
        for node in [node_ for node_ in nodes if isinstance(node_, BaseSimpleWall)]
    }
    merged_constructions.update(constructions)
    wall_constructions = [
        c for c in merged_constructions if isinstance(c, Construction)
    ]
    glazing = [c for c in merged_constructions if isinstance(c, Glass)]
    materials = {
        layer.material
        for construction in merged_constructions
        for layer in construction.layers
    }
    ideas_data = IdeasData(
        material=IdeasMaterial(constructions=list(materials)),
        construction=IdeasConstruction(constructions=wall_constructions),
        glazing=IdeasGlazing(constructions=glazing),
    )
    return MaterialProperties(
        data=ideas_data.generate_data(package_name), is_package=True
    )


def extract_properties(
    library: "Libraries", package_name: str, nodes: NodeView
) -> MaterialProperties:
    return library.extract_properties(package_name, nodes)
