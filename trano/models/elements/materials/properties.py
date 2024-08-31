from typing import TYPE_CHECKING

from networkx.classes.reportviews import NodeView

from trano.models.elements.construction import Construction, Glass, BaseData, ConstructionData, BaseConstructionData, \
    MaterialProperties
from trano.models.elements.envelope import MergedBaseWall, BaseSimpleWall

if TYPE_CHECKING:
    from trano.library.library import Library


def default_construction(nodes: NodeView) -> ConstructionData:
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
    return ConstructionData(
        constructions=wall_constructions, materials=[], glazing=glazing
    )


def merged_construction(nodes: NodeView) -> ConstructionData:
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
    return ConstructionData(
        constructions=wall_constructions, materials=list(materials), glazing=glazing
    )


def extract_data(
    package_name: str, nodes: NodeView, library: "Library"
) -> MaterialProperties:

    if library.merged_external_boundaries:
        data = merged_construction(nodes)
    else:
        data = default_construction(nodes)
    data_ = BaseConstructionData(
        template=library.templates.main,
        construction=BaseData(
            constructions=data.constructions, template=library.templates.construction
        ),
        glazing=BaseData(constructions=data.glazing, template=library.templates.glazing),
        material=BaseData(
            constructions=data.materials, template=library.templates.material
        ),
    )
    return MaterialProperties(data=data_.generate_data(package_name), is_package=library.templates.is_package)


def extract_properties(
    library: "Library", package_name: str, nodes: NodeView
) -> MaterialProperties:
    return extract_data(package_name, nodes, library)
