import abc
from typing import TYPE_CHECKING, List, Optional

from pydantic import computed_field

from trano.elements import BaseElement, BaseSimpleWall, BaseWall
from trano.elements.envelope import MergedBaseWall
from trano.elements.space import Space
from trano.elements.system import Boiler, System
from trano.reporting.types import (
    BaseTable,
    ConstructionTable,
    ContentDocumentation,
    ResultFile,
    SpaceTable,
    SystemTable,
    Topic,
)
from trano.reporting.utils import _dump_list_attributes, _get_elements, get_description

if TYPE_CHECKING:
    from trano.topology import Network


class BaseDocumentation(ContentDocumentation):
    title: Optional[str] = None
    introduction: Optional[str] = None
    table: BaseTable | List[BaseTable]
    conclusions: Optional[str] = None

    @classmethod
    @abc.abstractmethod
    def from_elements(
        cls, elements: List[BaseElement], content_documentation: ContentDocumentation
    ) -> "BaseDocumentation": ...

    @computed_field
    def topic(self) -> Topic:
        return type(self).__name__.replace("Documentation", "")  # type: ignore


class SpacesDocumentation(BaseDocumentation):
    table: List[SpaceTable]  # type: ignore

    @classmethod
    def from_elements(
        cls, elements: List[BaseElement], content_documentation: ContentDocumentation
    ) -> "SpacesDocumentation":
        spaces = _get_elements(elements, Space)
        spaces_ = []
        boundary_parameters = {
            "name": True,
            "surface": True,
            "type": True,
            "azimuth": True,
            "tilt": True,
            "construction": {"name"},
        }
        system_parameters = {"name": True, "parameters": True, "type": True}
        document_mapping = {
            "emissions": system_parameters,
            "ventilation_inlets": system_parameters,
            "ventilation_outlets": system_parameters,
            "external_boundaries": boundary_parameters,
            "internal_elements": boundary_parameters,
        }
        for space in spaces:
            main_space = space.model_dump(
                mode="json",
                include={
                    "name": True,
                    "occupancy": {"name": True},
                },
                exclude_none=True,
                by_alias=True,
            )
            if space.parameters:
                main_space["parameters"] = space.parameters.model_dump(
                    by_alias=True, exclude_none=True
                )
            if (
                main_space.get("occupancy")
                and isinstance(space, Space)
                and space.occupancy
                and space.occupancy.parameters
            ):
                main_space["occupancy"]["parameters"] = (
                    space.occupancy.parameters.model_dump(
                        by_alias=True, exclude_none=True
                    )
                )
            for key, value in document_mapping.items():
                values = _dump_list_attributes(space, key, value)
                if values:
                    main_space[key] = _dump_list_attributes(space, key, value)
            spaces_.append(SpaceTable(**main_space))
        data = {
            "table": spaces_,
        } | content_documentation.model_dump(exclude_none=True)
        return cls(**data)


class ConstructionDocumentation(BaseDocumentation):
    table: List[ConstructionTable]  # type: ignore

    @classmethod
    def from_elements(
        cls, elements: List[BaseElement], content_documentation: ContentDocumentation
    ) -> "ConstructionDocumentation":
        constructions = [
            c.model_dump(
                by_alias=True,
                exclude_none=True,
                exclude={
                    "total_thermal_resistance",
                    "total_thermal_capacitance",
                    "u_value",
                    "resistance_external",
                    "resistance_external_remaining",
                },
            )
            for c in {
                construction
                for w in _get_elements(elements, BaseWall)
                if isinstance(w, (BaseSimpleWall, MergedBaseWall))
                for construction in (
                    [w.construction] if hasattr(w, "construction") else w.constructions
                )
            }
        ]
        data = {
            "table": [
                ConstructionTable(**construction) for construction in constructions
            ],
        } | content_documentation.model_dump(exclude_none=True)
        return cls(**data)


class SystemsDocumentation(BaseDocumentation):
    table: List[SystemTable]  # type: ignore

    @classmethod
    def from_elements(
        cls, elements: List[BaseElement], content_documentation: ContentDocumentation
    ) -> "SystemsDocumentation":
        spaces = _get_elements(elements, Space)
        get_description()
        systems_to_exclude = {
            system
            for space in spaces
            if isinstance(space, Space)
            for system in space.emissions
            + space.ventilation_inlets
            + space.ventilation_outlets
        }
        systems = []
        for system in _get_elements(elements, System):
            if system not in systems_to_exclude:
                system_ = system.model_dump(
                    by_alias=True,
                    exclude_none=True,
                    include={"name": True, "parameters": True, "type": True},
                )
                if system.parameters:
                    system_["parameters"] = system.parameters.model_dump()
                systems.append(SystemTable(**system_))

        data = {
            "table": systems,
        } | content_documentation.model_dump(exclude_none=True)
        return cls(**data)


class ContentModelDocumentation(ContentDocumentation):
    spaces: ContentDocumentation
    constructions: ContentDocumentation
    systems: ContentDocumentation


class ModelDocumentation(ContentDocumentation):
    spaces: SpacesDocumentation
    constructions: ConstructionDocumentation
    systems: SystemsDocumentation
    elements: List[BaseElement]
    result: Optional[ResultFile] = None

    @classmethod
    def from_model_elements(
        cls,
        elements: List[BaseElement],
        content_documentation: ContentModelDocumentation,
        result: Optional[ResultFile] = None,
    ) -> "ModelDocumentation":
        spaces_documentation = SpacesDocumentation.from_elements(
            elements, content_documentation.spaces
        )
        constructions = ConstructionDocumentation.from_elements(
            elements, content_documentation.constructions
        )
        systems = SystemsDocumentation.from_elements(
            elements, content_documentation.systems
        )
        data = content_documentation.model_dump(exclude_none=True) | {
            "spaces": spaces_documentation,
            "constructions": constructions,
            "systems": systems,
            "elements": elements,
            "result": result,
        }

        return cls(**data)

    @classmethod
    def from_network(
        cls,
        network: "Network",
        content_model_documentation: Optional[ContentModelDocumentation] = None,
        result: Optional[ResultFile] = None,
    ) -> "ModelDocumentation":
        content_model_documentation = (
            content_model_documentation
            or ContentModelDocumentation(
                spaces=ContentDocumentation(),
                constructions=ContentDocumentation(),
                systems=ContentDocumentation(),
            )
        )
        elements = [
            x
            for x in list(network.graph.nodes)
            if isinstance(x, (Boiler, Space, BaseSimpleWall, MergedBaseWall))
        ]
        return cls.from_model_elements(elements, content_model_documentation, result)  # type: ignore
