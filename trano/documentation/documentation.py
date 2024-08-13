import abc
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, List, Literal, Optional, Type

from pydantic import BaseModel, computed_field

from trano.models.elements.base import BaseElement, BaseParameter
from trano.models.elements.boiler import Boiler
from trano.models.elements.envelope.base import BaseSimpleWall
from trano.models.elements.space import Space
from trano.models.elements.system import System

if TYPE_CHECKING:
    from trano.topology import Network

Topic = Literal["Spaces", "Construction", "Systems", "Base"]


class ContentDocumentation(BaseModel):
    title: Optional[str] = None
    introduction: Optional[str] = None
    conclusions: Optional[str] = None


def get_description() -> Dict[str, Any]:
    return {
        field.alias: (field.description or field.title)
        for cls in BaseParameter.__subclasses__()
        for field in cls.model_fields.values()
        if field.alias
    }


class BaseDocumentation(ContentDocumentation):
    title: str
    introduction: str
    table: Dict[str, Any] | List[Dict[str, Any]]
    conclusions: str

    @classmethod
    @abc.abstractmethod
    def from_elements(
        cls, elements: List[BaseElement], content_documentation: ContentDocumentation
    ) -> "BaseDocumentation":
        ...

    @computed_field
    def topic(self) -> Topic:
        return type(self).__name__.replace("Documentation", "")  # type: ignore


class SpacesDocumentation(BaseDocumentation):
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
                    "parameters": True,
                    "occupancy": {"name": True, "parameters": True},
                },
                exclude_none=True,
                by_alias=True,
            )
            for key, value in document_mapping.items():
                values = _dump_list_attributes(space, key, value)
                if values:
                    main_space[key] = _dump_list_attributes(space, key, value)
            spaces_.append(main_space)
        data = {
            "title": "Spaces",
            "introduction": "Introduction",
            "table": spaces_,
            "conclusions": "Conclusions",
        } | content_documentation.model_dump(exclude_none=True)
        return cls(**data)


class ConstructionDocumentation(BaseDocumentation):
    @classmethod
    def from_elements(
        cls, elements: List[BaseElement], content_documentation: ContentDocumentation
    ) -> "ConstructionDocumentation":
        constructions = [
            c.model_dump(by_alias=True, exclude_none=True)
            for c in {
                w.construction
                for w in _get_elements(elements, BaseSimpleWall)
                if isinstance(w, BaseSimpleWall)
            }
        ]
        data = {
            "title": "Spaces",
            "introduction": "Introduction",
            "table": constructions,
            "conclusions": "Conclusions",
        } | content_documentation.model_dump(exclude_none=True)
        return cls(**data)


class SystemsDocumentation(BaseDocumentation):
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
        try:
            systems = [
                system.model_dump(
                    by_alias=True,
                    exclude_none=True,
                    include={"name": True, "parameters": True, "type": True},
                )
                for system in _get_elements(elements, System)
                if system not in systems_to_exclude
            ]
        except:
            raise
        data = {
            "title": "Spaces",
            "introduction": "Introduction",
            "table": systems,
            "conclusions": "Conclusions",
        } | content_documentation.model_dump(exclude_none=True)
        return cls(**data)


class ContentModelDocumentation(ContentDocumentation):
    spaces: ContentDocumentation
    constructions: ContentDocumentation
    systems: ContentDocumentation


class ResultFile(BaseModel):
    path: Path
    type: Literal["openmodelica", "dymola"] = "openmodelica"


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
            "title": "Spaces",
            "introduction": "Introduction",
            "spaces": spaces_documentation,
            "constructions": constructions,
            "systems": systems,
            "conclusions": "Conclusions",
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
            if isinstance(x, (Boiler, Space, BaseSimpleWall))
        ]
        return cls.from_model_elements(elements, content_model_documentation, result)  # type: ignore


def _get_elements(
    elements: List[BaseElement], element_type: Type[Space | System | BaseSimpleWall]
) -> List[BaseElement]:
    return [element for element in elements if isinstance(element, element_type)]


def _dump_list_attributes(
    element: BaseElement, attribute_name: str, include_mapping: object
) -> List[Dict[int, Any]]:
    return [
        el.model_dump(
            by_alias=True,
            exclude_none=True,
            mode="json",
            include=include_mapping,
        )
        for el in getattr(element, attribute_name)
    ]
