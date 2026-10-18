"""The documentation of a network: trano's description of it, documented by trano-simulate."""

from typing import TYPE_CHECKING

from trano_simulate.reporting import reporting
from trano_simulate.reporting.reporting import (
    BaseDocumentation,
    ConstructionDocumentation,
    ContentModelDocumentation,
    SpacesDocumentation,
    SystemsDocumentation,
)
from trano_simulate.reporting.types import ContentDocumentation, ResultFile

from trano.model_description import model_description

if TYPE_CHECKING:
    from trano.topology import Network

__all__ = [
    "BaseDocumentation",
    "ConstructionDocumentation",
    "ContentDocumentation",
    "ContentModelDocumentation",
    "ModelDocumentation",
    "ResultFile",
    "SpacesDocumentation",
    "SystemsDocumentation",
]


class ModelDocumentation(reporting.ModelDocumentation):
    @classmethod
    def from_network(
        cls,
        network: "Network",
        content_model_documentation: ContentModelDocumentation | None = None,
        result: ResultFile | None = None,
    ) -> "ModelDocumentation":
        """The documentation of the network, from its description (``trano.model_description``)."""
        return cls.from_description(model_description(network), content_model_documentation, result)
