import tempfile
from pathlib import Path

from neosim.documentation.documentation import (
    ContentDocumentation,
    ContentModelDocumentation,
    ModelDocumentation,
    ResultFile,
)
from neosim.documentation.docx import to_docx
from neosim.documentation.html import to_html_documentation
from neosim.topology import Network


def test_documentation_buildings_two_rooms_with_storage(
    buildings_two_rooms_with_storage: Network,
) -> None:
    documentation = ModelDocumentation.from_network(buildings_two_rooms_with_storage)
    assert documentation


def test_documentation_generate_html(
    buildings_two_rooms_with_storage: Network,
) -> None:
    content = ContentModelDocumentation(
        systems=ContentDocumentation(
            title="Spaces",
            introduction="These are the sapces in the model",
            conclusions="results",
        ),
        spaces=ContentDocumentation(
            title="vdvdvd",
            introduction="These are the sapces in the model",
            conclusions="results",
        ),
        constructions=ContentDocumentation(
            title="dvdvd",
            introduction="These are the sapces in the model",
            conclusions="results",
        ),
    )
    documentation = ModelDocumentation.from_network(
        buildings_two_rooms_with_storage, content
    )
    html = to_html_documentation(documentation)
    assert html


def test_documentation_generate_docx(
    buildings_two_rooms_with_storage: Network,
) -> None:
    documentation = ModelDocumentation.from_network(buildings_two_rooms_with_storage)
    with tempfile.NamedTemporaryFile(suffix=".docx") as tmpfile:
        to_docx(documentation, Path(tmpfile.name))
        assert tmpfile


def test_documentation_generate_docx_with_figures(
    buildings_two_rooms_with_storage: Network,
) -> None:
    documentation = ModelDocumentation.from_network(
        buildings_two_rooms_with_storage,
        result=ResultFile(
            path=Path(__file__).parent.joinpath("resources", "data.mat"),
        ),
    )
    to_docx(documentation, Path("/home/aan/Documents/neosim/tests/report.docx"))
