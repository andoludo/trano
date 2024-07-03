import tempfile
from pathlib import Path

from trano.documentation.documentation import (
    ContentDocumentation,
    ContentModelDocumentation,
    ModelDocumentation,
    ResultFile,
)
from trano.documentation.docx import to_docx
from trano.documentation.html import to_html_documentation
from trano.topology import Network


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
        document = to_docx(documentation, Path(tmpfile.name))
        assert len(document.tables) == 6
        assert len(document.inline_shapes) == 0


def test_documentation_generate_docx_with_figures(
    buildings_two_rooms_with_storage: Network, result_data_path: Path
) -> None:
    documentation = ModelDocumentation.from_network(
        buildings_two_rooms_with_storage,
        result=ResultFile(
            path=result_data_path,
        ),
    )
    with tempfile.NamedTemporaryFile(suffix=".docx") as tmpfile:
        document = to_docx(documentation, Path(tmpfile.name))
        assert len(document.tables) == 6
        assert len(document.inline_shapes) == 13
