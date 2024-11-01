from typing import Any, Optional

from buildingspy.io.outputfile import Reader  # type: ignore
from pydantic import BaseModel, ConfigDict, Field, model_validator
from yattag import Doc, SimpleDoc

from trano.elements import Space
from trano.plot.plot import plot_plot_ly_many
from trano.reporting.reporting import BaseDocumentation, ModelDocumentation


class HtmlDoc(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    doc: SimpleDoc = Field(default=None)  # type: ignore
    tag: Any = Field(default=None)
    text: Any = Field(default=None)

    @model_validator(mode="after")
    def instantiate_from_package(self) -> "HtmlDoc":
        doc, tag, text = Doc().tagtext()
        self.doc = self.doc or doc
        self.tag = self.tag or tag
        self.text = self.text or text
        return self


def to_html(
    documentation: BaseDocumentation, html_doc: Optional[HtmlDoc] = None
) -> str:
    html_doc = html_doc or HtmlDoc()
    if not documentation.table:
        return ""
    with html_doc.tag("body"):
        with html_doc.tag("h1", style="font-size: 16px;"):
            html_doc.text(documentation.title or documentation.topic)

        with html_doc.tag("p"), html_doc.tag("p"):
            if documentation.introduction:
                html_doc.text(documentation.introduction)
        with html_doc.tag("p"):
            if isinstance(documentation.table, list):
                for t in documentation.table:
                    html_doc.doc.asis(t.to_html())
            else:
                html_doc.doc.asis(documentation.table.to_html())
        with html_doc.tag("p"), html_doc.tag("p"):
            if documentation.conclusions:
                html_doc.text(documentation.conclusions)
    return html_doc.doc.getvalue()


def to_html_reporting(documentation: ModelDocumentation) -> str:
    html_doc = HtmlDoc()
    with html_doc.tag("html"):
        with html_doc.tag("head"), html_doc.tag("title"):
            if documentation.title:
                html_doc.text(documentation.title)
        html_doc.doc.asis(to_html(documentation.spaces))
        html_doc.doc.asis(to_html(documentation.constructions))
        html_doc.doc.asis(to_html(documentation.systems))
        space = [s for s in documentation.elements if isinstance(s, Space)]
        if documentation.result:
            mat = Reader(documentation.result.path, documentation.result.type)
            for figures in list(zip(*[s.figures for s in space])):
                figures_plotly = plot_plot_ly_many(mat, list(figures), show=False)
                if figures_plotly:
                    html_doc.doc.asis(figures_plotly.to_html())

    return html_doc.doc.getvalue()
