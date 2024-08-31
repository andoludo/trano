from typing import Any, Optional

from json2html import json2html  # type: ignore
from pydantic import BaseModel, ConfigDict, Field, model_validator
from yattag import Doc, SimpleDoc

from trano.reporting.reproting import BaseDocumentation, ModelDocumentation


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
    with html_doc.tag("body"):
        with html_doc.tag("h1"):
            html_doc.text(documentation.title)

        with html_doc.tag("p"):
            with html_doc.tag("h2"):
                html_doc.text("Introduction")
            with html_doc.tag("p"):
                html_doc.text(documentation.introduction)
        with html_doc.tag("p"):
            if isinstance(documentation.table, list):
                for t in documentation.table:
                    table = json2html.convert(
                        json=t,
                        table_attributes="border='1'  align='center' bgcolor='#f0f0f0' "
                        "style='border-collapse: collapse; margin-top: "
                        "20px; margin-bottom: 20px;'",
                    )
                    html_doc.doc.asis(table)
            else:
                table = json2html.convert(
                    json=documentation.table,
                    table_attributes="border='1'  align='center' bgcolor='#f0f0f0' "
                    "style='border-collapse: collapse; margin-top: "
                    "20px; margin-bottom: 20px;'",
                )
                html_doc.doc.asis(table)
        with html_doc.tag("p"):
            with html_doc.tag("h2"):
                html_doc.text("Conclusions")
            with html_doc.tag("p"):
                html_doc.text(documentation.conclusions)
    return html_doc.doc.getvalue()


def to_html_reporting(documentation: ModelDocumentation) -> str:
    html_doc = HtmlDoc()
    with html_doc.tag("html"):
        with html_doc.tag("head"), html_doc.tag("title"):
            html_doc.text(documentation.title)
        html_doc.doc.asis(to_html(documentation.spaces))
        html_doc.doc.asis(to_html(documentation.constructions))
        html_doc.doc.asis(to_html(documentation.systems))
    return html_doc.doc.getvalue()
