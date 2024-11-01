from pathlib import Path

from docs_src.utils import CleanedText, DisplayObject, TitleText, Tutorial

INSTALLATIONS = [
    Tutorial(
        title="Installation",
        contents=[
            TitleText(content="""Python package"""),
            CleanedText(
                content="""Trano is a python package that can be installed using pip."""
            ),
            DisplayObject(language="bash", object="pip install trano"),
            CleanedText(content="""Trano can also be used using poetry."""),
            DisplayObject(language="bash", object="poetry add trano"),
            CleanedText(
                content="""The python package will allow to generate the Modelica model from the yaml
                configuration input.
                However, in order to be able to simulate model, Docker needs to be installed on the system."""
            ),
            TitleText(content="""Docker"""),
            CleanedText(
                content="""In order to simulate the Modelica model, Docker needs to be installed on the system.
                One can install Docker by following the instructions on the official
                website(https://docs.docker.com/engine/install/)."""
            ),
        ],
    )
]


def gen_installation_docs() -> None:
    for installation in INSTALLATIONS:
        installation.write(
            Path(__file__)
            .parents[2]
            .joinpath("docs", "getting-started", f"{installation.name()}.md")
        )


if __name__ == "__main__":
    gen_installation_docs()
