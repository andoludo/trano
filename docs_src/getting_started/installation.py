from pathlib import Path

from docs_src.utils import CleanedText, DisplayObject, TitleText, Tutorial, Text

INSTALLATIONS = [
    Tutorial(
        title="Installation",
        contents=[
            TitleText(content="""Optional dependencies"""),
            CleanedText(
                content="""### Graphviz
                Trano uses graphviz to position the different modelica components in the model.
                Although it is not necessary, it is recommended to install graphviz to have a better visualization 
                of the model.
                Graphviz can be installed by following the instructions on the official 
                website(https://graphviz.org/download/).
                For windows install, make sure that the graphviz bin folder is added to the system path."""
            ),
            CleanedText(
                content="""For linux, one can use the following command to install graphviz."""
            ),
            DisplayObject(
                language="bash",
                object="""
sudo apt update
sudo apt install graphviz
                """,
            ),
            CleanedText(
                content="""### Docker
    Similarly, trano uses the official openModelica docker image to run simulations. Therefore, it is recommended 
    to install docker on the system so
    trano can pull the necessary images and run simulations. One can install Docker by following the instructions 
    on the official
                website(https://docs.docker.com/engine/install/)."""
            ),
            TitleText(content="""Python package"""),
            Text(
                content="""
!!! warning
    Trano requires python 3.9 or higher and docker to be installed on the system.
            """
            ),
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
            CleanedText(
                content="""On windows systems, you may need to enable long path supports 
                (see https://learn.microsoft.com/en-us/windows/win32/fileio/maximum-file-path-limitation?tabs=registry)"""
            ),
            TitleText(content="""Check installation"""),
            CleanedText(
                content="""To check installation, run the following command in the terminal."""
            ),
            DisplayObject(
                language="bash",
                object="""trano verify""",
            ),
            CleanedText(
                content="""This command will check if your system is compatible for model generation and simulation.
                Being compatible for model simulation is optional if you only want to generate the Modelica model
                 only."""
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
