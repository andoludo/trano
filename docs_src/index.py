from pathlib import Path

from docs_src.utils import CleanedText, Tutorial

INDEXES = [
    Tutorial(
        title="""
<div style="display: flex; align-items: center;">
  <img src="img/trano.webp" alt="Image" style="width: 120px; height: 140px; margin-right: 10px;">
  <span style="color: #FFBF00; font-size: 96px; font-weight: bold;">Trano</span>
</div>
""",
        contents=[
            CleanedText(
                content="""Trano is a package that aims at facilitating the implementation of Building energy
                simulation models through simplified yaml input description. It takes advantages of existing validated
                Modelica libraries such as the Buildings library (https://simulationresearch.lbl.gov/modelica/),
                the IDEAS library (https://github.com/open-ideas/IDEAS) or any other Modelica libraries."""
            ),
            CleanedText(
                content="""Trano aims at reducing the learning curve of building building energy simulation models
                using modelica and removes the overhead of the details of building energy simulation model in Modelica,
                the users only needs to provide information specific to the building. Trano takes care of building the
                energy simulation model for the user. Its goal is to be able to automate and facilitate the scaling of
                the use of building energy simulation model with Modelica."""
            ),
            CleanedText(
                content="""It is library agnostic and can generate model virtually using any libraries. For now
                it is compatible with Buildings library and IDeas library. Although Trano can run simulation,
                Model generated with trano can be opened and simulated  with Modelica editors such as OpenModelica
                and Dymola provided that the Libraries used in the model are prealably loaded in the library.
                The model genrated with trano includes valid graphical representation which facilitates the
                modification of the genrated model through Modelica editors graphical interface. A set of
                Tutorial(/tutorials/first_simulation.md) is provided to get users get started."""
            ),
        ],
    )
]


def gen_index_docs() -> None:
    for index in INDEXES:
        index.write(Path(__file__).parents[1].joinpath("docs", "index.md"))


if __name__ == "__main__":
    gen_index_docs()
