#
<div style="display: flex; align-items: center;">
  <img src="img/trano.webp" alt="Image" style="width: 120px; height: 140px; margin-right: 10px;">
  <span style="color: #FFBF00; font-size: 96px; font-weight: bold;">Trano</span>
</div>

Trano is a package designed to streamline the implementation of building energy simulation models using a simplified YAML input format. It leverages existing validated Modelica libraries, including the [Buildings library](https://simulationresearch.lbl.gov/modelica/) and the [IDEAS library](https://github.com/open-ideas/IDEAS), as well as other Modelica libraries.

Trano is designed to minimize the learning curve associated with developing building energy simulation models using Modelica. It streamlines the process by abstracting the complexities of building energy simulation, allowing users to input only the specific information relevant to their buildings. Trano automates the construction of the energy simulation models, aiming to enhance and scale the adoption of building energy simulations with Modelica efficiently.

It is library-agnostic and can generate models using virtually any library. Currently, it is compatible with the Buildings library and the IDeas library. While Trano can execute simulations, models generated with it can also be opened and simulated in Modelica editors such as [OpenModelica](https://openmodelica.org) and [Dymola](https://www.3ds.com/products-services/dymola/), provided that the required libraries are preloaded. Models generated with Trano include valid graphical representations, allowing users to modify them through the graphical interface of Modelica editors. A set of [tutorials](tutorials/first_simulation.md) is available to help users get started.
