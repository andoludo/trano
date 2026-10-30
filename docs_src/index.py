from pathlib import Path

from docs_src.utils import Tutorial, Text

INDEXES = [
    Tutorial(
        title="Trano: building energy simulation models in Modelica, generated from YAML",
        page_title="Generate Modelica building energy simulation models from YAML",
        description=(
            "Trano is a Python tool that generates Modelica building energy simulation models (Buildings, IDEAS, "
            "AixLib, FastSmoothBuildings) from YAML, JSON or RDF, validated against ASHRAE 140 BESTEST and ready for "
            "OpenModelica, Dymola and model predictive control."
        ),
        contents=[
            Text(
                content="""
<img src="img/trano.webp" alt="Trano logo" style="width: 120px; height: 140px; float: left; margin-right: 16px;">

**Trano** is a Python package that generates **building energy simulation** (BES, building energy modelling) models 
in **Modelica** from a simple description of the building in **YAML, JSON or RDF**: envelope, HVAC systems and their 
controls, and electrical systems. The generated models are simulated with **OpenModelica** (or Dymola), validated 
against **ASHRAE 140 (BESTEST)**, and, with the FastSmoothBuildings library, translated to CasADi for **model 
predictive control (MPC)**. Unlike tools that directly convert **BIM** to BES, Trano introduces an intermediate, 
library-independent description, paving the way for seamless integration with **IFC translators**.  

Trano is **Modelica library agnostic** but is natively designed to work with:  
✅ **Validated detailed Modelica libraries** (e.g., **[Buildings](https://github.com/lbl-srg/modelica-buildings)**, 
**[IDEAS](https://github.com/open-ideas/IDEAS)**,...)  
✅ **Reduced-order models** (e.g., **[AIXLIB](https://github.com/RWTH-EBC/AixLib)**, **ISO13790**,...)  
✅ **your library...**  

## ✨ Key Features

### 🛠️ **Built for Open-Source BES**
- Designed with **widespread adoption** in mind.
- **Optimized for OpenModelica**, but also compatible with **Dymola**.

### 🔥 **Full Thermal & Electrical Modeling**
- Generates both **thermal** and **electrical** models.
- Supports **building envelope, systems, and electricity**.
- Models:
  - **Envelope** (geometry & materials) 🏢  
  - **HVAC systems** (emission, hydronic distribution, boilers) ❄️🔥  
  - **Electrical components** (PV systems, electrical loads) ⚡  

### 🎨 **Easy to Use & Modify**
- Generates **graphical representations** of components & connections 🎭.
![Graphical representation of a generated building model](img/building.jpg)
- Fully **modular design** for seamless modifications:
  - **Envelope** 🏠
  - **Emission** 💨
  - **Hydronic Distribution** 🚰
  - **Production & Electricity** ⚡  

🚀 **With Trano, creating and modifying detailed BES models has never been easier!**  

---
           """
            )
        ],
    )
]


def gen_index_docs() -> None:
    for index in INDEXES:
        index.write(Path(__file__).parents[1].joinpath("docs", "index.md"))


if __name__ == "__main__":
    gen_index_docs()
