from pathlib import Path

from docs_src.utils import Tutorial, Text

INDEXES = [
    Tutorial(
        title="""
<div style="display: flex; align-items: center;">
  <img src="img/trano.webp" alt="Image" style="width: 120px; height: 140px; margin-right: 10px;">
  <span style="color: #FFBF00; font-size: 96px; font-weight: bold;">Trano</span>
</div>
""",
        contents=[
            Text(
                content="""
**Trano** is an innovative Python package that automates the creation of complex **Building Energy Simulation 
(BES)** Modelica models from simplified information contained in widely used data formats like **YAML, JSON, or 
RDF**. Unlike traditional tools that directly convert **BIM** to **BES**, Trano introduces an intermediate step, 
paving the way for seamless integration with **IFC translators**.  

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
![building.jpg](docs/img/building.jpg)
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
