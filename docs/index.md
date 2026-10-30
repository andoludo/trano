---
title: "Generate Modelica building energy simulation models from YAML"
description: "Trano is a Python tool that generates Modelica building energy simulation models (Buildings, IDEAS, AixLib, FastSmoothBuildings) from YAML, JSON or RDF, validated against ASHRAE 140 BESTEST and ready for OpenModelica, Dymola and model predictive control."
---
# Trano: building energy simulation models in Modelica, generated from YAML

<img src="img/trano.webp" alt="Trano logo" style="width: 120px; height: 140px; float: left; margin-right: 16px;">


**Trano** is a Python package that generates **building energy simulation** (BES, building energy modelling) models in **Modelica** from a simple description of the building in **YAML, JSON or RDF**: envelope, HVAC systems and their controls, and electrical systems. The generated models are simulated with **OpenModelica** (or Dymola), validated against **ASHRAE 140 (BESTEST)**, and, with the FastSmoothBuildings library, translated to CasADi for **model predictive control (MPC)**. Unlike tools that directly convert **BIM** to BES, Trano introduces an intermediate, library-independent description, paving the way for seamless integration with **IFC translators**.  

Trano is **Modelica library agnostic** but is natively designed to work with:  
✅ **Validated detailed Modelica libraries** (e.g., **[Buildings](https://github.com/lbl-srg/modelica-buildings)**, **[IDEAS](https://github.com/open-ideas/IDEAS)**,...)  
✅ **Reduced-order models** (e.g., **[AIXLIB](https://github.com/RWTH-EBC/AixLib)**, **ISO13790**,...)  
✅ **Models for optimal control** (**[FastSmoothBuildings](reference/fast_smooth_buildings.md)**: close to Buildings, closed loop or for optimal control, translated to CasADi)  
✅ **your library...**  

## 🧪 Supported libraries and validation

| Library | Version | Zone model | Role |
|---|---|---|---|
| [Buildings](https://github.com/lbl-srg/modelica-buildings) | 13.0.0 | `Buildings.ThermalZones.Detailed.MixedAir` | detailed, validated |
| [IDEAS](https://github.com/open-ideas/IDEAS) | 3.0.0 | `IDEAS.Buildings.Components.Zone` | detailed, validated |
| [AixLib](https://github.com/RWTH-EBC/AixLib) reduced order (`reduced_order`) | 3.0.1 | `AixLib.ThermalZones.ReducedOrder.ThermalZone` | simplified, informational |
| ISO 13790 (`iso_13790`) | AixLib 3.0.1 | `ISO13790.Zone5R1C` | simplified, informational |
| `FastSmoothBuildings` | - | `FastSmoothBuildings.Zones.Zone` (the physics of `MixedAir`), with the hydronic and ventilation systems and their controllers | close to Buildings, closed loop, CasADi ready |
| `FastSmoothBuildingsMPC` | - | the same, the controls of the systems as inputs | optimal control, CasADi ready |

The HVAC components (radiators, valves, pumps, boilers, heat pumps, air handling units) come from the Buildings
library for every Modelica library. The Modelica name of every parameter in every library is listed in the
[parameter reference](https://andoludo.github.io/trano/reference/parameters/).

The models trano generates are **validated against ASHRAE Standard 140 (BESTEST)**: the 27 single-zone cases
are simulated for a year with every library and compared with the acceptance limits of the standard and the
spread of the reference programs. Buildings and IDEAS pass every case (see the
[validation results](https://andoludo.github.io/trano/validation/bestest/) and the
[walk through the cases](https://andoludo.github.io/trano/validation/bestest_cases/)); the two simplified
zones are reported for information. Simulations run in the official OpenModelica image.

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
           

