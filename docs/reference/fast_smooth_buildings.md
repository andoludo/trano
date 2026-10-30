---
description: "FastSmoothBuildings: a smooth Modelica building library close to Buildings, translated to CasADi by rumoca for simulation, state estimation and optimal control."
---
# FastSmoothBuildings

[FastSmoothBuildings](https://github.com/andoludo/FastSmoothBuildings) is a Modelica library of buildings and their
systems written as smooth, event-free, explicit ordinary differential equations: in closed loop it follows the
Buildings library, and with the controls of its systems as inputs [rumoca](https://github.com/CogniPilot/rumoca)
translates it into a CasADi model for model predictive control, state estimation and calibration. It lives in its
own repository, released on PyPI as `fastsmoothbuildings`, which trano depends on.

trano generates models with it as with Buildings and IDEAS, in two configurations of the same templates:

* **`FastSmoothBuildings`**, the closed loop: the controllers of the YAML are part of the model;
* **`FastSmoothBuildingsMPC`**, for optimal control: the controls of the systems are inputs of the model.

```bash
trano create-model house.yaml FastSmoothBuildings       # closed loop
trano create-model house.yaml FastSmoothBuildingsMPC    # the model for optimal control
```

As with Buildings and IDEAS, the generated package does not hold the library: it declares
`uses(FastSmoothBuildings(version=...))`, and the library is loaded beside it from the installed `fastsmoothbuildings`
package. `Network.libraries` gives its folder: trano-simulate loads it before the model (mounted in the container),
rumoca reads it as a root (`CasadiModel.from_modelica(text, name, roots=network.libraries)`). To open a generated
package in OMEdit or Dymola, add the folder that holds the library to the `MODELICAPATH`:

```bash
python -c "import fastsmoothbuildings; print(fastsmoothbuildings.ROOT)"
```

`--embed-libraries` (`Library.embed_libraries = True`) writes the library into the package instead (without its
examples and documentation): a single self-contained file to send.

The [documentation of FastSmoothBuildings](https://andoludo.github.io/FastSmoothBuildings/) describes its components
and their equations, class by class, its validation against Buildings and ASHRAE 140, and the analysis behind it. The
[FastSmoothBuildings tutorial](../tutorials/fast_smooth_buildings.md) shows how to use the generated models in CasADi.
