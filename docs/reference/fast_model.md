# FastSmoothBuildings

[FastSmoothBuildings](https://github.com/andoludo/FastSmoothBuildings) is a Modelica library of buildings and their
systems written as smooth, event-free, explicit ordinary differential equations: in closed loop it follows the
Buildings library, and with the controls of its systems as inputs [rumoca](https://github.com/CogniPilot/rumoca)
translates it into a CasADi model for model predictive control, state estimation and calibration. It was trano's
`FastModel` library and now lives in its own repository, released on PyPI as `fastsmoothbuildings`, which trano
depends on.

trano generates models with it as with Buildings and IDEAS, in two configurations of the same templates:

* **`FastSmoothBuildings`**, the closed loop: the controllers of the YAML are part of the model;
* **`FastSmoothBuildingsMPC`**, for optimal control: the controls of the systems are inputs of the model.

```bash
trano create-model house.yaml FastSmoothBuildings       # closed loop
trano create-model house.yaml FastSmoothBuildingsMPC    # the model for optimal control
```

The generated package embeds the library (without its examples and documentation). The former names `FastModel` and
`FastModelMPC` are still accepted, with a deprecation warning.

The [documentation of FastSmoothBuildings](https://andoludo.github.io/FastSmoothBuildings/) describes its components
and their equations, class by class, its validation against Buildings and ASHRAE 140, and the analysis behind it. The
[FastSmoothBuildings tutorial](../tutorials/fast_model.md) shows how to use the generated models in CasADi.
