"""The ``FastModel`` library: connectable Modelica components that translate into an explicit CasADi ODE.

trano assembles the components (:mod:`trano.fast_model.components`) into a ``building_mpc`` model from the YAML
description of a building: multilayer zones with the physics of the Buildings ``MixedAir`` zone, or RC zones, with
their hydronic circuits, ventilation units and PV arrays. The model is close to the Buildings one and still exports
to CasADi with rumoca, for simulation, state estimation and optimal control with IPOPT
(:mod:`trano.fast_model.optimal_control`). The flat RC models of the ``mpc`` library are left as they are.
"""
