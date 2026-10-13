"""Simulation performance benchmark of trano models: ``python -m validation.performance --help``.

A fixed set of models (envelope only and hydronic, with the Buildings and IDEAS libraries) is translated
and simulated in OpenModelica for each profile of performance options. The translation and the simulation
are timed separately, the solver statistics are collected, and the key outputs of each profile are compared
with the baseline profile. See docs/validation/simulation_performance.md.
"""
