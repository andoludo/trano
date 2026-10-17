"""Calibration-parameter inventory of the exported FMUs: ``python -m validation.fmu --help``.

Exports the baseline FMU of a YAML building, resolves every numeric field of the YAML to the FMU
parameters it sets, classifies them from the model description and runs the readback, impact and
equivalence checks of :mod:`trano.fmu.checks`. Writes ``parameter_inventory_<library>.csv``.
"""
