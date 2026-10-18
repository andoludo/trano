"""Simulation of the generated models: the modules of trano-simulate, under their former trano names.

``trano.simulate.simulate``, ``trano.simulate.result``, ``trano.simulate.install_libraries`` and
``trano.simulate.exceptions`` are the modules of ``trano_simulate`` itself (not copies), so that the code importing
or patching them keeps working. New code imports ``trano_simulate``.
"""

import importlib
import sys

for _name in ("exceptions", "install_libraries", "result", "simulate"):
    # by sys.modules: the package trano_simulate exports a function ``simulate`` that hides its module
    _module = importlib.import_module(f"trano_simulate.{_name}")
    sys.modules[f"{__name__}.{_name}"] = _module
    globals()[_name] = _module
