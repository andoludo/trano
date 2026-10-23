"""Overhangs and side fins of FastModel: the sunlit fractions against their closed form, and a shaded case in CasADi.

The functions ``Envelope.Functions.overhangSunlitFraction`` and ``sideFinsSunlitFraction`` (ports of Buildings'
``Overhang`` and ``SideFins``) are evaluated through rumoca and CasADi on a window 2 m high and 3 m wide under an
overhang 1 m deep, flush with its top and 10 m wider on each side, and between two fins 1 m deep, flush with its sides:

* where the shadow has a closed form (the sun facing the wall: the overhang shades ``dep*tan(alt)``, the fins
  nothing; the sun below the horizon or behind the wall: all in the shade), the fractions are compared with it;
* along the altitude and the azimuth, the fractions are tabulated to show that they are smooth.

BESTEST 630 (east and west windows with overhangs and fins) is then exported as FastModelMPC and as the closed loop,
and its derivatives are evaluated in the sun at several hours: they must be finite.

    python -m analysis.fast_model.scripts.shading_check

Needs trano-optimization with rumoca >= 0.10.3. Writes ``shading_check.json``.
"""

from __future__ import annotations

import argparse
import math
import warnings
from typing import Any

import numpy as np

from analysis.fast_model.scripts._common import BESTEST_CASES, KELVIN, configure_logging, write_json
from trano.data_models.conversion import convert_network
from trano.elements.jinja import modelica_library
from trano.elements.library.library import Library

HEIGHT, WIDTH, DEPTH = 2.0, 3.0, 1.0
PROBE = f"""
model Shade
  input Real verAzi;
  input Real alt;
  Real overhang(start=0, fixed=true);
  Real fins(start=0, fixed=true);
equation
  der(overhang) = FastModel.Envelope.Functions.overhangSunlitFraction(verAzi, alt, 0, {HEIGHT}, {WIDTH}, {DEPTH}, 0.0,
    10.0, 10.0);
  der(fins) = FastModel.Envelope.Functions.sideFinsSunlitFraction(verAzi, alt, {HEIGHT}, {WIDTH}, {DEPTH}, 0.0, 0.0);
end Shade;
"""
WEATHER = ("TDryBul", "TBlaSky", "winSpe", "winDir", "HDirNor", "HDifHor", "HGloHor")
SUNNY = dict(zip(WEATHER, (KELVIN + 15, KELVIN + 5, 3.0, math.pi, 700.0, 100.0, 500.0), strict=True))


def closed_form(ver_azi: float, alt: float) -> tuple[float, float] | None:
    """Sunlit fractions (overhang, fins) where the geometry gives them directly, else None."""
    if alt <= 0 or abs(ver_azi) >= math.pi / 2:
        return 0.0, 0.0
    if ver_azi == 0:
        return 1 - min(DEPTH * math.tan(alt), HEIGHT) / HEIGHT, 1.0
    if math.isclose(ver_azi, math.pi / 4) and math.isclose(alt, 0.05):
        # a low sun 45 degrees aside: the overhang shades a strip of dep*tan(alt)/cos(verAzi), a fin 1 m of the width
        # but for the triangle above its shadow (0.672 by the superposition of Buildings)
        return 1 - DEPTH * math.tan(alt) / math.cos(ver_azi) / HEIGHT, 0.672
    return None


def fractions() -> dict[str, Any]:
    from trano_optimization import CasadiModel

    model = CasadiModel.from_modelica(f"{modelica_library('FastModel')}\n{PROBE}", "Shade")

    def evaluate(ver_azi: float, alt: float) -> tuple[float, float]:
        values = model.derivatives(0.0, np.zeros(2), np.array([ver_azi, alt]))
        return float(values[0]), float(values[1])

    checks = []
    for ver_azi, alt in ((0.0, math.pi / 4), (0.0, math.pi / 3), (math.pi / 4, 0.05), (0.0, -0.2), (2.0, 0.5)):
        expected = closed_form(ver_azi, alt)
        overhang, fins = evaluate(ver_azi, alt)
        checks.append(
            {"ver_azi_rad": ver_azi, "alt_rad": alt, "overhang": overhang, "fins": fins}
            | ({"overhang_closed_form": expected[0], "fins_closed_form": expected[1]} if expected else {})
        )
    altitude = [
        {"alt_deg": degrees} | dict(zip(("overhang", "fins"), evaluate(0.0, math.radians(degrees)), strict=True))
        for degrees in range(-10, 85, 5)
    ]
    azimuth = [
        {"ver_azi_deg": degrees}
        | dict(zip(("overhang", "fins"), evaluate(math.radians(degrees), math.radians(30)), strict=True))
        for degrees in range(0, 125, 5)
    ]
    largest_gap = max(
        abs(row[key] - row[f"{key}_closed_form"])
        for row in checks
        if "overhang_closed_form" in row
        for key in ("overhang", "fins")
    )
    return {
        "checks": checks,
        "largest_gap_to_closed_form": largest_gap,
        "along_altitude_facing": altitude,
        "along_azimuth_alt_30": azimuth,
    }


def shaded_case() -> dict[str, Any]:
    from trano_optimization import CasadiModel

    path = BESTEST_CASES / "case_630.yaml"
    out: dict[str, Any] = {"yaml": "validation/bestest/cases/case_630.yaml"}
    for library in ("FastModelMPC", "FastModel"):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            text = convert_network(path.stem, path, library=Library.from_configuration(library)).model(
                include_container=True
            )
        model = CasadiModel.from_modelica(text, f"{path.stem}.building_mpc")
        description = model.description
        inputs = np.array([SUNNY.get(name.rsplit(".", 1)[-1], 0.0) for name in description.input_names])
        finite = {
            hour: bool(np.all(np.isfinite(model.derivatives(3600.0 * hour, description.initial_state, inputs))))
            for hour in (0, 6, 9, 12, 15, 18)
        }
        out[library] = {
            "states": len(description.states),
            "inputs": len(description.input_names),
            "shading_parameters": sorted(
                name
                for name in description.parameter_names
                if name.rsplit(".", 1)[-1] in {"depOve", "gapOve", "wLOve", "wROve", "depFin", "gapFin", "hFin"}
            ),
            "finite_derivatives_by_hour": finite,
        }
    return out


def main(argv: list[str] | None = None) -> None:
    argparse.ArgumentParser(description=__doc__.splitlines()[0]).parse_args(argv)
    configure_logging()
    write_json("shading_check.json", {"fractions": fractions(), "bestest_630": shaded_case()})


if __name__ == "__main__":
    main()
