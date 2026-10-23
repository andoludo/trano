# 5. Overhangs and side fins

**Question.** Can FastModel shade its windows as Buildings does, smoothly enough for CasADi, so that the four BESTEST
cases with shading run and pass?

**Answer.** Yes. The sunlit fraction of Buildings' `Overhang` and `SideFins` was ported to Modelica functions of
FastModel with smooth minima, maxima and steps; evaluated through rumoca and CasADi they match the closed form of the
shadow within 0.0006; the shaded BESTEST case 630 exports to CasADi with finite derivatives at every hour tried; the
four shading cases pass all 16 KPIs, within 2.2 % of Buildings.

Data: [`results/shading_check.json`](results/shading_check.json), the shading rows of
[`results/bestest_kpis.csv`](results/bestest_kpis.csv); script [`scripts/shading_check.py`](scripts/shading_check.py);
test `tests/test_fast_model.py::test_overhangs_and_side_fins_shade_as_buildings`.

## Implementation

* `FastModel.Envelope.Functions.overhangSunlitFraction(verAzi, alt, side, hWin, wWin, dep, gap, wL, wR)`: the
  superposition of four rectangles of Buildings, `min`/`max` replaced by `smoothMin`/`smoothMax` over a width of a
  hundredth of the window, `tan` written as a sine over a cosine kept away from zero, the side of the sun chosen by
  `regStep`, and the window all in the shade (`regStep` over 0.01 rad) when the sun is behind the wall or below the
  horizon.
* `sideFinsSunlitFraction(verAzi, alt, hWin, wWin, dep, gap, h)`: the same for the two fins.
* The window (`Envelope.Window`) computes the solar altitude, the angle of the sun to the normal in the horizontal plane
  and the side of the sun from the weather bus, and multiplies the **direct** irradiance absorbed by the panes and
  transmitted to the room by the sunlit fraction; the diffuse irradiance is not shaded, as in Buildings. With both an
  overhang and fins the fractions combine as Buildings combines them, `smoothMax(f_ove + f_fin − 1, 1e-3)`.
* The geometry comes from the YAML (`overhang`, `side_fins` of a window) through the FastModel template; a window
  without shading keeps a fraction of 1 through evaluated Boolean parameters (`haveOverhang`, `haveSideFins`), so the
  other models are unchanged. The geometry parameters are tunable in the CasADi model (14 of them in case 630).

## Checks

Window 2 m high, 3 m wide; overhang 1 m deep, flush with the top, 10 m wider on each side; fins 1 m deep, flush with
the sides:

| Sun | Overhang, FastModel | closed form | Fins, FastModel | closed form |
|---|---|---|---|---|
| facing the wall, 45 degrees high | 0.5000 | 1 − tan(45°)/2 = 0.5000 | 1.0000 | 1 |
| facing, 60 degrees high | 0.1340 | 1 − tan(60°)/2 = 0.1340 | 1.0000 | 1 |
| 45 degrees aside, 0.05 rad high | 0.9646 | 1 − tan(0.05)/cos(45°)/2 = 0.9646 | 0.6726 | 0.672 |
| below the horizon | 0.0000 | 0 | 0.0000 | 0 |
| behind the wall | 0.0000 | 0 | 0.0000 | 0 |

Along the altitude (sun facing the wall) the overhang fraction falls smoothly from 0.96 at 5 degrees to 0.13 at 60 and
0 from 65 degrees; at the horizon itself it is 0.5, the middle of the `regStep` that puts the window in the shade
below it. Along the azimuth at 30 degrees of altitude the fins go from 1 (sun facing) to 0 at 90 degrees, smoothly
(the tables are in `shading_check.json`).

BESTEST 630 (east and west windows with overhangs and fins) exported by rumoca: 27 states and 10 inputs as
FastModelMPC, 60 states and 9 inputs in closed loop; the derivatives are finite at 0, 6, 9, 12, 15 and 18 h with
700 W/m2 of direct sun.

## BESTEST

| Case | Shading | Annual heating | Annual cooling | Peak heating | Peak cooling |
|---|---|---|---|---|---|
| 610 | south overhang | +0.3 % | +1.1 % | +0.2 % | +1.0 % |
| 630 | east and west overhangs and fins | +0.5 % | +0.2 % | +0.2 % | +0.4 % |
| 910 | south overhang, heavy | −0.2 % | +1.4 % | +0.8 % | +2.2 % |
| 930 | east and west overhangs and fins, heavy | +0.9 % | +0.8 % | +0.8 % | +0.5 % |

(FastModel against Buildings; all inside their bands.) The direction of the effect of shading between two cases
(610 against 600, 630 against 620, 910 against 900, 930 against 920) is that of every reference program
(`tests/test_bestest_results.py`).
