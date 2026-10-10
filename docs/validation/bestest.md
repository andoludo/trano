# ASHRAE 140 (BESTEST) validation

The building models trano generates are validated against the 27 cases of section 5.2 of ASHRAE
Standard 140-2020 (the BESTEST single-zone cases): a 8 m x 6 m x 2.7 m zone in Denver with light
or heavy constructions, 12 m2 of double glazing, 0.414 air changes per hour of infiltration and
200 W of internal gains, with variants for the insulation, the glazing, the window orientation,
overhangs and side fins, an unconditioned sun-space, free-floating temperatures, a dual set point
ideal system with set-backs and night ventilation.

Each case is a trano YAML file (`validation/bestest/cases/`), simulated for a year with each
library. The annual heating and cooling loads must fall inside the acceptance limits of the
standard; the peak loads and the free-floating temperatures must fall inside the spread of the
reference programs (BSIMAC, CSE, DeST, EnergyPlus, ESP-r, TRNSYS) widened by 5 % of the largest
peak or 1 K. Buildings and IDEAS are gating libraries: `pytest -m bestest` fails when one of
their cases leaves its band, except for the deviations listed below with their reason. The
reduced-order (AixLib) and ISO 13790 zones are shown for information: ISO 13790 is a monthly
method applied hour by hour and the reduced-order zone still deviates on several cases, so
neither gates. Cases a library cannot describe (shading with IDEAS, which OpenModelica cannot
compile for IDEAS 3.0.0; shading, sun-space and night ventilation with the two simplified zones)
are left out. The tables are rendered from the last results with
`python -m validation.bestest report --docs`. [A walk through the BESTEST cases](bestest_cases.md) explains a few
cases in detail: what they are, what is expected, how the YAML describes them and how the results
compare.

## Buildings

| Case | KPI | trano | Band | Reference mean | Status |
|---|---|---|---|---|---|
| 600 | annual_heating | 4.449 MWh | 3.750 to 4.980 | 4.213 | pass |
| 600 | annual_cooling | 5.973 MWh | 5.000 to 6.830 | 5.856 | pass |
| 600 | peak_heating | 3.215 kW | 2.852 to 3.527 | 3.183 | pass |
| 600 | peak_cooling | 6.187 kW | 5.098 to 6.805 | 6.024 | pass |
| 610 | annual_heating | 4.475 MWh | 3.610 to 5.270 | 4.311 | pass |
| 610 | annual_cooling | 4.810 MWh | 2.740 to 6.030 | 4.256 | pass |
| 610 | peak_heating | 3.216 kW | 2.853 to 3.528 | 3.168 | pass |
| 610 | peak_cooling | 6.082 kW | 5.009 to 6.754 | 5.861 | pass |
| 620 | annual_heating | 4.563 MWh | 3.670 to 5.380 | 4.413 | pass |
| 620 | annual_cooling | 4.087 MWh | 2.760 to 5.190 | 4.090 | pass |
| 620 | peak_heating | 3.248 kW | 2.869 to 3.554 | 3.186 | pass |
| 620 | peak_cooling | 4.700 kW | 3.715 to 5.037 | 4.526 | pass |
| 630 | annual_heating | 4.746 MWh | 3.690 to 6.120 | 4.822 | pass |
| 630 | annual_cooling | 3.320 MWh | 1.080 to 4.420 | 2.814 | pass |
| 630 | peak_heating | 3.250 kW | 2.870 to 3.557 | 3.203 | pass |
| 630 | peak_cooling | 4.250 kW | 3.315 to 4.423 | 3.963 | pass |
| 640 | annual_heating | 2.715 MWh | 1.580 to 3.760 | 2.612 | pass |
| 640 | annual_cooling | 5.725 MWh | 4.440 to 6.860 | 5.636 | pass |
| 640 | peak_heating | 4.356 kW | 3.806 to 4.891 | 4.369 | pass |
| 640 | peak_cooling | 6.138 kW | 5.044 to 6.750 | 5.973 | pass |
| 650 | annual_heating | 0.000 MWh | 0.000 to 0.000 | 0.000 | pass |
| 650 | annual_cooling | 4.802 MWh | 3.460 to 5.880 | 4.644 | pass |
| 650 | peak_heating | 0.000 kW | 0.000 to 0.000 | 0.000 | pass |
| 650 | peak_cooling | 5.949 kW | 4.731 to 6.604 | 5.813 | pass |
| 660 | annual_heating | 3.620 MWh | 2.680 to 4.820 | 3.713 | pass |
| 660 | annual_cooling | 3.315 MWh | 1.910 to 4.330 | 3.172 | pass |
| 660 | peak_heating | 2.728 kW | 2.472 to 3.103 | 2.801 | pass |
| 660 | peak_cooling | 3.629 kW | 3.146 to 4.130 | 3.565 | pass |
| 670 | annual_heating | 6.473 MWh | 4.000 to 7.960 | 5.681 | pass |
| 670 | annual_cooling | 6.361 MWh | 5.050 to 7.670 | 6.402 | pass |
| 670 | peak_heating | 4.294 kW | 3.444 to 4.432 | 3.943 | pass |
| 670 | peak_cooling | 6.560 kW | 5.493 to 7.271 | 6.445 | pass |
| 680 | annual_heating | 2.227 MWh | 1.210 to 3.080 | 2.056 | pass |
| 680 | annual_cooling | 6.136 MWh | 5.130 to 7.700 | 6.264 | pass |
| 680 | peak_heating | 2.013 kW | 1.672 to 2.232 | 1.984 | pass |
| 680 | peak_cooling | 6.518 kW | 5.408 to 7.404 | 6.446 | pass |
| 685 | annual_heating | 4.932 MWh | 4.080 to 5.750 | 4.762 | pass |
| 685 | annual_cooling | 8.946 MWh | 7.700 to 10.140 | 8.886 | pass |
| 685 | peak_heating | 3.225 kW | 2.863 to 3.543 | 3.183 | pass |
| 685 | peak_cooling | 6.932 kW | 5.713 to 7.517 | 6.743 | pass |
| 695 | annual_heating | 2.768 MWh | 1.700 to 3.810 | 2.656 | pass |
| 695 | annual_cooling | 8.754 MWh | 7.490 to 10.580 | 8.912 | pass |
| 695 | peak_heating | 2.047 kW | 1.688 to 2.245 | 2.001 | pass |
| 695 | peak_cooling | 7.080 kW | 5.855 to 7.918 | 6.979 | pass |
| 900 | annual_heating | 1.718 MWh | 1.040 to 2.280 | 1.627 | pass |
| 900 | annual_cooling | 2.395 MWh | 2.350 to 2.600 | 2.467 | pass |
| 900 | peak_heating | 2.668 kW | 2.304 to 2.917 | 2.591 | pass |
| 900 | peak_cooling | 2.974 kW | 2.387 to 3.545 | 2.975 | pass |
| 910 | annual_heating | 1.887 MWh | 1.560 to 2.300 | 1.971 | pass |
| 910 | annual_cooling | 1.615 MWh | 0.860 to 2.000 | 1.374 | pass |
| 910 | peak_heating | 2.676 kW | 2.329 to 2.939 | 2.648 | pass |
| 910 | peak_cooling | 2.280 kW | 1.945 to 2.858 | 2.305 | pass |
| 920 | annual_heating | 3.324 MWh | 2.550 to 4.200 | 3.326 | pass |
| 920 | annual_cooling | 2.663 MWh | 2.430 to 3.080 | 2.786 | pass |
| 920 | peak_heating | 2.743 kW | 2.367 to 3.040 | 2.710 | pass |
| 920 | peak_cooling | 3.223 kW | 2.536 to 3.655 | 3.127 | pass |
| 930 | annual_heating | 3.745 MWh | 2.750 to 5.350 | 4.064 | pass |
| 930 | annual_cooling | 2.177 MWh | 1.240 to 2.640 | 1.898 | pass |
| 930 | peak_heating | 2.753 kW | 2.389 to 3.116 | 2.751 | pass |
| 930 | peak_cooling | 2.869 kW | 2.182 to 3.205 | 2.656 | pass |
| 940 | annual_heating | 1.100 MWh | 0.220 to 1.910 | 1.109 | pass |
| 940 | annual_cooling | 2.329 MWh | 2.240 to 3.140 | 2.401 | pass |
| 940 | peak_heating | 3.274 kW | 2.858 to 4.076 | 3.377 | pass |
| 940 | peak_cooling | 2.974 kW | 2.387 to 3.545 | 2.993 | pass |
| 950 | annual_heating | 0.000 MWh | 0.000 to 0.000 | 0.000 | pass |
| 950 | annual_cooling | 0.717 MWh | 0.430 to 1.520 | 0.634 | pass |
| 950 | peak_heating | 0.000 kW | 0.000 to 0.000 | 0.000 | pass |
| 950 | peak_cooling | 2.385 kW | 1.935 to 2.507 | 2.268 | pass |
| 960 | annual_heating | 2.588 MWh | 2.000 to 3.400 | 2.693 | pass |
| 960 | annual_cooling | 0.950 MWh | 0.620 to 1.810 | 0.896 | pass |
| 960 | peak_heating | 2.104 kW | 1.970 to 2.415 | 2.195 | pass |
| 960 | peak_cooling | 1.475 kW | 1.264 to 1.554 | 1.393 | pass |
| 960 | maximum_temperature | 48.036 degC | 47.100 to 54.200 | 49.920 | pass |
| 960 | minimum_temperature | 4.160 degC | 3.200 to 9.000 | 5.800 | pass |
| 960 | mean_temperature | 26.693 degC | 25.800 to 30.500 | 28.060 | pass |
| 980 | annual_heating | 0.441 MWh | -0.610 to 1.280 | 0.407 | pass |
| 980 | annual_cooling | 3.415 MWh | 3.520 to 4.490 | 3.710 | known deviation: Buildings' own Case980 gives 3.418 MWh, 3 % below the lower limit of the standard (3.52 MWh); trano reproduces the library's model. |
| 980 | peak_heating | 1.538 kW | 1.169 to 1.778 | 1.489 | pass |
| 980 | peak_cooling | 3.296 kW | 2.747 to 3.851 | 3.348 | pass |
| 985 | annual_heating | 2.371 MWh | 1.680 to 3.090 | 2.398 | pass |
| 985 | annual_cooling | 6.140 MWh | 5.950 to 7.260 | 6.351 | pass |
| 985 | peak_heating | 2.666 kW | 2.313 to 2.924 | 2.631 | pass |
| 985 | peak_cooling | 3.863 kW | 2.997 to 4.436 | 3.824 | pass |
| 995 | annual_heating | 0.983 MWh | -0.150 to 2.020 | 0.974 | pass |
| 995 | annual_cooling | 6.789 MWh | 6.580 to 8.410 | 7.145 | pass |
| 995 | peak_heating | 1.598 kW | 1.284 to 1.797 | 1.565 | pass |
| 995 | peak_cooling | 3.973 kW | 3.104 to 4.435 | 3.986 | pass |
| 600FF | maximum_temperature | 63.318 degC | 61.400 to 69.400 | 64.600 | pass |
| 600FF | minimum_temperature | -12.847 degC | -14.800 to -8.900 | -12.700 | pass |
| 600FF | mean_temperature | 24.664 degC | 23.300 to 27.100 | 25.250 | pass |
| 650FF | maximum_temperature | 62.135 degC | 60.100 to 67.800 | 63.067 | pass |
| 650FF | minimum_temperature | -16.922 degC | -18.800 to -15.700 | -17.333 | pass |
| 650FF | mean_temperature | 18.733 degC | 16.600 to 19.900 | 18.300 | pass |
| 680FF | maximum_temperature | 68.913 degC | 68.800 to 79.500 | 73.017 | pass |
| 680FF | minimum_temperature | -7.313 degC | -9.100 to -4.700 | -6.867 | pass |
| 680FF | mean_temperature | 30.137 degC | 29.200 to 34.300 | 31.850 | pass |
| 900FF | maximum_temperature | 43.840 degC | 42.300 to 47.000 | 44.583 | pass |
| 900FF | minimum_temperature | 1.107 degC | -0.400 to 3.200 | 1.250 | pass |
| 900FF | mean_temperature | 24.932 degC | 23.500 to 26.700 | 25.233 | pass |
| 950FF | maximum_temperature | 36.790 degC | 35.100 to 38.100 | 36.583 | pass |
| 950FF | minimum_temperature | -12.046 degC | -14.400 to -11.500 | -12.983 | pass |
| 950FF | mean_temperature | 15.243 degC | 13.400 to 16.000 | 14.733 | pass |
| 980FF | maximum_temperature | 48.663 degC | 47.500 to 53.800 | 50.500 | pass |
| 980FF | minimum_temperature | 9.634 degC | 6.300 to 13.500 | 10.350 | pass |
| 980FF | mean_temperature | 30.470 degC | 29.500 to 34.300 | 31.800 | pass |

## IDEAS

| Case | KPI | trano | Band | Reference mean | Status |
|---|---|---|---|---|---|
| 600 | annual_heating | 4.541 MWh | 3.750 to 4.980 | 4.213 | pass |
| 600 | annual_cooling | 6.265 MWh | 5.000 to 6.830 | 5.856 | pass |
| 600 | peak_heating | 3.314 kW | 2.852 to 3.527 | 3.183 | pass |
| 600 | peak_cooling | 6.545 kW | 5.098 to 6.805 | 6.024 | pass |
| 620 | annual_heating | 4.727 MWh | 3.670 to 5.380 | 4.413 | pass |
| 620 | annual_cooling | 4.235 MWh | 2.760 to 5.190 | 4.090 | pass |
| 620 | peak_heating | 3.373 kW | 2.869 to 3.554 | 3.186 | pass |
| 620 | peak_cooling | 4.908 kW | 3.715 to 5.037 | 4.526 | pass |
| 640 | annual_heating | 2.769 MWh | 1.580 to 3.760 | 2.612 | pass |
| 640 | annual_cooling | 6.009 MWh | 4.440 to 6.860 | 5.636 | pass |
| 640 | peak_heating | 4.370 kW | 3.806 to 4.891 | 4.369 | pass |
| 640 | peak_cooling | 6.491 kW | 5.044 to 6.750 | 5.973 | pass |
| 650 | annual_heating | 0.000 MWh | 0.000 to 0.000 | 0.000 | pass |
| 650 | annual_cooling | 5.067 MWh | 3.460 to 5.880 | 4.644 | pass |
| 650 | peak_heating | 0.000 kW | 0.000 to 0.000 | 0.000 | pass |
| 650 | peak_cooling | 6.324 kW | 4.731 to 6.604 | 5.813 | pass |
| 660 | annual_heating | 3.723 MWh | 2.680 to 4.820 | 3.713 | pass |
| 660 | annual_cooling | 3.438 MWh | 1.910 to 4.330 | 3.172 | pass |
| 660 | peak_heating | 2.759 kW | 2.472 to 3.103 | 2.801 | pass |
| 660 | peak_cooling | 3.873 kW | 3.146 to 4.130 | 3.565 | pass |
| 670 | annual_heating | 6.119 MWh | 4.000 to 7.960 | 5.681 | pass |
| 670 | annual_cooling | 6.683 MWh | 5.050 to 7.670 | 6.402 | pass |
| 670 | peak_heating | 4.166 kW | 3.444 to 4.432 | 3.943 | pass |
| 670 | peak_cooling | 6.899 kW | 5.493 to 7.271 | 6.445 | pass |
| 680 | annual_heating | 2.359 MWh | 1.210 to 3.080 | 2.056 | pass |
| 680 | annual_cooling | 6.642 MWh | 5.130 to 7.700 | 6.264 | pass |
| 680 | peak_heating | 2.144 kW | 1.672 to 2.232 | 1.984 | pass |
| 680 | peak_cooling | 6.964 kW | 5.408 to 7.404 | 6.446 | pass |
| 685 | annual_heating | 5.024 MWh | 4.080 to 5.750 | 4.762 | pass |
| 685 | annual_cooling | 9.271 MWh | 7.700 to 10.140 | 8.886 | pass |
| 685 | peak_heating | 3.329 kW | 2.863 to 3.543 | 3.183 | pass |
| 685 | peak_cooling | 7.282 kW | 5.713 to 7.517 | 6.743 | pass |
| 695 | annual_heating | 2.927 MWh | 1.700 to 3.810 | 2.656 | pass |
| 695 | annual_cooling | 9.316 MWh | 7.490 to 10.580 | 8.912 | pass |
| 695 | peak_heating | 2.185 kW | 1.688 to 2.245 | 2.001 | pass |
| 695 | peak_cooling | 7.527 kW | 5.855 to 7.918 | 6.979 | pass |
| 900 | annual_heating | 1.763 MWh | 1.040 to 2.280 | 1.627 | pass |
| 900 | annual_cooling | 2.599 MWh | 2.350 to 2.600 | 2.467 | pass |
| 900 | peak_heating | 2.765 kW | 2.304 to 2.917 | 2.591 | pass |
| 900 | peak_cooling | 3.171 kW | 2.387 to 3.545 | 2.975 | pass |
| 920 | annual_heating | 3.493 MWh | 2.550 to 4.200 | 3.326 | pass |
| 920 | annual_cooling | 2.820 MWh | 2.430 to 3.080 | 2.786 | pass |
| 920 | peak_heating | 2.861 kW | 2.367 to 3.040 | 2.710 | pass |
| 920 | peak_cooling | 3.388 kW | 2.536 to 3.655 | 3.127 | pass |
| 940 | annual_heating | 1.126 MWh | 0.220 to 1.910 | 1.109 | pass |
| 940 | annual_cooling | 2.524 MWh | 2.240 to 3.140 | 2.401 | pass |
| 940 | peak_heating | 3.253 kW | 2.858 to 4.076 | 3.377 | pass |
| 940 | peak_cooling | 3.170 kW | 2.387 to 3.545 | 2.993 | pass |
| 950 | annual_heating | 0.000 MWh | 0.000 to 0.000 | 0.000 | pass |
| 950 | annual_cooling | 0.741 MWh | 0.430 to 1.520 | 0.634 | pass |
| 950 | peak_heating | 0.000 kW | 0.000 to 0.000 | 0.000 | pass |
| 950 | peak_cooling | 2.519 kW | 1.935 to 2.507 | 2.268 | known deviation: 2.517 kW against a band ending at 2.507 kW (the spread of the reference programs plus 5 % of the largest peak): 0.4 % above a tolerance that is a convention, not a limit of the standard. |
| 960 | annual_heating | 2.695 MWh | 2.000 to 3.400 | 2.693 | pass |
| 960 | annual_cooling | 0.950 MWh | 0.620 to 1.810 | 0.896 | pass |
| 960 | peak_heating | 2.217 kW | 1.970 to 2.415 | 2.195 | pass |
| 960 | peak_cooling | 1.510 kW | 1.264 to 1.554 | 1.393 | pass |
| 960 | maximum_temperature | 51.309 degC | 47.100 to 54.200 | 49.920 | pass |
| 960 | minimum_temperature | 4.133 degC | 3.200 to 9.000 | 5.800 | pass |
| 960 | mean_temperature | 28.027 degC | 25.800 to 30.500 | 28.060 | pass |
| 980 | annual_heating | 0.480 MWh | -0.610 to 1.280 | 0.407 | pass |
| 980 | annual_cooling | 3.798 MWh | 3.520 to 4.490 | 3.710 | pass |
| 980 | peak_heating | 1.663 kW | 1.169 to 1.778 | 1.489 | pass |
| 980 | peak_cooling | 3.607 kW | 2.747 to 3.851 | 3.348 | pass |
| 985 | annual_heating | 2.464 MWh | 1.680 to 3.090 | 2.398 | pass |
| 985 | annual_cooling | 6.474 MWh | 5.950 to 7.260 | 6.351 | pass |
| 985 | peak_heating | 2.763 kW | 2.313 to 2.924 | 2.631 | pass |
| 985 | peak_cooling | 4.025 kW | 2.997 to 4.436 | 3.824 | pass |
| 995 | annual_heating | 1.063 MWh | -0.150 to 2.020 | 0.974 | pass |
| 995 | annual_cooling | 7.272 MWh | 6.580 to 8.410 | 7.145 | pass |
| 995 | peak_heating | 1.715 kW | 1.284 to 1.797 | 1.565 | pass |
| 995 | peak_cooling | 4.257 kW | 3.104 to 4.435 | 3.986 | pass |
| 600FF | maximum_temperature | 66.086 degC | 61.400 to 69.400 | 64.600 | pass |
| 600FF | minimum_temperature | -13.193 degC | -14.800 to -8.900 | -12.700 | pass |
| 600FF | mean_temperature | 25.109 degC | 23.300 to 27.100 | 25.250 | pass |
| 650FF | maximum_temperature | 64.789 degC | 60.100 to 67.800 | 63.067 | pass |
| 650FF | minimum_temperature | -16.889 degC | -18.800 to -15.700 | -17.333 | pass |
| 650FF | mean_temperature | 18.996 degC | 16.600 to 19.900 | 18.300 | pass |
| 680FF | maximum_temperature | 72.329 degC | 68.800 to 79.500 | 73.017 | pass |
| 680FF | minimum_temperature | -8.063 degC | -9.100 to -4.700 | -6.867 | pass |
| 680FF | mean_temperature | 30.991 degC | 29.200 to 34.300 | 31.850 | pass |
| 900FF | maximum_temperature | 44.753 degC | 42.300 to 47.000 | 44.583 | pass |
| 900FF | minimum_temperature | 0.814 degC | -0.400 to 3.200 | 1.250 | pass |
| 900FF | mean_temperature | 25.256 degC | 23.500 to 26.700 | 25.233 | pass |
| 950FF | maximum_temperature | 37.197 degC | 35.100 to 38.100 | 36.583 | pass |
| 950FF | minimum_temperature | -12.061 degC | -14.400 to -11.500 | -12.983 | pass |
| 950FF | mean_temperature | 15.341 degC | 13.400 to 16.000 | 14.733 | pass |
| 980FF | maximum_temperature | 50.242 degC | 47.500 to 53.800 | 50.500 | pass |
| 980FF | minimum_temperature | 9.351 degC | 6.300 to 13.500 | 10.350 | pass |
| 980FF | mean_temperature | 31.131 degC | 29.500 to 34.300 | 31.800 | pass |

## iso_13790

| Case | KPI | trano | Band | Reference mean | Status |
|---|---|---|---|---|---|
| 600 | annual_heating | 3.122 MWh | 3.750 to 4.980 | 4.213 | fail |
| 600 | annual_cooling | 5.298 MWh | 5.000 to 6.830 | 5.856 | pass |
| 600 | peak_heating | 3.099 kW | 2.852 to 3.527 | 3.183 | pass |
| 600 | peak_cooling | 4.870 kW | 5.098 to 6.805 | 6.024 | fail |
| 620 | annual_heating | 3.920 MWh | 3.670 to 5.380 | 4.413 | pass |
| 620 | annual_cooling | 3.777 MWh | 2.760 to 5.190 | 4.090 | pass |
| 620 | peak_heating | 3.112 kW | 2.869 to 3.554 | 3.186 | pass |
| 620 | peak_cooling | 4.328 kW | 3.715 to 5.037 | 4.526 | pass |
| 640 | annual_heating | 1.879 MWh | 1.580 to 3.760 | 2.612 | pass |
| 640 | annual_cooling | 4.942 MWh | 4.440 to 6.860 | 5.636 | pass |
| 640 | peak_heating | 4.350 kW | 3.806 to 4.891 | 4.369 | pass |
| 640 | peak_cooling | 4.870 kW | 5.044 to 6.750 | 5.973 | fail |
| 660 | annual_heating | 2.660 MWh | 2.680 to 4.820 | 3.713 | fail |
| 660 | annual_cooling | 2.612 MWh | 1.910 to 4.330 | 3.172 | pass |
| 660 | peak_heating | 2.557 kW | 2.472 to 3.103 | 2.801 | pass |
| 660 | peak_cooling | 3.078 kW | 3.146 to 4.130 | 3.565 | fail |
| 670 | annual_heating | 5.029 MWh | 4.000 to 7.960 | 5.681 | pass |
| 670 | annual_cooling | 4.901 MWh | 5.050 to 7.670 | 6.402 | fail |
| 670 | peak_heating | 4.045 kW | 3.444 to 4.432 | 3.943 | pass |
| 670 | peak_cooling | 5.015 kW | 5.493 to 7.271 | 6.445 | fail |
| 680 | annual_heating | 1.242 MWh | 1.210 to 3.080 | 2.056 | pass |
| 680 | annual_cooling | 6.451 MWh | 5.130 to 7.700 | 6.264 | pass |
| 680 | peak_heating | 2.096 kW | 1.672 to 2.232 | 1.984 | pass |
| 680 | peak_cooling | 5.138 kW | 5.408 to 7.404 | 6.446 | fail |
| 685 | annual_heating | 4.137 MWh | 4.080 to 5.750 | 4.762 | pass |
| 685 | annual_cooling | 9.344 MWh | 7.700 to 10.140 | 8.886 | pass |
| 685 | peak_heating | 3.090 kW | 2.863 to 3.543 | 3.183 | pass |
| 685 | peak_cooling | 5.754 kW | 5.713 to 7.517 | 6.743 | pass |
| 695 | annual_heating | 2.256 MWh | 1.700 to 3.810 | 2.656 | pass |
| 695 | annual_cooling | 10.030 MWh | 7.490 to 10.580 | 8.912 | pass |
| 695 | peak_heating | 2.090 kW | 1.688 to 2.245 | 2.001 | pass |
| 695 | peak_cooling | 6.030 kW | 5.855 to 7.918 | 6.979 | pass |
| 900 | annual_heating | 1.803 MWh | 1.040 to 2.280 | 1.627 | pass |
| 900 | annual_cooling | 3.581 MWh | 2.350 to 2.600 | 2.467 | fail |
| 900 | peak_heating | 2.880 kW | 2.304 to 2.917 | 2.591 | pass |
| 900 | peak_cooling | 3.532 kW | 2.387 to 3.545 | 2.975 | pass |
| 920 | annual_heating | 3.440 MWh | 2.550 to 4.200 | 3.326 | pass |
| 920 | annual_cooling | 3.264 MWh | 2.430 to 3.080 | 2.786 | fail |
| 920 | peak_heating | 2.933 kW | 2.367 to 3.040 | 2.710 | pass |
| 920 | peak_cooling | 3.535 kW | 2.536 to 3.655 | 3.127 | pass |
| 940 | annual_heating | 1.276 MWh | 0.220 to 1.910 | 1.109 | pass |
| 940 | annual_cooling | 3.498 MWh | 2.240 to 3.140 | 2.401 | fail |
| 940 | peak_heating | 4.156 kW | 2.858 to 4.076 | 3.377 | fail |
| 940 | peak_cooling | 3.531 kW | 2.387 to 3.545 | 2.993 | pass |
| 980 | annual_heating | 0.532 MWh | -0.610 to 1.280 | 0.407 | pass |
| 980 | annual_cooling | 5.255 MWh | 3.520 to 4.490 | 3.710 | fail |
| 980 | peak_heating | 1.886 kW | 1.169 to 1.778 | 1.489 | fail |
| 980 | peak_cooling | 3.733 kW | 2.747 to 3.851 | 3.348 | pass |
| 985 | annual_heating | 2.730 MWh | 1.680 to 3.090 | 2.398 | pass |
| 985 | annual_cooling | 7.937 MWh | 5.950 to 7.260 | 6.351 | fail |
| 985 | peak_heating | 2.871 kW | 2.313 to 2.924 | 2.631 | pass |
| 985 | peak_cooling | 4.204 kW | 2.997 to 4.436 | 3.824 | pass |
| 995 | annual_heating | 1.195 MWh | -0.150 to 2.020 | 0.974 | pass |
| 995 | annual_cooling | 8.954 MWh | 6.580 to 8.410 | 7.145 | fail |
| 995 | peak_heating | 1.886 kW | 1.284 to 1.797 | 1.565 | fail |
| 995 | peak_cooling | 4.228 kW | 3.104 to 4.435 | 3.986 | pass |
| 600FF | maximum_temperature | 54.174 degC | 61.400 to 69.400 | 64.600 | fail |
| 600FF | minimum_temperature | -7.447 degC | -14.800 to -8.900 | -12.700 | fail |
| 600FF | mean_temperature | 26.564 degC | 23.300 to 27.100 | 25.250 | pass |
| 680FF | maximum_temperature | 61.503 degC | 68.800 to 79.500 | 73.017 | fail |
| 680FF | minimum_temperature | -0.611 degC | -9.100 to -4.700 | -6.867 | fail |
| 680FF | mean_temperature | 33.913 degC | 29.200 to 34.300 | 31.850 | pass |
| 900FF | maximum_temperature | 47.038 degC | 42.300 to 47.000 | 44.583 | fail |
| 900FF | minimum_temperature | 0.045 degC | -0.400 to 3.200 | 1.250 | pass |
| 900FF | mean_temperature | 26.610 degC | 23.500 to 26.700 | 25.233 | pass |
| 980FF | maximum_temperature | 54.832 degC | 47.500 to 53.800 | 50.500 | fail |
| 980FF | minimum_temperature | 8.231 degC | 6.300 to 13.500 | 10.350 | pass |
| 980FF | mean_temperature | 33.914 degC | 29.500 to 34.300 | 31.800 | pass |

## reduced_order

| Case | KPI | trano | Band | Reference mean | Status |
|---|---|---|---|---|---|
| 600 | annual_heating | 4.469 MWh | 3.750 to 4.980 | 4.213 | pass |
| 600 | annual_cooling | 4.267 MWh | 5.000 to 6.830 | 5.856 | fail |
| 600 | peak_heating | 3.020 kW | 2.852 to 3.527 | 3.183 | pass |
| 600 | peak_cooling | 4.180 kW | 5.098 to 6.805 | 6.024 | fail |
| 620 | annual_heating | 4.565 MWh | 3.670 to 5.380 | 4.413 | pass |
| 620 | annual_cooling | 2.997 MWh | 2.760 to 5.190 | 4.090 | pass |
| 620 | peak_heating | 3.020 kW | 2.869 to 3.554 | 3.186 | pass |
| 620 | peak_cooling | 3.688 kW | 3.715 to 5.037 | 4.526 | fail |
| 640 | annual_heating | 2.789 MWh | 1.580 to 3.760 | 2.612 | pass |
| 640 | annual_cooling | 4.166 MWh | 4.440 to 6.860 | 5.636 | fail |
| 640 | peak_heating | 2.753 kW | 3.806 to 4.891 | 4.369 | fail |
| 640 | peak_cooling | 4.126 kW | 5.044 to 6.750 | 5.973 | fail |
| 660 | annual_heating | 3.160 MWh | 2.680 to 4.820 | 3.713 | pass |
| 660 | annual_cooling | 2.585 MWh | 1.910 to 4.330 | 3.172 | pass |
| 660 | peak_heating | 2.245 kW | 2.472 to 3.103 | 2.801 | fail |
| 660 | peak_cooling | 2.585 kW | 3.146 to 4.130 | 3.565 | fail |
| 670 | annual_heating | 8.577 MWh | 4.000 to 7.960 | 5.681 | fail |
| 670 | annual_cooling | 3.153 MWh | 5.050 to 7.670 | 6.402 | fail |
| 670 | peak_heating | 4.996 kW | 3.444 to 4.432 | 3.943 | fail |
| 670 | peak_cooling | 4.123 kW | 5.493 to 7.271 | 6.445 | fail |
| 680 | annual_heating | 2.398 MWh | 1.210 to 3.080 | 2.056 | pass |
| 680 | annual_cooling | 5.025 MWh | 5.130 to 7.700 | 6.264 | fail |
| 680 | peak_heating | 1.892 kW | 1.672 to 2.232 | 1.984 | pass |
| 680 | peak_cooling | 4.716 kW | 5.408 to 7.404 | 6.446 | fail |
| 685 | annual_heating | 4.582 MWh | 4.080 to 5.750 | 4.762 | pass |
| 685 | annual_cooling | 6.474 MWh | 7.700 to 10.140 | 8.886 | fail |
| 685 | peak_heating | 3.005 kW | 2.863 to 3.543 | 3.183 | pass |
| 685 | peak_cooling | 4.783 kW | 5.713 to 7.517 | 6.743 | fail |
| 695 | annual_heating | 2.494 MWh | 1.700 to 3.810 | 2.656 | pass |
| 695 | annual_cooling | 6.717 MWh | 7.490 to 10.580 | 8.912 | fail |
| 695 | peak_heating | 1.881 kW | 1.688 to 2.245 | 2.001 | pass |
| 695 | peak_cooling | 5.140 kW | 5.855 to 7.918 | 6.979 | fail |
| 900 | annual_heating | 3.876 MWh | 1.040 to 2.280 | 1.627 | fail |
| 900 | annual_cooling | 3.558 MWh | 2.350 to 2.600 | 2.467 | fail |
| 900 | peak_heating | 3.016 kW | 2.304 to 2.917 | 2.591 | fail |
| 900 | peak_cooling | 3.655 kW | 2.387 to 3.545 | 2.975 | fail |
| 920 | annual_heating | 4.129 MWh | 2.550 to 4.200 | 3.326 | pass |
| 920 | annual_cooling | 2.529 MWh | 2.430 to 3.080 | 2.786 | pass |
| 920 | peak_heating | 3.016 kW | 2.367 to 3.040 | 2.710 | pass |
| 920 | peak_cooling | 3.260 kW | 2.536 to 3.655 | 3.127 | pass |
| 940 | annual_heating | 2.356 MWh | 0.220 to 1.910 | 1.109 | fail |
| 940 | annual_cooling | 3.426 MWh | 2.240 to 3.140 | 2.401 | fail |
| 940 | peak_heating | 2.489 kW | 2.858 to 4.076 | 3.377 | fail |
| 940 | peak_cooling | 3.640 kW | 2.387 to 3.545 | 2.993 | fail |
| 980 | annual_heating | 2.344 MWh | -0.610 to 1.280 | 0.407 | fail |
| 980 | annual_cooling | 4.957 MWh | 3.520 to 4.490 | 3.710 | fail |
| 980 | peak_heating | 1.892 kW | 1.169 to 1.778 | 1.489 | fail |
| 980 | peak_cooling | 4.661 kW | 2.747 to 3.851 | 3.348 | fail |
| 985 | annual_heating | 4.070 MWh | 1.680 to 3.090 | 2.398 | fail |
| 985 | annual_cooling | 5.959 MWh | 5.950 to 7.260 | 6.351 | pass |
| 985 | peak_heating | 3.000 kW | 2.313 to 2.924 | 2.631 | fail |
| 985 | peak_cooling | 4.303 kW | 2.997 to 4.436 | 3.824 | pass |
| 995 | annual_heating | 2.450 MWh | -0.150 to 2.020 | 0.974 | fail |
| 995 | annual_cooling | 6.659 MWh | 6.580 to 8.410 | 7.145 | pass |
| 995 | peak_heating | 1.881 kW | 1.284 to 1.797 | 1.565 | fail |
| 995 | peak_cooling | 5.093 kW | 3.104 to 4.435 | 3.986 | fail |
| 600FF | maximum_temperature | 68.891 degC | 61.400 to 69.400 | 64.600 | pass |
| 600FF | minimum_temperature | -13.741 degC | -14.800 to -8.900 | -12.700 | pass |
| 600FF | mean_temperature | 22.650 degC | 23.300 to 27.100 | 25.250 | fail |
| 680FF | maximum_temperature | 97.605 degC | 68.800 to 79.500 | 73.017 | fail |
| 680FF | minimum_temperature | -9.231 degC | -9.100 to -4.700 | -6.867 | fail |
| 680FF | mean_temperature | 28.843 degC | 29.200 to 34.300 | 31.850 | fail |
| 900FF | maximum_temperature | 61.583 degC | 42.300 to 47.000 | 44.583 | fail |
| 900FF | minimum_temperature | -8.562 degC | -0.400 to 3.200 | 1.250 | fail |
| 900FF | mean_temperature | 22.664 degC | 23.500 to 26.700 | 25.233 | fail |
| 980FF | maximum_temperature | 95.320 degC | 47.500 to 53.800 | 50.500 | fail |
| 980FF | minimum_temperature | -6.723 degC | 6.300 to 13.500 | 10.350 | fail |
| 980FF | mean_temperature | 28.816 degC | 29.500 to 34.300 | 31.800 | fail |
