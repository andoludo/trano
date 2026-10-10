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
tables are rendered from the last results with `python -m validation.bestest report --docs`.

## Buildings

| Case | KPI | trano | Band | Reference mean | Status |
|---|---|---|---|---|---|
| 600 | annual_heating | 4.456 MWh | 3.750 to 4.980 | 4.213 | pass |
| 600 | annual_cooling | 5.973 MWh | 5.000 to 6.830 | 5.856 | pass |
| 600 | peak_heating | 3.221 kW | 2.852 to 3.527 | 3.183 | pass |
| 600 | peak_cooling | 6.188 kW | 5.098 to 6.805 | 6.024 | pass |
| 610 | annual_heating | 4.483 MWh | 3.610 to 5.270 | 4.311 | pass |
| 610 | annual_cooling | 4.810 MWh | 2.740 to 6.030 | 4.256 | pass |
| 610 | peak_heating | 3.221 kW | 2.853 to 3.528 | 3.168 | pass |
| 610 | peak_cooling | 6.082 kW | 5.009 to 6.754 | 5.861 | pass |
| 620 | annual_heating | 4.571 MWh | 3.670 to 5.380 | 4.413 | pass |
| 620 | annual_cooling | 4.088 MWh | 2.760 to 5.190 | 4.090 | pass |
| 620 | peak_heating | 3.253 kW | 2.869 to 3.554 | 3.186 | pass |
| 620 | peak_cooling | 4.702 kW | 3.715 to 5.037 | 4.526 | pass |
| 630 | annual_heating | 4.754 MWh | 3.690 to 6.120 | 4.822 | pass |
| 630 | annual_cooling | 3.321 MWh | 1.080 to 4.420 | 2.814 | pass |
| 630 | peak_heating | 3.255 kW | 2.870 to 3.557 | 3.203 | pass |
| 630 | peak_cooling | 4.251 kW | 3.315 to 4.423 | 3.963 | pass |
| 640 | annual_heating | 2.721 MWh | 1.580 to 3.760 | 2.612 | pass |
| 640 | annual_cooling | 5.725 MWh | 4.440 to 6.860 | 5.636 | pass |
| 640 | peak_heating | 4.363 kW | 3.806 to 4.891 | 4.369 | pass |
| 640 | peak_cooling | 6.139 kW | 5.044 to 6.750 | 5.973 | pass |
| 650 | annual_heating | 0.000 MWh | 0.000 to 0.000 | 0.000 | pass |
| 650 | annual_cooling | 4.802 MWh | 3.460 to 5.880 | 4.644 | pass |
| 650 | peak_heating | 0.000 kW | 0.000 to 0.000 | 0.000 | pass |
| 650 | peak_cooling | 5.950 kW | 4.731 to 6.604 | 5.813 | pass |
| 660 | annual_heating | 3.627 MWh | 2.680 to 4.820 | 3.713 | pass |
| 660 | annual_cooling | 3.315 MWh | 1.910 to 4.330 | 3.172 | pass |
| 660 | peak_heating | 2.733 kW | 2.472 to 3.103 | 2.801 | pass |
| 660 | peak_cooling | 3.630 kW | 3.146 to 4.130 | 3.565 | pass |
| 670 | annual_heating | 6.481 MWh | 4.000 to 7.960 | 5.681 | pass |
| 670 | annual_cooling | 6.361 MWh | 5.050 to 7.670 | 6.402 | pass |
| 670 | peak_heating | 4.297 kW | 3.444 to 4.432 | 3.943 | pass |
| 670 | peak_cooling | 6.561 kW | 5.493 to 7.271 | 6.445 | pass |
| 680 | annual_heating | 2.234 MWh | 1.210 to 3.080 | 2.056 | pass |
| 680 | annual_cooling | 6.135 MWh | 5.130 to 7.700 | 6.264 | pass |
| 680 | peak_heating | 2.017 kW | 1.672 to 2.232 | 1.984 | pass |
| 680 | peak_cooling | 6.519 kW | 5.408 to 7.404 | 6.446 | pass |
| 685 | annual_heating | 4.940 MWh | 4.080 to 5.750 | 4.762 | pass |
| 685 | annual_cooling | 8.947 MWh | 7.700 to 10.140 | 8.886 | pass |
| 685 | peak_heating | 3.230 kW | 2.863 to 3.543 | 3.183 | pass |
| 685 | peak_cooling | 6.933 kW | 5.713 to 7.517 | 6.743 | pass |
| 695 | annual_heating | 2.775 MWh | 1.700 to 3.810 | 2.656 | pass |
| 695 | annual_cooling | 8.754 MWh | 7.490 to 10.580 | 8.912 | pass |
| 695 | peak_heating | 2.052 kW | 1.688 to 2.245 | 2.001 | pass |
| 695 | peak_cooling | 7.082 kW | 5.855 to 7.918 | 6.979 | pass |
| 900 | annual_heating | 1.723 MWh | 1.040 to 2.280 | 1.627 | pass |
| 900 | annual_cooling | 2.394 MWh | 2.350 to 2.600 | 2.467 | pass |
| 900 | peak_heating | 2.672 kW | 2.304 to 2.917 | 2.591 | pass |
| 900 | peak_cooling | 2.967 kW | 2.387 to 3.545 | 2.975 | pass |
| 910 | annual_heating | 1.893 MWh | 1.560 to 2.300 | 1.971 | pass |
| 910 | annual_cooling | 1.614 MWh | 0.860 to 2.000 | 1.374 | pass |
| 910 | peak_heating | 2.680 kW | 2.329 to 2.939 | 2.648 | pass |
| 910 | peak_cooling | 2.280 kW | 1.945 to 2.858 | 2.305 | pass |
| 920 | annual_heating | 3.331 MWh | 2.550 to 4.200 | 3.326 | pass |
| 920 | annual_cooling | 2.663 MWh | 2.430 to 3.080 | 2.786 | pass |
| 920 | peak_heating | 2.747 kW | 2.367 to 3.040 | 2.710 | pass |
| 920 | peak_cooling | 3.225 kW | 2.536 to 3.655 | 3.127 | pass |
| 930 | annual_heating | 3.752 MWh | 2.750 to 5.350 | 4.064 | pass |
| 930 | annual_cooling | 2.177 MWh | 1.240 to 2.640 | 1.898 | pass |
| 930 | peak_heating | 2.757 kW | 2.389 to 3.116 | 2.751 | pass |
| 930 | peak_cooling | 2.870 kW | 2.182 to 3.205 | 2.656 | pass |
| 940 | annual_heating | 1.104 MWh | 0.220 to 1.910 | 1.109 | pass |
| 940 | annual_cooling | 2.328 MWh | 2.240 to 3.140 | 2.401 | pass |
| 940 | peak_heating | 3.278 kW | 2.858 to 4.076 | 3.377 | pass |
| 940 | peak_cooling | 2.967 kW | 2.387 to 3.545 | 2.993 | pass |
| 950 | annual_heating | 0.000 MWh | 0.000 to 0.000 | 0.000 | pass |
| 950 | annual_cooling | 0.717 MWh | 0.430 to 1.520 | 0.634 | pass |
| 950 | peak_heating | 0.000 kW | 0.000 to 0.000 | 0.000 | pass |
| 950 | peak_cooling | 2.378 kW | 1.935 to 2.507 | 2.268 | pass |
| 960 | annual_heating | 2.556 MWh | 2.000 to 3.400 | 2.693 | pass |
| 960 | annual_cooling | 0.973 MWh | 0.620 to 1.810 | 0.896 | pass |
| 960 | peak_heating | 2.098 kW | 1.970 to 2.415 | 2.195 | pass |
| 960 | peak_cooling | 1.488 kW | 1.264 to 1.554 | 1.393 | pass |
| 960 | maximum_temperature | 48.649 degC | 47.100 to 54.200 | 49.920 | pass |
| 960 | minimum_temperature | 4.344 degC | 3.200 to 9.000 | 5.800 | pass |
| 960 | mean_temperature | 27.082 degC | 25.800 to 30.500 | 28.060 | pass |
| 980 | annual_heating | 0.444 MWh | -0.610 to 1.280 | 0.407 | pass |
| 980 | annual_cooling | 3.411 MWh | 3.520 to 4.490 | 3.710 | known deviation: Buildings' own Case980 gives 3.418 MWh, 3 % below the lower limit of the standard (3.52 MWh); trano reproduces the library's model. |
| 980 | peak_heating | 1.542 kW | 1.169 to 1.778 | 1.489 | pass |
| 980 | peak_cooling | 3.295 kW | 2.747 to 3.851 | 3.348 | pass |
| 985 | annual_heating | 2.377 MWh | 1.680 to 3.090 | 2.398 | pass |
| 985 | annual_cooling | 6.139 MWh | 5.950 to 7.260 | 6.351 | pass |
| 985 | peak_heating | 2.670 kW | 2.313 to 2.924 | 2.631 | pass |
| 985 | peak_cooling | 3.865 kW | 2.997 to 4.436 | 3.824 | pass |
| 995 | annual_heating | 0.988 MWh | -0.150 to 2.020 | 0.974 | pass |
| 995 | annual_cooling | 6.786 MWh | 6.580 to 8.410 | 7.145 | pass |
| 995 | peak_heating | 1.602 kW | 1.284 to 1.797 | 1.565 | pass |
| 995 | peak_cooling | 3.972 kW | 3.104 to 4.435 | 3.986 | pass |
| 600FF | maximum_temperature | 63.305 degC | 61.400 to 69.400 | 64.600 | pass |
| 600FF | minimum_temperature | -12.857 degC | -14.800 to -8.900 | -12.700 | pass |
| 600FF | mean_temperature | 24.652 degC | 23.300 to 27.100 | 25.250 | pass |
| 650FF | maximum_temperature | 62.126 degC | 60.100 to 67.800 | 63.067 | pass |
| 650FF | minimum_temperature | -16.945 degC | -18.800 to -15.700 | -17.333 | pass |
| 650FF | mean_temperature | 18.725 degC | 16.600 to 19.900 | 18.300 | pass |
| 680FF | maximum_temperature | 68.870 degC | 68.800 to 79.500 | 73.017 | pass |
| 680FF | minimum_temperature | -7.339 degC | -9.100 to -4.700 | -6.867 | pass |
| 680FF | mean_temperature | 30.115 degC | 29.200 to 34.300 | 31.850 | pass |
| 900FF | maximum_temperature | 43.827 degC | 42.300 to 47.000 | 44.583 | pass |
| 900FF | minimum_temperature | 1.085 degC | -0.400 to 3.200 | 1.250 | pass |
| 900FF | mean_temperature | 24.921 degC | 23.500 to 26.700 | 25.233 | pass |
| 950FF | maximum_temperature | 36.779 degC | 35.100 to 38.100 | 36.583 | pass |
| 950FF | minimum_temperature | -12.071 degC | -14.400 to -11.500 | -12.983 | pass |
| 950FF | mean_temperature | 15.230 degC | 13.400 to 16.000 | 14.733 | pass |
| 980FF | maximum_temperature | 48.644 degC | 47.500 to 53.800 | 50.500 | pass |
| 980FF | minimum_temperature | 9.607 degC | 6.300 to 13.500 | 10.350 | pass |
| 980FF | mean_temperature | 30.449 degC | 29.500 to 34.300 | 31.800 | pass |
