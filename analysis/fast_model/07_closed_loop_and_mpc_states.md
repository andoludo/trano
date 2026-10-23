# 7. The closed loop and the MPC model share their states

**Question.** mpctrano runs the closed-loop FastModel model to bring a building to a realistic state ("spin-up") and
then starts the MPC model from it, copying the states that have the same name. After the walls of the closed loop
were split into sublayers (as Buildings splits them), is that copy still right?

**Answer.** It was not, and it is now. With the sublayers, `T[2]` of a wall was the boundary between the first two
layers in the MPC model but a node inside the first layer in the closed loop. mpctrano copied the closed-loop value
into the MPC model, which then started with the inner surfaces of its walls at the temperature of a node inside the
insulation. Its flexibility test found the heat pump at full power all day and no room to run harder. The walls now
name `T` the layer boundaries in both libraries and `TSub` the nodes inside the layers (closed loop only); every state
of the MPC model is a state of the closed loop under the same name and with the same meaning.

Data: the `state_names` block of [`results/mpc_residential_after_renovation.json`](results/mpc_residential_after_renovation.json)
and [`results/mpc_residential_before_renovation.json`](results/mpc_residential_before_renovation.json); test
`tests/test_fast_model.py::test_the_closed_loop_and_mpc_models_share_the_layer_boundaries`.

## How it was found

The full mpctrano suite against the branch passed 119 tests and failed three: the flexibility envelope, and two tests
of the YAML parameters that named one layer conductance by its index (`G[2]`), which the sublayers renumbered. The
flexibility test passed with the released trano 0.58.0. On the heat-pump house of mpctrano's tests, after a two-day
spin-up, the baseline MPC plan ran the heat pump at 0.61 of its power in the first hour, leaving 0.75 kWh of upward
flexibility over an hour; with the branch it ran at 1.0 at 06:00, 10:00 and 14:00 alike, with no flexibility left.
Running the same probe against each library commit of the branch (one git worktree per commit, trano installed from
each) gave 0.61 up to the window change (b6ee2b8) and 1.0 from the discretised walls (229dc99) on. mpctrano's
`spin_up` copies the states of the closed loop into the MPC model by name, and the discretisation had given the same
names to different nodes.

## The fix

* `PartialOpaque` holds one heat balance per node of the wall. The construction record says which node is the boundary
  of each layer (`nod`) and which nodes are inside the layers (`sub`); the templates write the arrays out, since
  rumoca 0.10.3 does not size arrays from computed values.
* The public states are `T[nLay + 1]` (the layer boundaries, as before the sublayers, in both libraries) and
  `TSub[nNod − nLay]` (the nodes inside the layers, closed loop only).
* The results did not change: BESTEST within 2.4e-5, the renovated house within 1e-5.
* mpctrano: the two tests that named a conductance by its index now use the wall resistance and every sublayer of a
  calibrated layer (branch `fix/closed-loop-groups-by-wall-resistance` of mpctrano).

## Checks now

| | After renovation | Before renovation |
|---|---|---|
| States of the MPC model | 189 | 160 |
| of which wall and window nodes (`.T[`) | 179 | 152 |
| MPC states missing in the closed loop | 0 | 0 |
| States of the closed loop | 278 | 224 |
| of which nodes inside the layers (`TSub`) | 64 | 43 |

mpctrano against the head of the branch (trano installed from it): 26 tests of the YAML parameters, the flexibility,
the project, the calibration and the model pass (`tests/test_yaml_parameters.py tests/test_flexibility.py
tests/test_project_cli.py tests/test_calibration.py tests/test_model.py`, 11 minutes).
