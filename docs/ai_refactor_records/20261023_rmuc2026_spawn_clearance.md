# RMUC 2026 CAD robot spawn clearance

## Diagnosis

The legacy spawn is `(4.234, 7.3)`. Its nearest occupied static-map cell centre
is `(3.875, 7.325)`, 0.35987 m away. Clearance to the occupied cell rectangle
is only 0.334 m. A visually non-contacting CAD model can therefore fail a
conservative circular navigation check.

An isolated Gazebo run with the legacy spawn and MINCO collision distance
0.44 m reproduced SMAC exhausting its open set after one expansion while
planning to `(4.6, 7.3)`. See `log/spawn_stuck_diagnosis/ab044.log`.
This reproduces the search blockage; it does not establish that every
recovery-state transition is caused by this parameter.

The current source profile already separates `chassis_radius: 0.321` from
`collision_radius: 0.44`. `configure_navigation` uses the former for MINCO's
collision distance/corridor and the latter for costmap inflation. The original
spawn can pass with 0.321 m, but leaves only about 13 mm static-map clearance
beyond that radius, before localization and discretization errors.

## Change

- Add an RMUC 2026 `real_spawn` at `(4.6, 7.3, 0.11)`, retaining yaw 0.04.
  The real-profile height offset remains 0.037 m. The legacy omni spawn is unchanged.
- Static-map cell-edge clearance becomes 0.700 m, exceeding the full 0.44 m
  envelope plus 0.15 m margin. Collision dimensions and radius settings are unchanged.
- Log the effective model spawn, chassis radius, and full envelope radius at launch.
- Add a spawn-clearance regression and assertions for the effective MINCO radii.
  Update stale geometry-count assertions to the current four-core/sixteen-edge
  chassis (38 total primitives); CAD coverage and ground-clearance checks remain.

## Validation

`python3 -m unittest discover -s src/nav-simulation/sentry_simulation/test
-p test_real_robot_profile.py -v`: 10 tests passed. Substituting the original
spawn makes the new clearance regression fail at `0.334 > 0.59`.

`colcon test --packages-select sentry_simulation --event-handlers console_cohesion+`
and `colcon test-result --test-result-base build/sentry_simulation --verbose`:
zero failures.

GUI Gazebo + RViz acceptance used ROS domain 89 and Gazebo partition
`spawn_stuck_final_v2`, with current profile and a fresh world. Ground-truth
initial XY was `(4.599582, 7.300406)`; initial localization error was 9.7 mm.
Navigation to `(4.5, 8.5)` and back to `(4.6, 7.3)` passed, with settled
ground-truth errors 5.1 mm and 63.3 mm. The 15 + 14 repeated goal submissions
all succeeded. `acceptance/result.json` has no violations. Logs and screenshots
are under `log/spawn_stuck_diagnosis/`.

The conservative 0.44 m control run was repeated with only the spawn changed
to `(4.6, 7.3)`: both navigation actions succeeded and the one-expansion
blockage disappeared. That separate run failed localization/accuracy limits
(initial localization error 0.124 m, first settled truth error 0.183 m), so it
is evidence for removing spawn blockage, not an accuracy acceptance. The
accepted GUI run above uses the unchanged current 0.321 m chassis profile.

An exploratory target `(6.0, 7.3)` was rejected as cost 253 (inflated obstacle),
and the probe canceled it after timeout. A longer exploratory route to
`(12.611, 10.593)` also failed; these routes are not claimed as validated.
This change addresses spawn clearance, not arbitrary-route feasibility.

Run the accepted configuration with:

```bash
./start_sim.sh world:=rmuc_2026 robot_model:=real
```

Launch resolves parameters once: restart existing navigation processes to apply
the new spawn. Use a fresh Gazebo partition for isolated tests; orphan servers
sharing a partition can leave probes observing the previous world.
