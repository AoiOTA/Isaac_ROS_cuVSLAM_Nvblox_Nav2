# Phase 2 Validation

Phase 2 was executed on 2026-07-17 with Isaac Sim 6.0.1.0 and the fixed local 6.0 assets. The composed stage exists only in memory and is never saved.

## Implemented contract

- The Warehouse root stage is opened once.
- `/World/NovaCarter` references the main `nova_carter.usd` from an anonymous session layer.
- Variants are fixed to `Physics=physx`, `Sensors=All_Sensors`, and `ROS=Disabled`.
- `Nova_Carter_ROS.usd` is rejected as an input and checked against the composed layer stack.
- The spawn is selected from actual collision-floor bounds using the padded robot footprint and nearby obstacle bounds.
- PhysX overlap queries validate the initial and final robot volume after the timeline starts.
- Stage identity and simulation time are monitored on every frame.
- A repository-scoped advisory lock rejects duplicate project instances without touching unrelated processes.

## Recorded acceptance runs

Headless command:

```bash
./scripts/run_sim.sh --headless --duration 60
```

Result:

| Measurement | Value |
|---|---:|
| Wall-clock simulation loop | 60.007 s |
| Frames | 6748 |
| Simulation-time advance | 112.467 s |
| Simulation-time regressions | 0 |
| Unexpected initial/final overlaps | 0 / 0 |
| Stage opens | 1 |
| Official asset fingerprints changed | no |

GUI command:

```bash
./scripts/run_sim.sh --gui --duration 10
```

Result:

| Measurement | Value |
|---|---:|
| Wall-clock simulation loop | 10.002 s |
| Frames | 1077 |
| Simulation-time advance | 17.950 s |
| Unexpected initial/final overlaps | 0 / 0 |
| X11 window class | `Isaac Sim Python 6.0.1` |
| X11 window owned by simulator PID | yes |

Both runs selected the first valid spawn at approximately `(-0.5, -0.5, 0.03)`, supported by `/World/Warehouse_Empty_small_realtime/SM_floor39/SM_floor02`. The search evaluated 26 collision-floor tiles against 463 obstacle bounds.

The duplicate-process test held the project lock in one process and invoked `navigation_sim.py` in a second process. The second invocation exited with status 73 before creating `SimulationApp`. An unrelated Isaac Sim project remained running throughout all tests.

Isaac Sim reports deprecation warnings for the official Hawk and Owl cameras' `fisheyePolynomial` projection. Phase 2 does not modify those official camera definitions; the camera pipeline is implemented in Phase 4.
