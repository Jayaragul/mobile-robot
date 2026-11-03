# Simulation workbench quality baseline — v0.2

## Scope and status

This upgrade covers the **simulation software and operator controls**. It does not certify the simulated robot, create a hard real-time controller, or validate the phone/leg/motor assumptions. No ISO conformity or ROS package quality level is claimed.

## Implemented architecture

- `robot_lab.py`: the model/environment and deterministic physics integration step.
- `lab_runtime.py`: exclusive operator ownership, operating states, fixed-step worker, heartbeat watchdog, event trail and run provenance.
- `lab_server.py`: a local-only JSON/HTTP adapter. It binds only to 127.0.0.1, rejects unexpected request fields and foreign browser origins, and does not expose arbitrary files.
- `live-lab.html` and `lab-client.js`: presentation and commands. Browser rendering no longer determines how many physics steps occur.
- `replay_run.py`: replays the exported configuration and disturbances and checks final positions and velocities.
- `tests/test_controls.py`: lifecycle, fault recovery, numerical failure handling, request validation, ownership, watchdog, bounded catch-up and replay tests.
- `.github/workflows/simulator-tests.yml`: test jobs configured for Windows and Linux on Python 3.11. Remote CI success is separate evidence from local test success.

The Python worker targets wall-clock pacing of fixed 1–4 ms integration steps, with a 2 ms default. The operating system, Python scheduling and solver workload do **not** guarantee hard real-time execution. Wall-time backlog beyond 50 ms per worker tick is discarded and reported; physics steps themselves are never skipped. The real-time factor and maximum measured batch duration are visible in the UI.

## Operating states

| State | Allowed normal actions | Recovery |
| --- | --- | --- |
| READY | Start, single integration step, reset | — |
| RUNNING | Pause, disturbance, simulation stop | Pause before changing configuration |
| PAUSED | Start, single integration step, reset | — |
| STOPPED | Acknowledge | Then reset before Start |
| FAULT | Acknowledge | Then reset before Start |
| NEEDS_RESET | Reset | Start remains blocked |
| COMPLETED | Reset | Run has reached 30 simulated seconds |

Operator heartbeat is sent approximately every 300 ms. If the server has not received one for more than two seconds while running, it latches `operator_connection_lost`. Browser background throttling can trigger this intentionally conservative timeout. Reconnecting does not automatically resume motion.

Fall detection or physics exceptions latch a fault. A failed numerical step restores the prior position/velocity/time checkpoint for inspection; the run remains faulted. This is a diagnostic rollback, not evidence that an unstable solver has been corrected.

The stop control **freezes the simulation**. It does not model real braking, stop distance, a power disconnect, or a safety-rated emergency-stop circuit. A UUID operator identifier prevents accidental conflicting tabs; it is not authentication suitable for an exposed network service. Keep this service local.

## Run evidence and reproducibility

The complete-run JSON export includes schema version, run ID, UTC creation time, configuration, engine/library versions, source SHA-256 hashes, model hash, physics step count, final generalized state, applied disturbances, recent bounded event history, and sampled telemetry. Disturbances are indexed by integration step so pauses and browser timing do not change when they are replayed.

Replay verifies the model hash, then compares final joint/base positions and velocities with a 1e-7 numerical tolerance. The automated test demonstrates matching replay on this installed platform. Exact or tolerance-level reproducibility across different CPUs, library versions or operating systems has not been established. Source hashes are recorded for traceability; a matching model hash alone does not establish that all controller code is unchanged.

Telemetry is finite-run, bounded by the 30-second experiment limit. The event trail retains up to 256 events. Numerical data in the current model remains idealized; traceable logs do not make its assumptions physically accurate.

## Checks completed locally

Thirteen automated tests cover:

1. Ready/start/pause/step/reset transitions and rejection of configuration changes while running.
2. Latched stop, acknowledgement, reset, and blocked automatic restart.
3. Watchdog fault independent of browser rendering.
4. Operator conflict rejection and stop availability to another local operator.
5. Invalid configuration preserving the active model.
6. Bounded wall-time catch-up and diagnostic reporting.
7. Numerical exception handling with a serializable inspection state.
8. Replay including an applied disturbance and model-hash mismatch rejection.
9. Invalid, boolean and non-finite configuration input rejection.
10. Local health endpoint and static client delivery.
11. Foreign-origin and unknown-field rejection.
12. Removal of the legacy browser-driven `/api/step` endpoint.
13. Run export schema availability.

## Remaining work before a production claim

- Establish quantified timing, reliability and performance targets on a specified deployment machine.
- Add longer randomized fault/scenario campaigns, coverage targets, load testing, browser integration tests in CI, dependency vulnerability review, and release/signing procedures.
- Validate contact geometry, self-collisions, sensing delays/noise, actuator curves and thermal/electrical models against measured hardware.
- Add proper foot-placement and recovery control. Current faster gaits still slip.
- If hardware integration is required, define a hardware abstraction and command/feedback contract, separate real-time actuator control from the UI, and evaluate ROS 2/ros2_control or an equivalent architecture. Selecting ROS does not by itself make a system production-ready.

## Engineering references

[ROS REP-2004](https://reps.openrobotics.org/rep-2004/) describes quality criteria including version policy, change control, documentation and testing. It is a useful reference, not a quality level this non-ROS project currently claims.

[ros2_control controller-manager documentation](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html) illustrates separation of controller lifecycle, hardware interfaces, limits and timing diagnostics. The linked Rolling documentation describes a development branch; any future integration should choose and pin a supported release.

[MuJoCo Python documentation](https://mujoco.readthedocs.io/en/stable/python.html) documents the underlying simulation API.
