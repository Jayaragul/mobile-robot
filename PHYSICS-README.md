# Phone biped physics concept

Open `phone-biped-physics.html` in a browser. It is self-contained, with recorded MuJoCo videos and telemetry embedded. It is a replay, not a live browser solver.

`stand.mp4`, `step.mp4`, and `push.mp4` are separate video exports. CSV files contain 20 ms telemetry, including floor contact loads, sole-centre heights, tilt and actuator torque. `physics-results.json` contains six test summaries; `physics-checks.json` contains verification results.

## Reproduce

Requires Python with `mujoco`, `numpy`, `pillow` and `imageio-ffmpeg` installed.

```text
python simulate_robot.py
python simulate_robot.py --render
```

The first command reruns physics without rendering. The second also exports video. Run from any directory; output files are written beside the script. The script regenerates `phone-biped-model.xml`. The HTML replay is a snapshot of this delivered run and does not automatically update when experiments are rerun.

## Model boundaries

- Units: metres, kilograms, seconds, newton-metres. X is forward, Y is left, Z is up.
- 14 actuated hinge axes, a free-floating base, gravity and torque-limited position servos. No root position is prescribed and no tether or supporting constraint is used.
- Estimated dimensions, mass distribution, friction and motor limits. They are not measured or selected hardware specifications.
- Collision masks enable robot–floor contact but disable robot self-contact. Overlapping joint housing proxies must be refined before enabling self-collision.
- Simplified rectangular links and phone, inertias computed from proxy geometry. Neck pan is a command, not a simulated visual person-tracking system.
- Basic open-loop joint targets with local ideal servo feedback, not an IMU/contact-based balancing controller. No torque–speed curve, motor thermal model, gearbox backlash, battery drain, latency, noise or terrain model.
- Fall criterion: body tilt >45 degrees or pelvis height <100 mm. Push case: 12 N at pelvis COM, forward, for 160 ms beginning at 2 s.
- Finite six-second experiments do not establish robust stability. Forward travel can contain foot slip. No body-turning gait, obstacle avoidance, stairs or floor-edge behaviour is tested.

## Observations

With a 250 g phone and friction coefficient 0.8, standing and phone pan remain upright. The stepping trial advances approximately 9.8 cm in six seconds; only 12% of sampled frames have exactly one sole in contact, so this is an early shuffle. The push case reaches the fall criterion at 2.236 seconds and saturates the assumed leg torque limit.

The mean standing normal force matches the 16.589 N model weight. A 1 ms rerun gives 9.39 cm travel versus 9.78 cm at 2 ms, a 4.0% difference: useful preliminary consistency, not a full convergence proof.

Engine documentation: https://mujoco.readthedocs.io/en/stable/modeling.html
