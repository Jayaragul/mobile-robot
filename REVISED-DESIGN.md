# Phone biped V2: compact geometry and body-angle feedback

The recommended next prototype retains two legs and 14 actuated axes. It uses a landscape phone nearer the pelvis, 70 mm thigh and shin links, 120 × 72 mm soles, 118 mm foot-centre spacing, a narrower battery 12 mm below the pelvis origin, and lighter link mass targets. Approximate height is 277 mm versus 367 mm, and initial centre of mass is 146 mm versus 169 mm. Estimated mass is 1.659 kg versus 1.691 kg.

The controller adds ideal pitch/roll measurements and angular-rate feedback to ankle targets. The original joint servo gains, assumed torque limits and gait period remain unchanged. This is not yet a full balance planner or recovery-step controller. No passive springs are installed in the selected V2.

## Evidence

For the same first 10 cm of travel, excluding the one-second standing setup, positive actuator work decreases from 5.228 J to 4.442 J (15.0%). The integral of squared actuator torque decreases from 3.353 to 2.690 Nm²s (19.8%). Time to the target decreases from 6.028 s to 5.498 s. No final stopping/settling phase is included.

Positive work is the sum over all actuators of the integral of max(torque × joint speed, 0). It excludes electrical losses and does not credit regeneration. Torque-squared integral is an effort/heating proxy for identical motor/transmission families, not electrical energy. Neither result establishes a battery-consumption percentage or runtime.

With the original controller unchanged, compact geometry alone reduces six-second positive work per metre from 53.16 to 47.33 J/m (11.0%). The selected compact geometry plus feedback gives 43.59 J/m in that window. These are comparative results for an early shuffle, not efficient walking benchmarks.

The original falls under a 12 N lateral push applied at the pelvis COM for 160 ms from t=2 s. V2 does not fall during that six-second test and reaches approximately 19.9° maximum tilt. Both designs fall under the same forward push. Stepping peak torque increases from 0.763 to 0.822 Nm, so the results do not justify downsizing motors.

At a finer 1 ms timestep, the matched-distance work benefit remains 13.5%, and V2 survives the lateral push with 18.1° maximum tilt. V2 remains upright over a 20-second stepping test and with a 350 g phone in a six-second test.

Both controllers still shuffle and slide. The summed contact-point slip path across both feet over six seconds is approximately 0.36 m for the original and 0.39 m for V2. That metric is not robot travel distance. A lift-and-place gait, robust foot contact handling and recovery steps remain necessary.

## Power-saving behaviours to develop

- Turn the phone first; avoid unnecessary body turning and walking.
- Balance the phone cradle near its tilt axis.
- Develop a mechanically supported parking mode before reducing leg holding current. No brake/latch/support is simulated here.
- Reduce idle perception/radio activity while retaining independent local balance and fault monitoring.
- Measure whole-device input power on a prototype across standing, tracking, turning and walking. Select motor constants, gearboxes and driver efficiencies before estimating battery Wh or runtime.

## Reproduction

Requires Python, MuJoCo, NumPy, Pillow and imageio-ffmpeg. `improve_design.py` imports `simulate_robot.py` from the same folder. Its default invocation reruns a geometry/controller sweep. To reproduce the selected comparisons:

```python
from improve_design import experiment
original = experiment(design='baseline', duration=8)
revised = experiment(design='compact', feedback=.5, duration=8)
print(original['first_10cm'], revised['first_10cm'])
for scenario in ['stand', 'step', 'push', 'push_side']:
    experiment(design='baseline', scenario=scenario, record=True)
    experiment(design='compact', feedback=.5, scenario=scenario, record=True)
```

Run `python render_revised_design.py` to regenerate side-by-side movement and lateral-push videos from the recorded states. The HTML comparison is a self-contained snapshot and does not automatically update when a test is rerun.

## Model limits

Masses, inertias, motor limits, grip and dimensions remain assumptions. Contact masks disable self-collision and permit robot–floor contact. Collision proxies, physical wiring, phone retention, structure stiffness, gear backlash, torque–speed limits, sensor noise, delays and electrical/thermal dynamics require further modelling and hardware validation. The V2 battery proxy is roughly equal in volume to the first model, but real cell arrangement, charging and enclosure clearances are not designed.

Fall detection: body tilt >45° or pelvis height below half the initial pelvis height, accommodating the shorter V2 legs. The push-test falls occur after crossing the tilt threshold, so the comparison is not explained by the change in pelvis-height threshold.

References: [MuJoCo dynamics](https://mujoco.readthedocs.io/en/stable/computation/), [Maxon motor data and losses](https://support.maxongroup.com/hc/en-us/articles/360013761160-Motor-data-and-simulation), [MIT walking energetics](https://underactuated.mit.edu/simple_legs.html).
