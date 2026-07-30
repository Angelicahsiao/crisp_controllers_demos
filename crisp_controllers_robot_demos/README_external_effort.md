# External Joint-Effort Estimation

Gravity-free ("external") joint torque for robots whose `/joint_states` effort
is the **total** actuated torque — e.g. the UR arms, whose effort is estimated
from motor current and therefore includes the torque spent holding the arm
against gravity.

The UR's `/joint_states` **"effort" is motor current (Amps), not torque** — the
driver fills it from RTDE `actual_current`. So the estimator converts current to
torque with a per-joint gain and subtracts the Pinocchio gravity:

```
tau_ext[Nm] = effort_gain * I  -  g(q)  -  offset
```

- `I` — measured current from `/joint_states` effort (Amps).
- `effort_gain` (`k`, Nm/A) — per-joint current→torque constant (torque constant
  × gear ratio, ~10–12 for the UR7e big joints). The unit conversion lives here,
  on the measurement.
- `g(q)` — gravity torque from the robot model (built from the live
  `/robot_description`), coefficient **1** — the RNEA gravity is physically
  correct and is **not** rescaled. Only as accurate as the URDF masses (see the
  payload note).
- `offset` (Nm) — optional per-joint constant (current bias / static friction).

> **Calibration is required, not optional.** With `effort_gain = 1` you would
> subtract an Nm gravity from an Amp current — meaningless (the gravity-bearing
> joints read the full uncompensated gravity). Run `calibrate_external_effort` to
> identify `effort_gain` and `offset`.

> **Payload accuracy matters.** `g(q)` is only right if the URDF link masses are
> right. The `robotiq_description` models only ~0.36 kg of the 2F-140, while the
> real gripper + coupling measured ~1.3 kg — the missing mass biases `g(q)`
> *pose-dependently*. Measure the true payload with `identify_payload` (wrist
> F/T) and put it in the URDF (done for `ur_single_robotiq.urdf.xacro`).

With the arm at rest and nothing touching it, `tau_ext` should be near zero in
any pose; pushing on a link deflects the joints upstream of the contact.

## Components

| File | Role |
|---|---|
| `crisp_controllers_robot_demos/external_effort.py` | `ExternalEffortEstimator`: Pinocchio model, gravity term, calibration fit. |
| `crisp_controllers_robot_demos/external_effort_node.py` | ROS 2 node: subscribes `/robot_description` + `/joint_states`, publishes `tau_ext`. |
| `crisp_controllers_robot_demos/calibrate_external_effort.py` | Records contact-free samples, fits per-joint `effort_gain` (current→torque) + `offset`, writes a calibration YAML. |
| `crisp_controllers_robot_demos/identify_payload.py` | Measures the tool payload mass + COM from the wrist F/T sensor (feeds the URDF payload so `g(q)` is accurate). |
| `launch/external_effort.launch.py` | Launches the node, optionally with a calibration file. |

Pinocchio stays on the robot side: consumers (e.g. `crisp_gym`) read the
published topic as a plain `std_msgs/Float32MultiArray` sensor and need no
dynamics dependency.

## Quick start (UR7e defaults)

```bash
# Auto-loads config/ur/external_effort_calibration.yaml if present.
# (Calibrate first — uncalibrated output is meaningless: effort_gain=1, Amps≠Nm.)
ros2 launch crisp_controllers_robot_demos external_effort.launch.py

# Add a live per-joint plot (rqt_plot, one trace per joint):
ros2 launch crisp_controllers_robot_demos external_effort.launch.py visualize:=true
```

Publishes `external_joint_effort` (`Float32MultiArray`, one value per joint in
`joint_names` order, in Nm).

Requires the robot bring-up to be running and publishing:

- `/robot_description` (latched) — the URDF the model is built from,
- `/joint_states` **with the effort field filled** (the UR driver does this).

## Calibration (required)

Calibration identifies the per-joint current→torque gain `effort_gain` and the
constant `offset` from contact-free motion. Because `g(q)` loads each joint,
identifying its gain needs gravity to **vary** across the samples.

**1. Record and fit.** Recording stops when you press **ENTER** (or after
`duration` seconds as a safety cap). `joint_names` is **required**. Example for
a UR arm:

```bash
# Writes to config/ur/external_effort_calibration.yaml by default (auto-loaded
# by the launch). min_span:=3 keeps only the well-loaded joints (shoulder_lift,
# elbow) identified and falls the lightly loaded wrists back to the nominal gain.
ros2 run crisp_controllers_robot_demos calibrate_external_effort \
  --ros-args \
  -p joint_names:="[shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]" \
  -p min_span:=3.0
```

**Move, then PAUSE** at each pose (nothing touching the arm): samples are only
recorded when the arm is *settled* (`vel_threshold`, default 0.02 rad/s) — mid-
motion current is acceleration + kinetic friction, not gravity, and corrupts the
gain. Dwell a second or two at each pose, driving each joint through a wide range
of its gravity torque (the wider the span, the better the gain, especially for
`elbow`):

| Joint | Motion needed for a good fit |
|---|---|
| `shoulder_lift` | raise/lower the whole arm: horizontal → up → down |
| `elbow` | fully fold and fully extend the elbow |
| `wrist_1` | pitch the wrist up and down |
| `wrist_2` | **roll** the wrist so its axis tilts between vertical and horizontal |
| `shoulder_pan`, `wrist_3` | rotate about near-vertical axes — gravity barely loads them in **any** pose, so their gain is **unidentifiable**; the script uses the mean of the identified gains and fits only their offset. Expected, not an error. |

While recording, the node prints a live **gravity span** per joint
(`shoulder_lift:4.2OK  elbow:0.3..`). Keep moving a joint until its span is
several Nm (`OK`); a span below `min_span` (default 1 Nm) means that joint's gain
falls back to the nominal.

The final report prints per-joint `gain` (Nm/A, ~10–12 for the big joints),
`offset`, `gravity_span` and `residual_rms` (roughly the noise floor — a large
value means that joint was poorly excited or friction-dominated).

The default `output_file` is `config/ur/external_effort_calibration.yaml`, which
the launch **auto-loads** — so calibrating with the default and then launching
needs no `calibration_file:` argument.

**2. Launch the node (auto-loads the calibration):**

```bash
ros2 launch crisp_controllers_robot_demos external_effort.launch.py
# or, with a live plot / an explicit file:
ros2 launch crisp_controllers_robot_demos external_effort.launch.py \
  visualize:=true \
  calibration_file:=/path/to/external_effort_calibration.yaml
```

The node checks that the calibration was fitted for the same joints and then
uses its `effort_gain`/`offset` instead of the defaults. With no calibration it
warns and runs uncalibrated (`effort_gain = 1`, meaningless).

**Re-calibrate whenever the end-effector mass changes** (different gripper,
added camera, tool payload).

## Parameters

### `external_effort_node`

| Parameter | Default | Description |
|---|---|---|
| `joint_names` | UR joint list (launch) | Actuated arm joints, in output order. **Required.** |
| `joint_state_topic` | `joint_states` | Source topic (must carry effort). |
| `output_topic` | `external_joint_effort` | Published `Float32MultiArray`. |
| `calibration_file` | `""` | YAML from the calibration script; empty = `effort_gain 1, offset 0` (meaningless — calibrate). |
| `effort_gain` / `offset` | `1.0` / `0.0` per joint | Manual per-joint current→torque gain (Nm/A) and offset (Nm); overridden by `calibration_file`. |

A node namespace (e.g. `right`) is prepended to joint names (`right_...`) when
matching `/joint_states`, mirroring crisp_py; if the URDF already bakes the
prefix in, the un-prefixed names are used as a fallback.

### `calibrate_external_effort`

| Parameter | Default | Description |
|---|---|---|
| `joint_names` | — | Same as the node. **Required** (has a UR default in the node only). |
| `stop_on_key` | `True` | Stop recording when ENTER is pressed. |
| `duration` | `120.0` | Max recording seconds / safety cap when `stop_on_key`. |
| `sample_rate` | `5.0` | Sampling rate in Hz. |
| `min_span` | `1.0` | Gravity span (Nm) below which a joint's gain is unidentifiable, so it uses the nominal gain and only its offset is fit. |
| `output_file` | `external_effort_calibration.yaml` | Where to write the fit. |
| `joint_state_topic` | `joint_states` | Source topic. |

Calibration YAML format:

```yaml
joint_names:  [shoulder_pan_joint, ...]
effort_gain:  [12.0, ...]    # per-joint current->torque gain k  [Nm/A]
offset:       [-0.31, ...]   # per-joint constant bias  [Nm]
gravity_span: [0.0, ...]     # how much gravity torque varied while recording [Nm]
residual_rms: [0.7, ...]     # fit quality per joint [Nm]
n_samples: 240
```

## Limitations

- **Quasi-static.** Only gravity is subtracted — inertial/Coriolis torques are
  not — so readings during fast motion overestimate contact. Intended for
  teleop-speed motion and contact detection.
- **Friction is not modeled.** The `offset` absorbs its static average, but
  direction-dependent friction remains in the signal (the main residual after
  calibration).
- **Current-derived source.** The UR has no joint torque sensors; treat the
  output as an estimate, not calibrated Nm.
- **Model completeness.** Anything with mass that is not in `/robot_description`
  (a gripper whose URDF mass is too low, a camera, a tool) biases the estimate
  *pose-dependently* — a constant `offset` cannot absorb it. Measure it with
  `identify_payload` and add it to the URDF (see the payload note at the top).

## Consuming from crisp_py / crisp_gym

The topic is a flat `float32[]`, so crisp_py's registered `float32_array`
sensor reads it directly — no Pinocchio needed on the consumer side. Example
sensor YAML (fields from `crisp_py.sensors.SensorConfig`):

```yaml
name: external_effort
sensor_type: float32_array
data_topic: /external_joint_effort
shape: [6]
```
