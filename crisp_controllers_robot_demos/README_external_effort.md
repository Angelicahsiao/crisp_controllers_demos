# External Joint-Effort Estimation

Gravity-free ("external") joint torque for robots whose `/joint_states` effort
is the **total** actuated torque — e.g. the UR arms, whose effort is estimated
from motor current and therefore includes the torque spent holding the arm
against gravity.

The estimator subtracts a Pinocchio gravity model from the measured effort:

```
tau_ext = tau_measured - (scale * g(q) + offset)
```

- `g(q)` — gravity torque from the robot model (built from the live
  `/robot_description`, so gripper link masses are included automatically).
- `scale`, `offset` — optional per-joint calibration fitted from contact-free
  motion. They absorb the current-to-torque scale error and static offsets
  that otherwise show up as a pose-dependent residual.

With the arm at rest and nothing touching it, `tau_ext` should be near zero in
any pose; pushing on a link deflects the joints upstream of the contact.

## Components

| File | Role |
|---|---|
| `crisp_controllers_robot_demos/external_effort.py` | `ExternalEffortEstimator`: Pinocchio model, gravity term, calibration fit. |
| `crisp_controllers_robot_demos/external_effort_node.py` | ROS 2 node: subscribes `/robot_description` + `/joint_states`, publishes `tau_ext`. |
| `crisp_controllers_robot_demos/calibrate_external_effort.py` | Records contact-free samples, fits `scale`/`offset`, writes a calibration YAML. |
| `launch/external_effort.launch.py` | Launches the node, optionally with a calibration file. |

Pinocchio stays on the robot side: consumers (e.g. `crisp_gym`) read the
published topic as a plain `std_msgs/Float32MultiArray` sensor and need no
dynamics dependency.

## Quick start (UR7e defaults)

```bash
# uncalibrated (pure gravity subtraction)
ros2 launch crisp_controllers_robot_demos external_effort.launch.py
```

Publishes `external_joint_effort` (`Float32MultiArray`, one value per joint in
`joint_names` order, in Nm).

Requires the robot bring-up to be running and publishing:

- `/robot_description` (latched) — the URDF the model is built from,
- `/joint_states` **with the effort field filled** (the UR driver does this).

## Calibration (recommended)

Pure gravity subtraction leaves a pose-dependent residual of a few Nm because
the UR's current-derived effort has per-joint scale errors. A one-time
calibration removes most of it.

**1. Record and fit.** Recording stops when you press **ENTER** (or after
`duration` seconds as a safety cap). `joint_names` is **required**. Example for
a UR arm:

```bash
ros2 run crisp_controllers_robot_demos calibrate_external_effort \
  --ros-args \
  -p joint_names:="[shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]" \
  -p output_file:=/home/ros/ros2_ws/src/crisp_controllers_demos/external_effort_calibration.yaml
```

Move the arm slowly with **nothing touching it**. The fit can only identify a
joint's scale if **gravity loads that joint differently across the poses you
record** — so you must actively drive each joint through motions that change
its gravity torque:

| Joint | Motion needed for a good fit |
|---|---|
| `shoulder_lift` | raise/lower the whole arm: horizontal → up → down |
| `elbow` | fully fold and fully extend the elbow |
| `wrist_1` | pitch the wrist up and down |
| `wrist_2` | **roll** the wrist so its axis tilts between vertical and horizontal |
| `shoulder_pan`, `wrist_3` | rotate about near-vertical axes — gravity barely loads them in **any** pose, so their scale is **physically unidentifiable**; the script keeps `scale=1.0` and fits only their offset. Expected, not an error. |

While recording, the node prints a live **gravity span** per joint
(`shoulder_lift:4.2OK  elbow:0.3..`). Keep moving a joint until its span is
several Nm (`OK`); `..` means it still needs more excitation. A span below
`min_span` (default 1 Nm) at the end means that joint's scale is left at 1.0.

The final report prints per-joint `scale`, `offset`, `gravity_span` and
`residual_rms` (roughly the noise floor of your estimate — typically 0.5–1.5 Nm
after a good calibration; a much larger value means that joint was poorly
excited or friction-dominated).

**2. Launch the node with the calibration:**

```bash
ros2 launch crisp_controllers_robot_demos external_effort.launch.py \
  calibration_file:=/path/to/external_effort_calibration.yaml
```

The node checks that the calibration was fitted for the same joints and then
uses its `scale`/`offset` instead of the defaults.

**Re-calibrate whenever the end-effector mass changes** (different gripper,
added camera, tool payload).

## Parameters

### `external_effort_node`

| Parameter | Default | Description |
|---|---|---|
| `joint_names` | UR joint list (launch) | Actuated arm joints, in output order. **Required.** |
| `joint_state_topic` | `joint_states` | Source topic (must carry effort). |
| `output_topic` | `external_joint_effort` | Published `Float32MultiArray`. |
| `calibration_file` | `""` | YAML from the calibration script; empty = `scale 1, offset 0`. |
| `scale` / `offset` | `1.0` / `0.0` per joint | Manual calibration; overridden by `calibration_file`. |

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
| `min_span` | `1.0` | Gravity span (Nm) below which a joint's scale is left at 1.0 and only its offset is fit. |
| `output_file` | `external_effort_calibration.yaml` | Where to write the fit. |
| `joint_state_topic` | `joint_states` | Source topic. |

Calibration YAML format:

```yaml
joint_names:  [shoulder_pan_joint, ...]
scale:        [1.043, ...]   # per-joint gain a (1.0 if span < min_span)
offset:       [-0.31, ...]   # per-joint offset b  [Nm]
gravity_span: [0.0, ...]     # how much gravity torque varied while recording [Nm]
residual_rms: [0.7, ...]     # fit quality per joint [Nm]
n_samples: 240
```

## Limitations

- **Quasi-static.** Only gravity is subtracted — inertial/Coriolis torques are
  not — so readings during fast motion overestimate contact. Intended for
  teleop-speed motion and contact detection.
- **Friction is not modeled.** The scale/offset fit absorbs its static
  average, but direction-dependent friction remains in the signal (the main
  residual after calibration).
- **Current-derived source.** The UR has no joint torque sensors; treat the
  output as an estimate, not calibrated Nm.
- **Model completeness.** Anything with mass that is not in
  `/robot_description` (e.g. a camera) biases the estimate — add it to the
  URDF or rely on calibration to absorb it at fixed mounting.

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
