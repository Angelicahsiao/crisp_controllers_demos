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

**1. Record and fit** (~30 s). Move the arm slowly through diverse poses —
freedrive or slow teleop — with **nothing touching it**. Exercise
`shoulder_lift` and `elbow` through high and low poses especially: the fit
needs the gravity torque to vary.

```bash
ros2 run crisp_controllers_robot_demos calibrate_external_effort \
  --ros-args -p output_file:=/path/to/external_effort_calibration.yaml
```

The script prints per-joint `scale`, `offset` and `residual_rms` (roughly the
noise floor of your estimate, typically 0.5–1.5 Nm after calibration) and
warns if a joint's gravity torque barely changed during recording — move that
joint through more poses and re-run. Joints that hardly fight gravity
(`shoulder_pan`, `wrist_3` in many poses) will always trigger this warning;
that is expected and harmless since their gravity torque is near zero.

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
| `duration` | `30.0` | Recording time in seconds. |
| `sample_rate` | `20.0` | Sampling rate in Hz. |
| `output_file` | `external_effort_calibration.yaml` | Where to write the fit. |
| `joint_state_topic` | `joint_states` | Source topic. |

Calibration YAML format:

```yaml
joint_names: [shoulder_pan_joint, ...]
scale:       [1.043, ...]   # per-joint gain a
offset:      [-0.31, ...]   # per-joint offset b  [Nm]
residual_rms: [0.7, ...]    # fit quality per joint [Nm]
n_samples: 600
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
