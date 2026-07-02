# User Guide

> **⚠️ Migration notice:** The `offset` parameter is deprecated and ignored. The driver now always centers at 2048 (midpoint of the 0–4095 tick range). Use `homing_offset` instead — it writes directly to the servo's EEPROM so the centering happens in hardware. To migrate: `homing_offset = old_offset - 2048`. The driver will log a warning if it detects the old `offset` parameter.

## ros2_control urdf tag

The feetech system interface has a few `ros2_control` urdf tags to customize its behavior.

#### Hardware Parameters

* `usb_port` (required). Example: `<param name="usb_port">/dev/ttyUSB0</param>`.
* `joint_config_file` (optional): Path to a YAML file with per-joint parameters. If omitted, only URDF params are used (backward-compatible). See [YAML Joint Configuration](#yaml-joint-configuration-file) below.

#### Per-joint Parameters

Make sure to look at [Memory table](https://docs.google.com/spreadsheets/d/1GVs7W1VS1PqdhA1nW-abeyAHhTUxKUdR/edit?gid=364516031#gid=364516031) for a detailed explanation of the parameters.

* `id` (**required**, must be in URDF): Servo ID on the bus. The driver uses this to address the servo and to match YAML config entries. Example: `<param name="id">1</param>`.
* `p_coefficient` (optional): Proportional coefficient of the PID controller. Example: `<param name="p_coefficient">8</param>`.
* `i_coefficient` (optional): Integral coefficient of the PID controller. Example: `<param name="i_coefficient">0</param>`.
* `d_coefficient` (optional): Derivative coefficient of the PID controller. Example: `<param name="d_coefficient">32</param>`.
* `homing_offset` (optional): Signed offset written to the servo's EEPROM. The servo firmware applies `Present_Position = Actual_Position - Homing_Offset`, so setting `homing_offset = actual_position - 2048` makes the servo report 2048 (center) at your desired physical center. If migrating from the old `offset` parameter: `homing_offset = old_offset - 2048` (since the old offset was effectively the actual position at center).
* `range_min` (optional): Minimum angle limit (raw ticks, after homing offset is applied).
* `range_max` (optional): Maximum angle limit (raw ticks, after homing offset is applied).
* `max_torque_limit` (optional): Maximum torque limit, written to the servo's EEPROM (raw units, `0`–`1000` = `0`–`100.0%` of the servo's max torque). Also used as the default runtime torque limit (see [Runtime Limit Command Interfaces](#runtime-limit-command-interfaces) below). If omitted, the driver reads back whatever value is already configured on the servo; if that read also fails, it falls back to `1000` (full torque).
* `protection_current` (optional): Protection current threshold.
* `overload_torque` (optional): Overload torque threshold.
* `return_delay_time` (optional): Response delay time.
* `acceleration` (optional): Acceleration value.
* `use_velocity_limit_interface` (optional, default `false`): Export a `set_max_velocity` command interface for this joint (rad/s). See [Runtime Limit Command Interfaces](#runtime-limit-command-interfaces).
* `use_torque_limit_interface` (optional, default `false`): Export a `set_max_torque` command interface for this joint (Nm, see `torque_scale` below). See [Runtime Limit Command Interfaces](#runtime-limit-command-interfaces).
* `torque_scale` (**required** if `use_torque_limit_interface` is `true`): The torque (Nm) that corresponds to a fully saturated (raw `1000`) torque-limit command — i.e. the servo's stall torque, from its datasheet. If you don't know the real value, set `torque_scale: 100` and command `set_max_torque` directly in percent (`0`–`100`) instead of Nm.

> **Note:** `use_velocity_limit_interface` / `use_torque_limit_interface` are ignored (with a warning) on joints that have no `command_interface` (e.g. leader/state-only joints) — the limit interfaces only make sense for commanded joints.

### Example

Take a look at [ros2_so_arm100](https://github.com/JafarAbdi/ros2_so_arm100/blob/main/so_arm100_description/control/so_arm100.ros2_control.xacro) for an example of how to use the URDF tags.

---

## YAML Joint Configuration File

As an alternative (or addition) to URDF `<param>` tags, joint parameters can be loaded from a YAML file. This is useful for calibration values that change between robots (like `homing_offset`) without modifying the URDF.

The `id` parameter **must** be defined in the URDF (it is the hardware identity used to address the servo on the bus). YAML entries are matched to URDF joints by servo `id`, not by joint name — so the YAML joint name is just a human-friendly label. When both URDF params and YAML are provided for the same `id`, YAML values take precedence.

### Format

```yaml
joints:
  joint_name:
    id: 1
    homing_offset: 530
    range_min: 866
    range_max: 3231
    p_coefficient: 16
    i_coefficient: 0
    d_coefficient: 32
    return_delay_time: 0
    acceleration: 254
```

### URDF Integration

Pass the YAML file path as a hardware parameter:

```xml
<param name="joint_config_file">$(find my_robot_bringup)/config/joints.yaml</param>
```

## Runtime Limit Command Interfaces

By default, every commanded joint moves at a fixed velocity (2400 ticks/s ≈ 3.68 rad/s) and the torque limit configured at startup (`max_torque_limit`, or the servo's current EEPROM value if unset). A joint can opt in to override these dynamically, per `write()` cycle, via two additional `ros2_control` command interfaces:

| Interface | Unit | Enabled by |
|---|---|---|
| `<joint>/set_max_velocity` | rad/s | `use_velocity_limit_interface: true` |
| `<joint>/set_max_torque` | Nm (see `torque_scale`) | `use_torque_limit_interface: true` |

Both interfaces are initialized to `NaN`. While unclaimed, or while the commanded value is `NaN`, the joint uses its default limit (from `max_torque_limit` / the built-in velocity constant) — so a controller that never writes these interfaces sees no change in behavior.

### Torque units

The servo's torque-limit register only understands a fraction of its own stall torque (`0`–`1000` raw = `0`–`100.0%`). The parameter `torque_scale` (Nm) tells the driver what "100%" means for your joint, so it can convert the `set_max_torque` command interface value to raw units:

```
raw = clamp(round(set_max_torque / torque_scale * 1000), 0, 1000)
```

* **Know the motor's stall torque?** Set `torque_scale` to that value (from the datasheet, e.g. the STS3215's rated kg·cm converted to Nm), and command `set_max_torque` in real Nm.
* **Don't know it?** Set `torque_scale: 100` and command `set_max_torque` directly in percent (`0`–`100`).

### Example (`parallel_gripper_action_controller`)

The [`parallel_gripper_action_controller`](https://control.ros.org) supports driving arbitrary velocity/effort command interfaces via its own `max_velocity_interface` / `max_effort_interface` params — a natural pairing with the interfaces above:

```yaml
# joint_config.yaml
joints:
  gripper:
    id: 6
    max_torque_limit: 1000
    use_velocity_limit_interface: true
    use_torque_limit_interface: true
    torque_scale: 100  # unknown real rating -> command set_max_torque in percent
```

### Examples

* URDF-only setup: [ros2_so_arm100](https://github.com/JafarAbdi/ros2_so_arm100/blob/main/so_arm100_description/control/so_arm100.ros2_control.xacro)
* YAML config setup: [so101-ros-physical-ai](https://github.com/legalaspro/so101-ros-physical-ai) — see [follower](https://github.com/legalaspro/so101-ros-physical-ai/blob/main/so101_bringup/config/hardware/follower_joints.yaml) and [leader](https://github.com/legalaspro/so101-ros-physical-ai/blob/main/so101_bringup/config/hardware/leader_joints.yaml) arm configs.
