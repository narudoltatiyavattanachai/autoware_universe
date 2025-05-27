# Common Messages

This document lists all message definitions used in the common category.

## Message Types

- [IsFilterActivated](#isfilteractivated)
- [OperationModeTransitionManagerDebug](#operationmodetransitionmanagerdebug)
- [Particle](#particle)
- [ParticleArray](#particlearray)

## Message Details

### IsFilterActivated

for additional information

**File:** `control/autoware_vehicle_cmd_gate/msg/IsFilterActivated.msg`

**Fields:**

| Type | Name | Description |
| ---- | ---- | ----------- |
| `builtin_interfaces/Time` | `stamp` |  |
| `bool` | `is_activated` |  |
| `bool` | `is_activated_on_steering` |  |
| `bool` | `is_activated_on_steering_rate` |  |
| `bool` | `is_activated_on_speed` |  |
| `bool` | `is_activated_on_acceleration` |  |
| `bool` | `is_activated_on_jerk` |  |

### OperationModeTransitionManagerDebug

states

**File:** `control/autoware_operation_mode_transition_manager/msg/OperationModeTransitionManagerDebug.msg`

**Fields:**

| Type | Name | Description |
| ---- | ---- | ----------- |
| `builtin_interfaces/Time` | `stamp` |  |
| `string` | `status` |  |
| `bool` | `in_autoware_control` |  |
| `bool` | `in_transition` |  |
| `bool` | `is_all_ok` |  |
| `bool` | `engage_allowed_for_stopped_vehicle` |  |
| `bool` | `trajectory_available_ok` |  |
| `bool` | `lateral_deviation_ok` |  |
| `bool` | `yaw_deviation_ok` |  |
| `bool` | `speed_upper_deviation_ok` |  |
| `bool` | `speed_lower_deviation_ok` |  |
| `bool` | `stop_ok` |  |
| `bool` | `large_acceleration_ok` |  |
| `bool` | `large_lateral_acceleration_ok` |  |
| `bool` | `large_lateral_acceleration_diff_ok` |  |
| `float64` | `current_speed` |  |
| `float64` | `target_control_speed` |  |
| `float64` | `target_planning_speed` |  |
| `float64` | `target_control_acceleration` |  |
| `float64` | `lateral_acceleration` |  |
| `float64` | `lateral_acceleration_deviation` |  |
| `float64` | `lateral_deviation` |  |
| `float64` | `yaw_deviation` |  |
| `float64` | `speed_deviation` |  |

### Particle

A representation of particle, composed of weight and pose.

**File:** `localization/yabloc/yabloc_particle_filter/msg/Particle.msg`

**Fields:**

| Type | Name | Description |
| ---- | ---- | ----------- |
| `float32` | `weight` |  |
| `geometry_msgs/Pose` | `pose` |  |

### ParticleArray

An array of particles with a header for global reference.

**File:** `localization/yabloc/yabloc_particle_filter/msg/ParticleArray.msg`

**Fields:**

| Type | Name | Description |
| ---- | ---- | ----------- |
| `std_msgs/Header` | `header` |  |
| `int32` | `id` |  |
| `Particle[]` | `particles` |  |

