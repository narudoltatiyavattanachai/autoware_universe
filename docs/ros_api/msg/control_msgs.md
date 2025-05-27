# Control Messages

This document lists all message definitions used in the control category.

## Message Types

- [ControlValidatorStatus](#controlvalidatorstatus)
- [DrivingMonitorStamped](#drivingmonitorstamped)
- [Error](#error)
- [ErrorStamped](#errorstamped)
- [FloatStamped](#floatstamped)

## Message Details

### ControlValidatorStatus

states

**File:** `control/autoware_control_validator/msg/ControlValidatorStatus.msg`

**Fields:**

| Type | Name | Description |
| ---- | ---- | ----------- |
| `builtin_interfaces/Time` | `stamp` |  |
| `bool` | `is_valid_max_distance_deviation` |  |
| `bool` | `is_valid_acc` |  |
| `bool` | `is_rolling_back` |  |
| `bool` | `is_over_velocity` |  |
| `bool` | `is_valid_lateral_jerk` |  |
| `bool` | `has_overrun_stop_point` |  |
| `bool` | `will_overrun_stop_point` |  |
| `bool` | `is_valid_latency` |  |
| `float64` | `max_distance_deviation` |  |
| `float64` | `steering_rate` |  |
| `float64` | `lateral_jerk` |  |
| `float64` | `desired_acc` |  |
| `float64` | `measured_acc` |  |
| `float64` | `target_vel` |  |
| `float64` | `vehicle_vel` |  |
| `float64` | `dist_to_stop` |  |
| `float64` | `pred_dist_to_stop` |  |
| `float64` | `nearest_trajectory_vel` |  |
| `float64` | `latency` |  |
| `int64` | `invalid_count` |  |

### DrivingMonitorStamped

**File:** `control/autoware_control_performance_analysis/msg/DrivingMonitorStamped.msg`

**Fields:**

| Type | Name | Description |
| ---- | ---- | ----------- |
| `autoware_control_performance_analysis/FloatStamped` | `longitudinal_acceleration` |  |
| `autoware_control_performance_analysis/FloatStamped` | `longitudinal_jerk` |  |
| `autoware_control_performance_analysis/FloatStamped` | `lateral_acceleration` |  |
| `autoware_control_performance_analysis/FloatStamped` | `lateral_jerk` |  |
| `autoware_control_performance_analysis/FloatStamped` | `desired_steering_angle` |  |
| `autoware_control_performance_analysis/FloatStamped` | `controller_processing_time` |  |

### Error

**File:** `control/autoware_control_performance_analysis/msg/Error.msg`

**Fields:**

| Type | Name | Description |
| ---- | ---- | ----------- |
| `float64` | `lateral_error` |  |
| `float64` | `lateral_error_velocity` |  |
| `float64` | `lateral_error_acceleration` |  |
| `float64` | `longitudinal_error` |  |
| `float64` | `longitudinal_error_velocity` |  |
| `float64` | `longitudinal_error_acceleration` |  |
| `float64` | `heading_error` |  |
| `float64` | `heading_error_velocity` |  |
| `float64` | `control_effort_energy` |  |
| `float64` | `error_energy` |  |
| `float64` | `value_approximation` |  |
| `float64` | `curvature_estimate` |  |
| `float64` | `curvature_estimate_pp` |  |
| `float64` | `vehicle_velocity_error` |  |
| `float64` | `tracking_curvature_discontinuity_ability` |  |

### ErrorStamped

**File:** `control/autoware_control_performance_analysis/msg/ErrorStamped.msg`

**Fields:**

| Type | Name | Description |
| ---- | ---- | ----------- |
| `std_msgs/Header` | `header` |  |
| `autoware_control_performance_analysis/Error` | `error` |  |

### FloatStamped

**File:** `control/autoware_control_performance_analysis/msg/FloatStamped.msg`

**Fields:**

| Type | Name | Description |
| ---- | ---- | ----------- |
| `std_msgs/Header` | `header` |  |
| `float64` | `data` |  |

