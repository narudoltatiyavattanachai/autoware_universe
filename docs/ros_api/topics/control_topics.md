# Control Topics

This document lists all topics used in the control category.

| Topic | Message Type | Publishers | Subscribers |
| ----- | ------------ | ---------- | ----------- |
| `/accel_brake_map_calibrator/output/calibration_status` | `CalibrationStatus` | accel_brake_map_calibrator_node.cpp |  |
| `/api/autoware/get/engage` | `AutowareEngage` |  | compatibility.cpp |
| `/api/planning/steering_factors` | `SteeringFactorArray` |  | velocity_steering_factors_panel.cpp |
| `/api/planning/velocity_factors` | `VelocityFactorArray` |  | velocity_steering_factors_panel.cpp |
| `/autoware/engage` | `AutowareEngage` | compatibility.cpp |  |
| `/control/current_gate_mode` | `GateMode` |  | compatibility.cpp |
| `/control/external_cmd_selector/current_selector_mode` | `SelectorModeMsg` |  | compatibility.cpp |
| `/control/gate_mode_cmd` | `GateMode` | compatibility.cpp |  |
| `/localization/acceleration` | `geometry_msgs::msg::AccelWithCovarianceStamped` |  | velocity_steering_factors_panel.cpp |
| `/localization/kinematic_state` | `Odometry` | test_filter_in_vehicle_cmd_gate_node.cpp | vehicle_cmd_gate.cpp, velocity_steering_factors_panel.cpp |
| `/vehicle/raw_vehicle_cmd_converter/debug/compensated_control_cmd` | `Control` | node.cpp |  |
| `/vehicle/raw_vehicle_cmd_converter/debug/steer_pid` | `Float32MultiArrayStamped` | node.cpp |  |
| `autoware/state` | `AutowareState` | test.cpp |  |
| `controller/input/current_accel` | `AccelWithCovarianceStamped` | test_controller_node.cpp |  |
| `controller/input/current_odometry` | `VehicleOdometry` | test_controller_node.cpp |  |
| `controller/input/current_operation_mode` | `OperationModeState` | test_controller_node.cpp |  |
| `controller/input/current_steering` | `SteeringReport` | test_controller_node.cpp |  |
| `controller/input/reference_trajectory` | `Trajectory` | test_controller_node.cpp |  |
| `controller/output/control_cmd` | `Control` |  | test_controller_node.cpp |
| `input/acceleration` | `AccelWithCovarianceStamped` | test_filter_in_vehicle_cmd_gate_node.cpp | vehicle_cmd_gate.cpp |
| `input/auto/control_cmd` | `Control` | test_filter_in_vehicle_cmd_gate_node.cpp | vehicle_cmd_gate.cpp |
| `input/auto/gear_cmd` | `GearCommand` | test_filter_in_vehicle_cmd_gate_node.cpp |  |
| `input/auto/hazard_lights_cmd` | `HazardLightsCommand` | test_filter_in_vehicle_cmd_gate_node.cpp |  |
| `input/auto/turn_indicators_cmd` | `TurnIndicatorsCommand` | test_filter_in_vehicle_cmd_gate_node.cpp |  |
| `input/emergency/control_cmd` | `Control` |  | vehicle_cmd_gate.cpp |
| `input/external/control_cmd` | `Control` |  | vehicle_cmd_gate.cpp |
| `input/external_emergency_stop_heartbeat` | `Heartbeat` | test_filter_in_vehicle_cmd_gate_node.cpp | vehicle_cmd_gate.cpp |
| `input/gate_mode` | `GateMode` | test_filter_in_vehicle_cmd_gate_node.cpp | vehicle_cmd_gate.cpp |
| `input/mrm_state` | `MrmState` | test_filter_in_vehicle_cmd_gate_node.cpp | vehicle_cmd_gate.cpp |
| `input/obstacle_pointcloud` | `sensor_msgs::msg::PointCloud2` |  | obstacle_collision_checker_node.cpp |
| `input/odometry` | `nav_msgs::msg::Odometry` |  | obstacle_collision_checker_node.cpp |
| `input/operation_mode` | `OperationModeState` | test_filter_in_vehicle_cmd_gate_node.cpp | vehicle_cmd_gate.cpp |
| `input/predicted_trajectory` | `autoware_planning_msgs::msg::Trajectory` |  | obstacle_collision_checker_node.cpp |
| `input/reference_trajectory` | `autoware_planning_msgs::msg::Trajectory` |  | obstacle_collision_checker_node.cpp |
| `input/steering` | `SteeringReport` | test_filter_in_vehicle_cmd_gate_node.cpp | vehicle_cmd_gate.cpp |
| `output/control_cmd` | `Control` | simple_trajectory_follower.cpp, vehicle_cmd_gate.cpp | test_filter_in_vehicle_cmd_gate_node.cpp |
| `output/control_command` | `autoware_control_msgs::msg::Control` | joy_controller_node.cpp |  |
| `output/engage` | `EngageMsg` | vehicle_cmd_gate.cpp |  |
| `output/external_control_command` | `tier4_external_api_msgs::msg::ControlCommandStamped` | joy_controller_node.cpp |  |
| `output/external_emergency` | `Emergency` | vehicle_cmd_gate.cpp |  |
| `output/gate_mode` | `tier4_control_msgs::msg::GateMode` | joy_controller_node.cpp, vehicle_cmd_gate.cpp |  |
| `output/gear_cmd` | `autoware_vehicle_msgs::msg::GearCommand` | autoware_shift_decider.cpp, vehicle_cmd_gate.cpp |  |
| `output/hazard_lights_cmd` | `HazardLightsCommand` | vehicle_cmd_gate.cpp |  |
| `output/heartbeat` | `tier4_external_api_msgs::msg::Heartbeat` | joy_controller_node.cpp |  |
| `output/operation_mode` | `OperationModeState` | vehicle_cmd_gate.cpp |  |
| `output/shift` | `tier4_external_api_msgs::msg::GearShiftStamped` | joy_controller_node.cpp |  |
| `output/turn_indicators_cmd` | `TurnIndicatorsCommand` | vehicle_cmd_gate.cpp |  |
| `output/turn_signal` | `tier4_external_api_msgs::msg::TurnSignalStamped` | joy_controller_node.cpp |  |
| `output/vehicle_cmd_emergency` | `VehicleEmergencyStamped` | vehicle_cmd_gate.cpp |  |
| `output/vehicle_engage` | `autoware_vehicle_msgs::msg::Engage` | joy_controller_node.cpp |  |
| `~/debug/control_cmd_horizon` | `autoware_control_msgs::msg::ControlHorizon` | controller_node.cpp |  |
| `~/debug/data_average_occ_map` | `OccupancyGrid` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/data_count_occ_map` | `OccupancyGrid` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/data_count_self_pose_occ_map` | `OccupancyGrid` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/data_std_dev_occ_map` | `OccupancyGrid` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/ld_outputs` | `autoware_internal_debug_msgs::msg::Float32MultiArrayStamped` | autoware_pure_pursuit_lateral_controller.cpp |  |
| `~/debug/occ_index` | `MarkerArray` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/offset_covariance` | `Float32MultiArray` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/original_occ_map` | `OccupancyGrid` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/original_raw_map` | `Float32MultiArray` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/predicted_trajectory_in_frenet_coordinate` | `Trajectory` | mpc.cpp |  |
| `~/debug/resampled_reference_trajectory` | `Trajectory` | mpc.cpp |  |
| `~/debug/rss_distance` | `tier4_debug_msgs::msg::Float32Stamped` | node.cpp |  |
| `~/debug/update_occ_map` | `OccupancyGrid` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/virtual_wall` | `visualization_msgs::msg::MarkerArray` | debug_marker.cpp |  |
| `~/debug_info` | `ModeChangeBase::DebugInfo` | node.cpp |  |
| `~/debug_markers` | `visualization_msgs::msg::MarkerArray` | node.hpp |  |
| `~/input/actuation_status` | `ActuationStatusStamped` |  | node.cpp |
| `~/input/control_cmd` | `Control` |  | node.cpp, control_validator.cpp |
| `~/input/control_raw` | `Control` |  | control_performance_analysis_node.cpp |
| `~/input/current_accel` | `geometry_msgs::msg::AccelWithCovarianceStamped` |  | predicted_path_checker_node.cpp |
| `~/input/imu` | `Imu` | test.cpp |  |
| `~/input/local/gear_cmd` | `GearCommand` |  | external_cmd_selector_node.cpp |
| `~/input/local/hazard_lights_cmd` | `HazardLightsCommand` |  | external_cmd_selector_node.cpp |
| `~/input/local/heartbeat` | `OperatorHeartbeat` |  | external_cmd_selector_node.cpp |
| `~/input/local/pedals_cmd` | `PedalsCommand` |  | external_cmd_selector_node.cpp |
| `~/input/local/steering_cmd` | `SteeringCommand` |  | external_cmd_selector_node.cpp |
| `~/input/local/turn_indicators_cmd` | `TurnIndicatorsCommand` |  | external_cmd_selector_node.cpp |
| `~/input/measured_steering` | `SteeringReport` |  | control_performance_analysis_node.cpp |
| `~/input/predicted_trajectory` | `Trajectory` | test.cpp | predicted_path_checker_node.cpp |
| `~/input/reference_trajectory` | `Trajectory` |  | control_performance_analysis_node.cpp, predicted_path_checker_node.cpp |
| `~/input/remote/gear_cmd` | `GearCommand` |  | external_cmd_selector_node.cpp |
| `~/input/remote/hazard_lights_cmd` | `HazardLightsCommand` |  | external_cmd_selector_node.cpp |
| `~/input/remote/heartbeat` | `OperatorHeartbeat` |  | external_cmd_selector_node.cpp |
| `~/input/remote/pedals_cmd` | `PedalsCommand` |  | external_cmd_selector_node.cpp |
| `~/input/remote/steering_cmd` | `SteeringCommand` |  | external_cmd_selector_node.cpp |
| `~/input/remote/turn_indicators_cmd` | `TurnIndicatorsCommand` |  | external_cmd_selector_node.cpp |
| `~/input/steer` | `Steering` |  | steer_offset_estimator_node.cpp |
| `~/input/steering` | `Steering` |  | node.cpp |
| `~/input/velocity` | `VelocityReport` | test.cpp |  |
| `~/is_filter_activated` | `IsFilterActivated` | vehicle_cmd_gate.cpp |  |
| `~/is_filter_activated/flag` | `BoolStamped` | vehicle_cmd_gate.cpp |  |
| `~/is_filter_activated/marker` | `MarkerArray` | vehicle_cmd_gate.cpp |  |
| `~/is_filter_activated/marker_raw` | `MarkerArray` | vehicle_cmd_gate.cpp |  |
| `~/lateral/debug/processing_time_ms` | `Float64Stamped` | controller_node.cpp |  |
| `~/longitudinal/debug/processing_time_ms` | `Float64Stamped` | controller_node.cpp |  |
| `~/output/actuation_cmd` | `ActuationCommandStamped` | node.cpp |  |
| `~/output/control_cmd` | `autoware_control_msgs::msg::Control` | controller_node.cpp |  |
| `~/output/current_map_error` | `Float32Stamped` | accel_brake_map_calibrator_node.cpp |  |
| `~/output/current_selector_mode` | `CommandSourceMode` | external_cmd_selector_node.cpp |  |
| `~/output/debug_marker` | `visualization_msgs::msg::MarkerArray` | controller_node.cpp |  |
| `~/output/debug_values` | `Float32MultiArrayStamped` | accel_brake_map_calibrator_node.cpp |  |
| `~/output/driving_status_stamped` | `DrivingMonitorStamped` | control_performance_analysis_node.cpp |  |
| `~/output/error_stamped` | `ErrorStamped` | control_performance_analysis_node.cpp |  |
| `~/output/estimated_steer_offset` | `Float32Stamped` | mpc_lateral_controller.cpp |  |
| `~/output/gear_cmd` | `GearCommand` | external_cmd_selector_node.cpp |  |
| `~/output/heartbeat` | `OperatorHeartbeat` | external_cmd_selector_node.cpp |  |
| `~/output/lateral_diagnostic` | `Float32MultiArrayStamped` | mpc_lateral_controller.cpp |  |
| `~/output/longitudinal_diagnostic` | `autoware_internal_debug_msgs::msg::Float32MultiArrayStamped` | pid_longitudinal_controller.cpp |  |
| `~/output/map_error_ratio` | `Float32Stamped` | accel_brake_map_calibrator_node.cpp |  |
| `~/output/pedals_cmd` | `PedalsCommand` | external_cmd_selector_node.cpp |  |
| `~/output/predicted_trajectory` | `autoware_planning_msgs::msg::Trajectory` | autoware_pure_pursuit_lateral_controller.cpp, mpc_lateral_controller.cpp |  |
| `~/output/slope_angle` | `autoware_internal_debug_msgs::msg::Float32MultiArrayStamped` | pid_longitudinal_controller.cpp |  |
| `~/output/steering_cmd` | `SteeringCommand` | external_cmd_selector_node.cpp |  |
| `~/output/steering_offset` | `Float32Stamped` | steer_offset_estimator_node.cpp |  |
| `~/output/steering_offset_covariance` | `Float32Stamped` | steer_offset_estimator_node.cpp |  |
| `~/output/steering_status` | `Steering` | node.cpp |  |
| `~/output/update_raw_map` | `Float32MultiArray` | accel_brake_map_calibrator_node.cpp |  |
| `~/output/update_suggest` | `std_msgs::msg::Bool` | accel_brake_map_calibrator_node.cpp |  |
| `~/output/updated_map_error` | `Float32Stamped` | accel_brake_map_calibrator_node.cpp |  |
