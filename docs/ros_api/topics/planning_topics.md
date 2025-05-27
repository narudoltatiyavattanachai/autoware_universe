# Planning Topics

This document lists all topics used in the planning category.

| Topic | Message Type | Publishers | Subscribers |
| ----- | ------------ | ---------- | ----------- |
| `/planning/scenario_planning/current_max_velocity` | `autoware_internal_planning_msgs::msg::VelocityLimit` |  | remaining_distance_time_calculator_node.cpp |
| `/planning/scenario_planning/trajectory` | `Trajectory` |  | optimization_based_planner.cpp, optimization_based_planner.cpp |
| `/planning_validator/input/acceleration` | `AccelWithCovarianceStamped` | test_planning_validator_pubsub.cpp |  |
| `/planning_validator/input/kinematics` | `Odometry` | test_planning_validator_pubsub.cpp |  |
| `/planning_validator/input/trajectory` | `Trajectory` | test_planning_validator_pubsub.cpp |  |
| `/rviz/routing/pose` | `PoseStamped` |  | route_panel.cpp |
| `/tf` | `tf2_msgs::msg::TFMessage` | simple_planning_simulator_core.cpp |  |
| `behavior_velocity_planner_node/input/virtual_traffic_light_states` | `VirtualTrafficLightStateArray` | test_node_interface.cpp |  |
| `input/ackermann_control_command` | `Control` | test_simple_planning_simulator.cpp | simple_planning_simulator_core.cpp |
| `input/actuation_command` | `ActuationCommandStamped` | test_simple_planning_simulator.cpp | simple_planning_simulator_core.cpp |
| `input/engage` | `Engage` | test_filter_in_vehicle_cmd_gate_node.cpp | simple_planning_simulator_core.cpp, vehicle_cmd_gate.cpp |
| `input/gear_command` | `GearCommand` | test_simple_planning_simulator.cpp | simple_planning_simulator_core.cpp |
| `input/hazard_lights_command` | `HazardLightsCommand` |  | simple_planning_simulator_core.cpp |
| `input/initialpose` | `PoseWithCovarianceStamped` | test_simple_planning_simulator.cpp | simple_planning_simulator_core.cpp |
| `input/initialtwist` | `TwistStamped` |  | simple_planning_simulator_core.cpp |
| `input/lane_driving/trajectory` | `autoware_planning_msgs::msg::Trajectory` |  | node.cpp |
| `input/lanelet_map` | `autoware_map_msgs::msg::LaneletMapBin` |  | node.cpp |
| `input/manual_ackermann_control_command` | `Control` |  | simple_planning_simulator_core.cpp |
| `input/manual_gear_command` | `GearCommand` |  | simple_planning_simulator_core.cpp |
| `input/parking/trajectory` | `autoware_planning_msgs::msg::Trajectory` |  | node.cpp |
| `input/route` | `autoware_planning_msgs::msg::LaneletRoute` |  | node.cpp, goal_pose_visualizer.cpp |
| `input/trajectory` | `Trajectory` |  | simple_planning_simulator_core.cpp |
| `input/turn_indicators_command` | `TurnIndicatorsCommand` |  | simple_planning_simulator_core.cpp |
| `input/velocity_limit_clear_command_from_internal` | `VelocityLimitClearCommand` |  | external_velocity_limit_selector_node.cpp |
| `input/velocity_limit_from_api` | `VelocityLimit` |  | external_velocity_limit_selector_node.cpp |
| `input/velocity_limit_from_internal` | `VelocityLimit` |  | external_velocity_limit_selector_node.cpp |
| `is_completed` | `std_msgs::msg::Bool` | freespace_planner_node.cpp |  |
| `output/acceleration` | `AccelWithCovarianceStamped` | simple_planning_simulator_core.cpp |  |
| `output/actuation_status` | `ActuationStatusStamped` | simple_planning_simulator_core.cpp |  |
| `output/control_mode_report` | `ControlModeReport` | simple_planning_simulator_core.cpp |  |
| `output/debug` | `StringStamped` | external_velocity_limit_selector_node.cpp |  |
| `output/external_velocity_limit` | `VelocityLimit` | external_velocity_limit_selector_node.cpp |  |
| `output/gear_report` | `GearReport` | simple_planning_simulator_core.cpp |  |
| `output/goal_pose` | `geometry_msgs::msg::PoseStamped` | goal_pose_visualizer.cpp |  |
| `output/hazard_lights_report` | `HazardLightsReport` | simple_planning_simulator_core.cpp |  |
| `output/imu` | `Imu` | simple_planning_simulator_core.cpp |  |
| `output/odometry` | `Odometry` | simple_planning_simulator_core.cpp | test_simple_planning_simulator.cpp |
| `output/pose` | `PoseWithCovarianceStamped` | simple_planning_simulator_core.cpp |  |
| `output/scenario` | `autoware_internal_planning_msgs::msg::Scenario` | node.cpp |  |
| `output/steering` | `SteeringReport` | simple_planning_simulator_core.cpp |  |
| `output/trajectory` | `autoware_planning_msgs::msg::Trajectory` | node.cpp |  |
| `output/turn_indicators_report` | `TurnIndicatorsReport` | simple_planning_simulator_core.cpp |  |
| `output/twist` | `VelocityReport` | simple_planning_simulator_core.cpp |  |
| `~/` | `visualization_msgs::msg::MarkerArray` | run_out_module.cpp, dynamic_obstacle_stop_module.cpp and 2 more |  |
| `~/adaptive_cruise_control/debug_values` | `autoware_internal_debug_msgs::msg::Float32MultiArrayStamped` | adaptive_cruise_control.cpp |  |
| `~/boundary` | `Trajectory` | optimization_based_planner.cpp, optimization_based_planner.cpp |  |
| `~/debug/` | `autoware_internal_debug_msgs::msg::Float64Stamped` | run_out_module.cpp, dynamic_obstacle_stop_module.cpp and 3 more |  |
| `~/debug/avoidance_debug_message_array` | `AvoidanceDebugMsgArray` | behavior_path_planner_node.cpp |  |
| `~/debug/bound` | `MarkerArray` | behavior_path_planner_node.cpp |  |
| `~/debug/calculation_time` | `StringStamped` | node.cpp, node.cpp and 1 more |  |
| `~/debug/collision_info` | `autoware_internal_debug_msgs::msg::StringStamped` | scene_crosswalk.cpp |  |
| `~/debug/collision_pointcloud` | `PointCloud2` | node.cpp |  |
| `~/debug/cruise_planning_info` | `Float32MultiArrayStamped` | node.cpp |  |
| `~/debug/eb_fixed_traj` | `Trajectory` | elastic_band.cpp |  |
| `~/debug/eb_traj` | `Trajectory` | elastic_band.cpp |  |
| `~/debug/extended_traj` | `Trajectory` | node.cpp, elastic_band_smoother.cpp |  |
| `~/debug/footprint` | `PolygonStamped` | debug_marker.cpp |  |
| `~/debug/footprint_offset` | `PolygonStamped` | debug_marker.cpp |  |
| `~/debug/footprint_recover_offset` | `PolygonStamped` | debug_marker.cpp |  |
| `~/debug/goal_footprint` | `MarkerArray` | default_planner.cpp |  |
| `~/debug/intersection/decision_state` | `std_msgs::msg::String` | manager.cpp |  |
| `~/debug/intersection/ego_ttc` | `autoware_internal_debug_msgs::msg::Float64MultiArrayStamped` | scene_intersection.cpp |  |
| `~/debug/intersection/object_ttc` | `autoware_internal_debug_msgs::msg::Float64MultiArrayStamped` | scene_intersection.cpp |  |
| `~/debug/intersection_traffic_signal` | `autoware_perception_msgs::msg::TrafficLightGroup` | manager.cpp |  |
| `~/debug/mpt_fixed_traj` | `Trajectory` | mpt_optimizer.cpp |  |
| `~/debug/mpt_ref_traj` | `Trajectory` | mpt_optimizer.cpp |  |
| `~/debug/mpt_traj` | `Trajectory` | mpt_optimizer.cpp |  |
| `~/debug/obstacle_cruise/planning_info` | `Float32MultiArrayStamped` | obstacle_cruise_module.cpp |  |
| `~/debug/obstacle_cruise/processing_time_ms` | `Float64Stamped` | obstacle_cruise_module.cpp |  |
| `~/debug/obstacle_pointcloud` | `sensor_msgs::msg::PointCloud2` | node.cpp, node.cpp |  |
| `~/debug/obstacle_slow_down/planning_info` | `Float32MultiArrayStamped` | obstacle_slow_down_module.cpp |  |
| `~/debug/obstacle_slow_down/processing_time_ms` | `Float64Stamped` | obstacle_slow_down_module.cpp |  |
| `~/debug/partial_pose_array` | `PoseArray` | freespace_planner_node.cpp |  |
| `~/debug/pose_array` | `PoseArray` | freespace_planner_node.cpp |  |
| `~/debug/processing_time_detail_ms/obstacle_cruise` | `autoware_utils::ProcessingTimeDetail` | obstacle_cruise_module.cpp |  |
| `~/debug/processing_time_detail_ms/obstacle_slow_down` | `autoware_utils::ProcessingTimeDetail` | obstacle_slow_down_module.cpp |  |
| `~/debug/processing_time_ms` | `autoware_internal_debug_msgs::msg::Float64Stamped` | planning_evaluator_node.cpp, control_evaluator_node.cpp and 14 more |  |
| `~/debug/route_marker` | `MarkerArray` | mission_planner.cpp |  |
| `~/debug/run_out/accel_reason` | `Int32Stamped` | debug.cpp |  |
| `~/debug/run_out/debug_values` | `Float32MultiArrayStamped` | debug.cpp |  |
| `~/debug/run_out/filtered_pointcloud` | `PointCloud2` | debug.cpp |  |
| `~/debug/slow_down_planning_info` | `Float32MultiArrayStamped` | node.cpp |  |
| `~/debug/stop_planning_info` | `Float32MultiArrayStamped` | node.cpp |  |
| `~/debug/turn_signal_info` | `MarkerArray` | behavior_path_planner_node.cpp |  |
| `~/debug/wall_marker` | `MarkerArray` | optimization_based_planner.cpp, optimization_based_planner.cpp |  |
| `~/drivable_lanes/` | `MarkerArray` | scene_module_manager_interface.cpp |  |
| `~/info/` | `MarkerArray` | scene_module_manager_interface.cpp |  |
| `~/input/acceleration` | `AccelWithCovarianceStamped` |  | node.cpp |
| `~/input/expand_stop_range` | `ExpandStopRange` |  | node.cpp |
| `~/input/map` | `HADMapBin` |  | remaining_distance_time_calculator_node.cpp |
| `~/input/modified_goal` | `PoseWithUuidStamped` |  | mission_planner.cpp |
| `~/input/operation_mode_state` | `OperationModeState` |  | mission_planner.cpp |
| `~/input/path` | `Path` |  | node.cpp, node.cpp and 1 more |
| `~/input/scenario` | `autoware_internal_planning_msgs::msg::Scenario` |  | remaining_distance_time_calculator_node.cpp |
| `~/input/trajectory` | `Trajectory` |  | planning_validator.cpp, invalid_trajectory_publisher.cpp and 2 more |
| `~/input/twist` | `nav_msgs::msg::Odometry` |  | motion_evaluator_node.cpp, kinematic_evaluator_node.cpp and 3 more |
| `~/input/vector_map_inside_area_filtered_pointcloud` | `sensor_msgs::msg::PointCloud2` |  | dynamic_obstacle.cpp |
| `~/main/route` | `LaneletRoute` | route_selector.cpp |  |
| `~/main/state` | `RouteState` | route_selector.cpp |  |
| `~/metrics` | `MetricArrayMsg` | planning_evaluator_node.cpp, kinematic_evaluator_node.cpp and 6 more |  |
| `~/mrm/route` | `LaneletRoute` | route_selector.cpp |  |
| `~/mrm/state` | `RouteState` | route_selector.cpp |  |
| `~/obstacle_cruise/debug_markers` | `MarkerArray` | obstacle_cruise_module.cpp |  |
| `~/obstacle_cruise/virtual_walls` | `MarkerArray` | obstacle_cruise_module.cpp |  |
| `~/obstacle_slow_down/debug_markers` | `MarkerArray` | obstacle_slow_down_module.cpp |  |
| `~/obstacle_slow_down/virtual_walls` | `MarkerArray` | obstacle_slow_down_module.cpp |  |
| `~/obstacle_stop/debug_values` | `Float32MultiArrayStamped` | debug_marker.cpp |  |
| `~/optimized_st_graph` | `Trajectory` | optimization_based_planner.cpp, optimization_based_planner.cpp |  |
| `~/optimized_sv_trajectory` | `Trajectory` | optimization_based_planner.cpp, optimization_based_planner.cpp |  |
| `~/output/clear_velocity_limit` | `VelocityLimitClearCommand` | node.cpp |  |
| `~/output/distance` | `autoware_internal_debug_msgs::msg::Float64Stamped` | path_distance_calculator.cpp |  |
| `~/output/grid_map` | `grid_map_msgs::msg::GridMap` | costmap_generator.cpp |  |
| `~/output/hazard_lights_cmd` | `HazardLightsCommand` | behavior_path_planner_node.cpp, external_cmd_selector_node.cpp |  |
| `~/output/infrastructure_commands` | `tier4_v2x_msgs::msg::InfrastructureCommandArray` | manager.cpp |  |
| `~/output/is_reroute_available` | `RerouteAvailability` | behavior_path_planner_node.cpp |  |
| `~/output/markers` | `visualization_msgs::msg::MarkerArray` | planning_validator.cpp, control_validator.cpp |  |
| `~/output/max_velocity` | `VelocityLimit` | node.cpp, node.cpp |  |
| `~/output/mission_remaining_distance_time` | `MissionRemainingDistanceTime` | remaining_distance_time_calculator_node.cpp |  |
| `~/output/modified_goal` | `PoseWithUuidStamped` | behavior_path_planner_node.cpp |  |
| `~/output/occupancy_grid` | `nav_msgs::msg::OccupancyGrid` | costmap_generator.cpp |  |
| `~/output/path` | `PathWithLaneId` | behavior_path_planner_node.cpp, node.cpp and 2 more |  |
| `~/output/stop_reason` | `DiagnosticStatus` | node.cpp |  |
| `~/output/stop_speed_exceeded` | `StopSpeedExceeded` | planner_interface.hpp |  |
| `~/output/traffic_signal` | `autoware_perception_msgs::msg::TrafficLightGroup` | manager.cpp |  |
| `~/output/traj` | `Trajectory` | elastic_band_smoother.cpp |  |
| `~/output/trajectory` | `Trajectory` | freespace_planner_node.cpp, planning_validator.cpp and 3 more |  |
| `~/output/turn_indicators_cmd` | `TurnIndicatorsCommand` | behavior_path_planner_node.cpp, external_cmd_selector_node.cpp |  |
| `~/output/validation_status` | `PlanningValidatorStatus` | planning_validator.cpp, control_validator.cpp |  |
| `~/output/velocity_limit` | `VelocityLimit` | node.cpp, mrm_comfortable_stop_operator_core.cpp |  |
| `~/output/velocity_limit_clear_command` | `VelocityLimitClearCommand` | node.cpp, node.cpp |  |
| `~/planner/route` | `LaneletRoute` |  | route_selector.cpp |
| `~/planner/state` | `RouteState` |  | route_selector.cpp |
| `~/processing_time/` | `autoware_utils::ProcessingTimeDetail` | scene_module_manager_interface.cpp |  |
| `~/route` | `LaneletRoute` | mission_planner.cpp |  |
| `~/state` | `RouteState` | mission_planner.cpp |  |
| `~/virtual_wall` | `visualization_msgs::msg::MarkerArray` | debug_marker.cpp, debug_marker.cpp and 5 more |  |
| `~/virtual_wall/` | `MarkerArray` | scene_module_manager_interface.cpp |  |
| `~/virtual_wall/cruise` | `MarkerArray` | node.cpp |  |
| `~/virtual_wall/slow_down` | `MarkerArray` | node.cpp |  |
| `~/virtual_wall/stop` | `MarkerArray` | node.cpp |  |
