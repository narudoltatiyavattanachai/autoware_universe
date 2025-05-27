# Full Topic Map

This document provides a complete overview of all topics used in Autoware Universe.

| Topic | Message Type | Publishers | Subscribers |
| ----- | ------------ | ---------- | ----------- |
| `/accel_brake_map_calibrator/output/calibration_status` | `CalibrationStatus` | accel_brake_map_calibrator_node.cpp |  |
| `/api/autoware/get/emergency` | `tier4_external_api_msgs::msg::Emergency` |  | autoware_state_panel.cpp |
| `/api/autoware/get/engage` | `AutowareEngage` |  | compatibility.cpp |
| `/api/autoware/get/map/info/hash` | `tier4_external_api_msgs::msg::MapHash` |  | elevation_map_loader_node.cpp |
| `/api/fail_safe/mrm_state` | `MRMState` |  | autoware_state_panel.cpp |
| `/api/localization/initialization_state` | `LocalizationInitializationState` |  | autoware_state_panel.cpp |
| `/api/motion/state` | `MotionState` |  | autoware_state_panel.cpp |
| `/api/operation_mode/state` | `OperationModeState` |  | autoware_state_panel.cpp, operation_mode_debug_panel.cpp |
| `/api/planning/steering_factors` | `SteeringFactorArray` |  | velocity_steering_factors_panel.cpp |
| `/api/planning/velocity_factors` | `VelocityFactorArray` |  | velocity_steering_factors_panel.cpp |
| `/api/routing/state` | `RouteState` |  | autoware_state_panel.cpp |
| `/api/system/diagnostics/status` | `autoware_adapi_v1_msgs::msg::DiagGraphStatus` | diagnostics.cpp |  |
| `/api/system/diagnostics/struct` | `autoware_adapi_v1_msgs::msg::DiagGraphStruct` | diagnostics.cpp |  |
| `/autoware/engage` | `AutowareEngage` | compatibility.cpp |  |
| `/autoware/state` | `AutowareState` | autoware_state.cpp |  |
| `/control/current_gate_mode` | `GateMode` |  | compatibility.cpp |
| `/control/external_cmd_selector/current_selector_mode` | `SelectorModeMsg` |  | compatibility.cpp |
| `/control/gate_mode_cmd` | `GateMode` | compatibility.cpp |  |
| `/diagnostics` | `diagnostic_msgs::msg::DiagnosticArray` | diagnostics_interface.cpp, pose_instability_detector.cpp and 3 more | fusion_node.cpp, test_diagnostics_interface.cpp and 17 more |
| `/diagnostics_array` | `DiagnosticArray` | converter.cpp |  |
| `/diagnostics_graph/status` | `DiagGraphStatus` | aggregator.cpp | subscription.cpp |
| `/diagnostics_graph/struct` | `DiagGraphStruct` | aggregator.cpp | subscription.cpp |
| `/diagnostics_graph/unknowns` | `DiagnosticArray` | aggregator.cpp |  |
| `/external/` | `OperatorHeartbeat` | manual_control.cpp |  |
| `/input_topic` | `Int32` | test_fake_test_node.cpp | test_fake_test_node.cpp |
| `/localization/acceleration` | `geometry_msgs::msg::AccelWithCovarianceStamped` |  | velocity_steering_factors_panel.cpp |
| `/localization/kinematic_state` | `Odometry` | test_filter_in_vehicle_cmd_gate_node.cpp | vehicle_cmd_gate.cpp, velocity_steering_factors_panel.cpp |
| `/map/vector_map` | `LaneletMapBin` |  | traffic_light_publish_panel.cpp |
| `/marker_array` | `MarkerArray` | particle_visualize_node.cpp |  |
| `/output_topic` | `Bool` | test_fake_test_node.cpp | test_fake_test_node.cpp |
| `/particle_array` | `ParticleArray` |  | particle_visualize_node.cpp |
| `/perception/traffic_light_recognition/traffic_signals` | `TrafficLightGroupArray` | traffic_light_publish_panel.cpp |  |
| `/planning/scenario_planning/current_max_velocity` | `autoware_internal_planning_msgs::msg::VelocityLimit` |  | remaining_distance_time_calculator_node.cpp |
| `/planning/scenario_planning/max_velocity_default` | `autoware_internal_planning_msgs::msg::VelocityLimit` | autoware_state_panel.cpp |  |
| `/planning/scenario_planning/trajectory` | `Trajectory` |  | optimization_based_planner.cpp, optimization_based_planner.cpp |
| `/planning_validator/input/acceleration` | `AccelWithCovarianceStamped` | test_planning_validator_pubsub.cpp |  |
| `/planning_validator/input/kinematics` | `Odometry` | test_planning_validator_pubsub.cpp |  |
| `/planning_validator/input/trajectory` | `Trajectory` | test_planning_validator_pubsub.cpp |  |
| `/points_raw` | `sensor_msgs::msg::PointCloud2` | stub_sensor_pcd_publisher.hpp |  |
| `/pose_instability_detector/input/odometry` | `Odometry` | test_message_helper_node.hpp |  |
| `/pose_instability_detector/input/twist` | `TwistWithCovarianceStamped` | test_message_helper_node.hpp |  |
| `/rviz/routing/pose` | `PoseStamped` |  | route_panel.cpp |
| `/service_log` | `ServiceLog` | interface.hpp | service_log_checker.cpp |
| `/system/operation_mode/availability` | `Availability` | availability.cpp |  |
| `/test_ransac_ground_filter/input_cloud` | `sensor_msgs::msg::PointCloud2` | test_ransac_ground_filter.cpp |  |
| `/test_ransac_ground_filter/output_cloud` | `sensor_msgs::msg::PointCloud2` | test_ransac_ground_filter.cpp |  |
| `/test_ray_ground_filter/input_cloud` | `sensor_msgs::msg::PointCloud2` | test_ray_ground_filter.cpp |  |
| `/test_ray_ground_filter/output_cloud` | `sensor_msgs::msg::PointCloud2` | test_ray_ground_filter.cpp |  |
| `/tf` | `tf2_msgs::msg::TFMessage` | simple_planning_simulator_core.cpp |  |
| `/vector_map` | `LaneletMapBin` |  | radar_object_tracker_node.cpp, map_based_prediction_node.cpp |
| `/vehicle/raw_vehicle_cmd_converter/debug/compensated_control_cmd` | `Control` | node.cpp |  |
| `/vehicle/raw_vehicle_cmd_converter/debug/steer_pid` | `Float32MultiArrayStamped` | node.cpp |  |
| `Removed_polygon` | `visualization_msgs::msg::Marker` | polygon_remover.cpp |  |
| `angular_z` | `autoware_internal_debug_msgs::msg::Float32Stamped` | pose2twist_core.cpp |  |
| `autoware/state` | `AutowareState` | test.cpp |  |
| `behavior_velocity_planner_node/input/virtual_traffic_light_states` | `VirtualTrafficLightStateArray` | test_node_interface.cpp |  |
| `blockage_diag/debug/ground_blockage_ratio` | `autoware_internal_debug_msgs::msg::Float32Stamped` | blockage_diag_node.cpp |  |
| `blockage_diag/debug/ground_dust_ratio` | `autoware_internal_debug_msgs::msg::Float32Stamped` | blockage_diag_node.cpp |  |
| `blockage_diag/debug/sky_blockage_ratio` | `autoware_internal_debug_msgs::msg::Float32Stamped` | blockage_diag_node.cpp |  |
| `controller/input/current_accel` | `AccelWithCovarianceStamped` | test_controller_node.cpp |  |
| `controller/input/current_odometry` | `VehicleOdometry` | test_controller_node.cpp |  |
| `controller/input/current_operation_mode` | `OperationModeState` | test_controller_node.cpp |  |
| `controller/input/current_steering` | `SteeringReport` | test_controller_node.cpp |  |
| `controller/input/reference_trajectory` | `Trajectory` | test_controller_node.cpp |  |
| `controller/output/control_cmd` | `Control` |  | test_controller_node.cpp |
| `debug/clusters` | `sensor_msgs::msg::PointCloud2` | euclidean_cluster_node.cpp, voxel_grid_based_euclidean_cluster_node.cpp and 1 more |  |
| `debug/divided_objects` | `autoware_perception_msgs::msg::DetectedObjects` | debugger.hpp |  |
| `debug/downsampled_map/pointcloud` | `sensor_msgs::msg::PointCloud2` | voxel_grid_map_loader.cpp |  |
| `debug/ellipse_marker` | `visualization_msgs::msg::Marker` | localization_error_monitor.cpp |  |
| `debug/ground/pointcloud` | `sensor_msgs::msg::PointCloud2` | node.cpp |  |
| `debug/initial_objects` | `autoware_perception_msgs::msg::DetectedObjects` | debugger.hpp |  |
| `debug/instance_pointcloud` | `sensor_msgs::msg::PointCloud2` | debugger.cpp |  |
| `debug/interpolated_sub_object` | `TrackedObjects` | decorative_tracker_merger_node.cpp |  |
| `debug/loaded_pointcloud_map` | `sensor_msgs::msg::PointCloud2` | map_update_module.cpp |  |
| `debug/merged_objects` | `autoware_perception_msgs::msg::DetectedObjects` | debugger.hpp |  |
| `debug/plane_pose_array` | `geometry_msgs::msg::PoseArray` | node.cpp |  |
| `debug/ring_outlier_filter` | `PointCloud2` | ring_outlier_filter_node.cpp |  |
| `debug/tracked_objects` | `autoware_perception_msgs::msg::DetectedObjects` | debugger.hpp |  |
| `dual_return_outlier_filter/debug/pointcloud_noise` | `sensor_msgs::msg::PointCloud2` | dual_return_outlier_filter_node.cpp |  |
| `dual_return_outlier_filter/debug/visibility` | `autoware_internal_debug_msgs::msg::Float32Stamped` | dual_return_outlier_filter_node.cpp |  |
| `ekf_pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped` |  | ndt_scan_matcher_core.cpp |
| `exe_time_ms` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ndt_scan_matcher_core.cpp |  |
| `gnss_pose_cov` | `PoseWithCovarianceStamped` |  | gnss_module.cpp |
| `image_diag/image_state_diag` | `autoware_internal_debug_msgs::msg::Int32Stamped` | image_diagnostics_node.cpp |  |
| `in/heartbeat` | `ManualOperatorHeartbeat` |  | node.cpp |
| `in/pedals_cmd` | `PedalsCommand` |  | node.cpp |
| `initial_pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped` | ndt_scan_matcher_core.cpp |  |
| `initial_to_result_distance` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ndt_scan_matcher_core.cpp |  |
| `initial_to_result_distance_new` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ndt_scan_matcher_core.cpp |  |
| `initial_to_result_distance_old` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ndt_scan_matcher_core.cpp |  |
| `initial_to_result_relative_pose` | `geometry_msgs::msg::PoseStamped` | ndt_scan_matcher_core.cpp |  |
| `input` | `sensor_msgs::msg::Imu` |  | imu_corrector_core.cpp, livox_tag_filter_node.cpp and 6 more |
| `input/acceleration` | `AccelWithCovarianceStamped` | test_filter_in_vehicle_cmd_gate_node.cpp | vehicle_cmd_gate.cpp |
| `input/ackermann_control_command` | `Control` | test_simple_planning_simulator.cpp | simple_planning_simulator_core.cpp |
| `input/actuation_command` | `ActuationCommandStamped` | test_simple_planning_simulator.cpp | simple_planning_simulator_core.cpp |
| `input/auto/control_cmd` | `Control` | test_filter_in_vehicle_cmd_gate_node.cpp | vehicle_cmd_gate.cpp |
| `input/auto/gear_cmd` | `GearCommand` | test_filter_in_vehicle_cmd_gate_node.cpp |  |
| `input/auto/hazard_lights_cmd` | `HazardLightsCommand` | test_filter_in_vehicle_cmd_gate_node.cpp |  |
| `input/auto/turn_indicators_cmd` | `TurnIndicatorsCommand` | test_filter_in_vehicle_cmd_gate_node.cpp |  |
| `input/elevation_map` | `grid_map_msgs::msg::GridMap` |  | node.cpp |
| `input/emergency/control_cmd` | `Control` |  | vehicle_cmd_gate.cpp |
| `input/engage` | `Engage` | test_filter_in_vehicle_cmd_gate_node.cpp | simple_planning_simulator_core.cpp, vehicle_cmd_gate.cpp |
| `input/external/control_cmd` | `Control` |  | vehicle_cmd_gate.cpp |
| `input/external_emergency_stop_heartbeat` | `Heartbeat` | test_filter_in_vehicle_cmd_gate_node.cpp | vehicle_cmd_gate.cpp |
| `input/gate_mode` | `GateMode` | test_filter_in_vehicle_cmd_gate_node.cpp | vehicle_cmd_gate.cpp |
| `input/gear_command` | `GearCommand` | test_simple_planning_simulator.cpp | simple_planning_simulator_core.cpp |
| `input/ground_truth_pose` | `PoseStamped` |  | reaction_analyzer_node.cpp |
| `input/hazard_lights_command` | `HazardLightsCommand` |  | simple_planning_simulator_core.cpp |
| `input/initialpose` | `PoseWithCovarianceStamped` | test_simple_planning_simulator.cpp | simple_planning_simulator_core.cpp |
| `input/initialtwist` | `TwistStamped` |  | simple_planning_simulator_core.cpp |
| `input/kinematics` | `Odometry` |  | reaction_analyzer_node.cpp |
| `input/lane_driving/trajectory` | `autoware_planning_msgs::msg::Trajectory` |  | node.cpp |
| `input/lanelet_map` | `autoware_map_msgs::msg::LaneletMapBin` |  | node.cpp |
| `input/localization_initialization_state` | `LocalizationInitializationState` |  | reaction_analyzer_node.cpp |
| `input/main_object` | `TrackedObjects` |  | decorative_tracker_merger_node.cpp |
| `input/manual_ackermann_control_command` | `Control` |  | simple_planning_simulator_core.cpp |
| `input/manual_gear_command` | `GearCommand` |  | simple_planning_simulator_core.cpp |
| `input/mrm_state` | `MrmState` | test_filter_in_vehicle_cmd_gate_node.cpp | vehicle_cmd_gate.cpp |
| `input/object` | `autoware_perception_msgs::msg::DetectedObjects` |  | object_range_splitter_node.cpp, position_filter.cpp and 2 more |
| `input/objects` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` |  | low_intensity_cluster_filter_node.cpp |
| `input/obstacle_pointcloud` | `sensor_msgs::msg::PointCloud2` |  | obstacle_collision_checker_node.cpp |
| `input/odom` | `nav_msgs::msg::Odometry` |  | localization_error_monitor.cpp |
| `input/odometry` | `nav_msgs::msg::Odometry` |  | obstacle_collision_checker_node.cpp |
| `input/operation_mode` | `OperationModeState` | test_filter_in_vehicle_cmd_gate_node.cpp | vehicle_cmd_gate.cpp |
| `input/operation_mode_state` | `OperationModeState` |  | reaction_analyzer_node.cpp |
| `input/parking/trajectory` | `autoware_planning_msgs::msg::Trajectory` |  | node.cpp |
| `input/pointcloud` | `PointCloud2` |  | lanelet2_map_filter_node.cpp, node.cpp |
| `input/pointcloud_map` | `sensor_msgs::msg::PointCloud2` |  | elevation_map_loader_node.cpp |
| `input/pointcloud_map_metadata` | `autoware_map_msgs::msg::PointCloudMapMetaData` |  | elevation_map_loader_node.cpp |
| `input/predicted_trajectory` | `autoware_planning_msgs::msg::Trajectory` |  | obstacle_collision_checker_node.cpp |
| `input/raw_image` | `sensor_msgs::msg::Image` |  | image_diagnostics_node.cpp |
| `input/reference_trajectory` | `autoware_planning_msgs::msg::Trajectory` |  | obstacle_collision_checker_node.cpp |
| `input/route` | `autoware_planning_msgs::msg::LaneletRoute` |  | node.cpp, goal_pose_visualizer.cpp |
| `input/routing_state` | `RouteState` |  | reaction_analyzer_node.cpp |
| `input/steering` | `SteeringReport` | test_filter_in_vehicle_cmd_gate_node.cpp | vehicle_cmd_gate.cpp |
| `input/sub_object` | `TrackedObjects` |  | decorative_tracker_merger_node.cpp |
| `input/trajectory` | `Trajectory` |  | simple_planning_simulator_core.cpp |
| `input/turn_indicators_command` | `TurnIndicatorsCommand` |  | simple_planning_simulator_core.cpp |
| `input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` |  | lanelet2_map_filter_node.cpp, vector_map_inside_area_filter_node.cpp and 3 more |
| `input/velocity_limit_clear_command_from_internal` | `VelocityLimitClearCommand` |  | external_velocity_limit_selector_node.cpp |
| `input/velocity_limit_from_api` | `VelocityLimit` |  | external_velocity_limit_selector_node.cpp |
| `input/velocity_limit_from_internal` | `VelocityLimit` |  | external_velocity_limit_selector_node.cpp |
| `input_geo_pose` | `GeoPoseWithCovariance` |  | geo_pose_projector.cpp |
| `input_gnss_pose_with_cov_topic` | `geometry_msgs::msg::PoseWithCovarianceStamped` |  | pose_covariance_modifier.cpp |
| `input_ndt_pose_with_cov_topic` | `geometry_msgs::msg::PoseWithCovarianceStamped` |  | pose_covariance_modifier.cpp |
| `is_completed` | `std_msgs::msg::Bool` | freespace_planner_node.cpp |  |
| `iteration_num` | `autoware_internal_debug_msgs::msg::Int32Stamped` | ndt_scan_matcher_core.cpp |  |
| `kinematic_state` | `nav_msgs::msg::Odometry` |  | voxel_grid_map_loader.cpp |
| `linear_x` | `autoware_internal_debug_msgs::msg::Float32Stamped` | pose2twist_core.cpp |  |
| `maneuver` | `visualization_msgs::msg::MarkerArray` | map_based_prediction_node.cpp |  |
| `map` | `sensor_msgs::msg::PointCloud2` |  | voxel_grid_map_loader.cpp |
| `monte_carlo_initial_pose_marker` | `visualization_msgs::msg::MarkerArray` | ndt_scan_matcher_core.cpp |  |
| `multi_initial_pose` | `geometry_msgs::msg::PoseArray` | ndt_scan_matcher_core.cpp |  |
| `multi_ndt_pose` | `geometry_msgs::msg::PoseArray` | ndt_scan_matcher_core.cpp |  |
| `ndt_marker` | `visualization_msgs::msg::MarkerArray` | ndt_scan_matcher_core.cpp |  |
| `ndt_pose` | `geometry_msgs::msg::PoseStamped` | ndt_scan_matcher_core.cpp |  |
| `ndt_pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped` | ndt_scan_matcher_core.cpp |  |
| `nearest_voxel_transformation_likelihood` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ndt_scan_matcher_core.cpp |  |
| `no_ground_nearest_voxel_transformation_likelihood` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ndt_scan_matcher_core.cpp |  |
| `no_ground_transform_probability` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ndt_scan_matcher_core.cpp |  |
| `objects` | `DetectedObjectsWithFeature` | shape_estimation_node.cpp |  |
| `out/control_cmd` | `Control` | node.cpp |  |
| `output` | `sensor_msgs::msg::Imu` | imu_corrector_core.cpp, livox_tag_filter_node.cpp and 12 more |  |
| `output/acceleration` | `AccelWithCovarianceStamped` | simple_planning_simulator_core.cpp |  |
| `output/actuation_status` | `ActuationStatusStamped` | simple_planning_simulator_core.cpp |  |
| `output/clusters` | `DetectedObjectsWithFeature` | cluster_merger_node.cpp |  |
| `output/control_cmd` | `Control` | simple_trajectory_follower.cpp, vehicle_cmd_gate.cpp | test_filter_in_vehicle_cmd_gate_node.cpp |
| `output/control_command` | `autoware_control_msgs::msg::Control` | joy_controller_node.cpp |  |
| `output/control_mode_report` | `ControlModeReport` | simple_planning_simulator_core.cpp |  |
| `output/debug` | `StringStamped` | external_velocity_limit_selector_node.cpp |  |
| `output/dynamic_object` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | node.cpp |  |
| `output/elevation_map` | `grid_map_msgs::msg::GridMap` | elevation_map_loader_node.cpp |  |
| `output/elevation_map_cloud` | `sensor_msgs::msg::PointCloud2` | elevation_map_loader_node.cpp |  |
| `output/engage` | `EngageMsg` | vehicle_cmd_gate.cpp |  |
| `output/external_control_command` | `tier4_external_api_msgs::msg::ControlCommandStamped` | joy_controller_node.cpp |  |
| `output/external_emergency` | `Emergency` | vehicle_cmd_gate.cpp |  |
| `output/external_velocity_limit` | `VelocityLimit` | external_velocity_limit_selector_node.cpp |  |
| `output/gate_mode` | `tier4_control_msgs::msg::GateMode` | joy_controller_node.cpp, vehicle_cmd_gate.cpp |  |
| `output/gear_cmd` | `autoware_vehicle_msgs::msg::GearCommand` | autoware_shift_decider.cpp, vehicle_cmd_gate.cpp |  |
| `output/gear_report` | `GearReport` | simple_planning_simulator_core.cpp |  |
| `output/goal` | `geometry_msgs::msg::PoseStamped` | reaction_analyzer_node.cpp |  |
| `output/goal_pose` | `geometry_msgs::msg::PoseStamped` | goal_pose_visualizer.cpp |  |
| `output/hazard_lights_cmd` | `HazardLightsCommand` | vehicle_cmd_gate.cpp |  |
| `output/hazard_lights_report` | `HazardLightsReport` | simple_planning_simulator_core.cpp |  |
| `output/heartbeat` | `tier4_external_api_msgs::msg::Heartbeat` | joy_controller_node.cpp |  |
| `output/imu` | `Imu` | simple_planning_simulator_core.cpp |  |
| `output/initialpose` | `geometry_msgs::msg::PoseWithCovarianceStamped` | reaction_analyzer_node.cpp |  |
| `output/labeled_clusters` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | node.cpp |  |
| `output/long_range_object` | `autoware_perception_msgs::msg::DetectedObjects` | object_range_splitter_node.cpp |  |
| `output/object` | `autoware_perception_msgs::msg::DetectedObjects` | object_association_merger_node.cpp, decorative_tracker_merger_node.cpp and 2 more |  |
| `output/objects` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | low_intensity_cluster_filter_node.cpp, topic_publisher.cpp |  |
| `output/odometry` | `Odometry` | simple_planning_simulator_core.cpp | test_simple_planning_simulator.cpp |
| `output/operation_mode` | `OperationModeState` | vehicle_cmd_gate.cpp |  |
| `output/pointcloud` | `PointCloud2` | topic_publisher.cpp |  |
| `output/points_raw` | `sensor_msgs::msg::PointCloud2` | node.cpp |  |
| `output/pose` | `PoseWithCovarianceStamped` | simple_planning_simulator_core.cpp |  |
| `output/scenario` | `autoware_internal_planning_msgs::msg::Scenario` | node.cpp |  |
| `output/shift` | `tier4_external_api_msgs::msg::GearShiftStamped` | joy_controller_node.cpp |  |
| `output/short_range_object` | `autoware_perception_msgs::msg::DetectedObjects` | object_range_splitter_node.cpp |  |
| `output/steering` | `SteeringReport` | simple_planning_simulator_core.cpp |  |
| `output/traffic_rois` | `TrafficLightRoiArray` | traffic_light_selector_node.cpp |  |
| `output/traffic_signals` | `TrafficLightArray` | traffic_light_category_merger_node.cpp |  |
| `output/trajectory` | `autoware_planning_msgs::msg::Trajectory` | node.cpp |  |
| `output/turn_indicators_cmd` | `TurnIndicatorsCommand` | vehicle_cmd_gate.cpp |  |
| `output/turn_indicators_report` | `TurnIndicatorsReport` | simple_planning_simulator_core.cpp |  |
| `output/turn_signal` | `tier4_external_api_msgs::msg::TurnSignalStamped` | joy_controller_node.cpp |  |
| `output/twist` | `VelocityReport` | simple_planning_simulator_core.cpp |  |
| `output/vehicle_cmd_emergency` | `VehicleEmergencyStamped` | vehicle_cmd_gate.cpp |  |
| `output/vehicle_engage` | `autoware_vehicle_msgs::msg::Engage` | joy_controller_node.cpp |  |
| `output_pose` | `PoseWithCovariance` | geo_pose_projector.cpp |  |
| `output_pose_with_covariance_topic` | `geometry_msgs::msg::PoseWithCovarianceStamped` | pose_covariance_modifier.cpp |  |
| `pointcloud_map` | `sensor_msgs::msg::PointCloud2` |  | pcd_map_tf_generator_node.cpp |
| `points_aligned` | `sensor_msgs::msg::PointCloud2` | ndt_scan_matcher_core.cpp |  |
| `points_aligned_no_ground` | `sensor_msgs::msg::PointCloud2` | ndt_scan_matcher_core.cpp |  |
| `points_raw` | `sensor_msgs::msg::PointCloud2` |  | ndt_scan_matcher_core.cpp |
| `pose` | `geometry_msgs::msg::PoseStamped` |  | pose2twist_core.cpp |
| `pose_reset` | `PoseWithCovarianceStamped` | pose_initializer_core.cpp |  |
| `processing_time` | `autoware::universe_utils::ProcessingTimeDetail` | example_time_keeper.cpp, costmap_generator.cpp |  |
| `regularization_pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped` |  | ndt_scan_matcher_core.cpp |
| `ring_outlier_filter/debug/visibility` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ring_outlier_filter_node.cpp |  |
| `stop_check_twist` | `TwistWithCovarianceStamped` |  | stop_check_module.cpp |
| `topic` | `std_msgs::msg::String` | example_polling_subscriber.cpp |  |
| `topic_name` | `T` |  | shared_data.hpp |
| `transform_probability` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ndt_scan_matcher_core.cpp |  |
| `twist` | `geometry_msgs::msg::TwistStamped` | pose2twist_core.cpp |  |
| `vector_map` | `autoware_map_msgs::msg::LaneletMapBin` |  | vector_map_tf_generator_node.cpp |
| `voxel_score_points` | `sensor_msgs::msg::PointCloud2` | ndt_scan_matcher_core.cpp |  |
| `~/` | `visualization_msgs::msg::MarkerArray` | run_out_module.cpp, dynamic_obstacle_stop_module.cpp and 2 more |  |
| `~/adaptive_cruise_control/debug_values` | `autoware_internal_debug_msgs::msg::Float32MultiArrayStamped` | adaptive_cruise_control.cpp |  |
| `~/boundary` | `Trajectory` | optimization_based_planner.cpp, optimization_based_planner.cpp |  |
| `~/component_system_usage` | `ResourceUsageReport` | component_monitor_node.cpp |  |
| `~/cpu_usage` | `tier4_external_api_msgs::msg::CpuUsage` | cpu_monitor_base.cpp |  |
| `~/crop_box_polygon` | `geometry_msgs::msg::PolygonStamped` | crop_box_filter_node.cpp |  |
| `~/debug` | `visualization_msgs::msg::Marker` | reaction_analyzer_node.cpp |  |
| `~/debug/` | `autoware_internal_debug_msgs::msg::Float64Stamped` | run_out_module.cpp, dynamic_obstacle_stop_module.cpp and 3 more |  |
| `~/debug/avoidance_debug_message_array` | `AvoidanceDebugMsgArray` | behavior_path_planner_node.cpp |  |
| `~/debug/bound` | `MarkerArray` | behavior_path_planner_node.cpp |  |
| `~/debug/calculation_time` | `StringStamped` | node.cpp, node.cpp and 1 more |  |
| `~/debug/collision_info` | `autoware_internal_debug_msgs::msg::StringStamped` | scene_crosswalk.cpp |  |
| `~/debug/collision_pointcloud` | `PointCloud2` | node.cpp |  |
| `~/debug/control_cmd_horizon` | `autoware_control_msgs::msg::ControlHorizon` | controller_node.cpp |  |
| `~/debug/cost_map_image` | `Image` | camera_particle_corrector_core.cpp |  |
| `~/debug/cost_map_range` | `MarkerArray` | camera_particle_corrector_core.cpp |  |
| `~/debug/cruise_planning_info` | `Float32MultiArrayStamped` | node.cpp |  |
| `~/debug/data_average_occ_map` | `OccupancyGrid` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/data_count_occ_map` | `OccupancyGrid` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/data_count_self_pose_occ_map` | `OccupancyGrid` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/data_std_dev_occ_map` | `OccupancyGrid` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/debug_line_segments` | `PointCloud2` | segment_filter_core.cpp |  |
| `~/debug/detected_tag` | `PoseArray` | ar_tag_based_localizer.cpp |  |
| `~/debug/diff_pose` | `PoseStamped` | pose_instability_detector.cpp |  |
| `~/debug/eb_fixed_traj` | `Trajectory` | elastic_band.cpp |  |
| `~/debug/eb_traj` | `Trajectory` | elastic_band.cpp |  |
| `~/debug/error_graph_text` | `autoware_internal_debug_msgs::msg::StringStamped` | logging.cpp |  |
| `~/debug/exe_time_ms` | `autoware_internal_debug_msgs::msg::Float32Stamped` | traffic_light_fine_detector_node.cpp |  |
| `~/debug/extended_traj` | `Trajectory` | node.cpp, elastic_band_smoother.cpp |  |
| `~/debug/footprint` | `PolygonStamped` | debug_marker.cpp |  |
| `~/debug/footprint_offset` | `PolygonStamped` | debug_marker.cpp |  |
| `~/debug/footprint_recover_offset` | `PolygonStamped` | debug_marker.cpp |  |
| `~/debug/gnss_position_stddev` | `std_msgs::msg::Float64` | pose_covariance_modifier.cpp |  |
| `~/debug/gnss_range_marker` | `MarkerArray` | gnss_corrector_core.cpp |  |
| `~/debug/goal_footprint` | `MarkerArray` | default_planner.cpp |  |
| `~/debug/ground_markers` | `Marker` | ground_server_core.cpp |  |
| `~/debug/ground_status` | `String` | ground_server_core.cpp |  |
| `~/debug/image` | `Image` | ar_tag_based_localizer.cpp |  |
| `~/debug/image_with_colored_line_segments` | `Image` | line_segments_overlay_core.cpp |  |
| `~/debug/image_with_line_segments` | `Image` | line_segment_detector_core.cpp |  |
| `~/debug/init_candidates` | `MarkerArray` | marker_module.cpp |  |
| `~/debug/init_marker` | `Marker` | predictor.cpp |  |
| `~/debug/intersection/decision_state` | `std_msgs::msg::String` | manager.cpp |  |
| `~/debug/intersection/ego_ttc` | `autoware_internal_debug_msgs::msg::Float64MultiArrayStamped` | scene_intersection.cpp |  |
| `~/debug/intersection/object_ttc` | `autoware_internal_debug_msgs::msg::Float64MultiArrayStamped` | scene_intersection.cpp |  |
| `~/debug/intersection_traffic_signal` | `autoware_perception_msgs::msg::TrafficLightGroup` | manager.cpp |  |
| `~/debug/lanelet2_overlay_image` | `sensor_msgs::msg::Image` | lanelet2_overlay_core.cpp |  |
| `~/debug/ld_outputs` | `autoware_internal_debug_msgs::msg::Float32MultiArrayStamped` | autoware_pure_pursuit_lateral_controller.cpp |  |
| `~/debug/low_confidence_objects` | `DetectedObjects` | node.cpp |  |
| `~/debug/mapped_tag` | `MarkerArray` | ar_tag_based_localizer.cpp |  |
| `~/debug/marker` | `visualization_msgs::msg::MarkerArray` | lanelet_filter.cpp, debug_marker.cpp and 7 more |  |
| `~/debug/marker_array` | `MarkerArray` | pose_estimator_arbiter_core.cpp |  |
| `~/debug/marker_detected` | `PoseArray` | lidar_marker_localizer.cpp |  |
| `~/debug/marker_mapped` | `MarkerArray` | lidar_marker_localizer.cpp |  |
| `~/debug/marker_pointcloud` | `PointCloud2` | lidar_marker_localizer.cpp |  |
| `~/debug/markers` | `visualization_msgs::msg::MarkerArray` | traffic_light_map_based_detector_node.cpp, autoware_pure_pursuit_lateral_controller.cpp and 1 more |  |
| `~/debug/match_image` | `Image` | camera_particle_corrector_core.cpp |  |
| `~/debug/mpt_fixed_traj` | `Trajectory` | mpt_optimizer.cpp |  |
| `~/debug/mpt_ref_traj` | `Trajectory` | mpt_optimizer.cpp |  |
| `~/debug/mpt_traj` | `Trajectory` | mpt_optimizer.cpp |  |
| `~/debug/ndt_position_stddev` | `std_msgs::msg::Float64` | pose_covariance_modifier.cpp |  |
| `~/debug/near_cloud` | `PointCloud2` | ground_server_core.cpp |  |
| `~/debug/neighbor_pointcloud` | `sensor_msgs::msg::PointCloud2` | debugger.hpp |  |
| `~/debug/objects_markers` | `visualization_msgs::msg::MarkerArray` | debugger.cpp |  |
| `~/debug/obstacle_cruise/planning_info` | `Float32MultiArrayStamped` | obstacle_cruise_module.cpp |  |
| `~/debug/obstacle_cruise/processing_time_ms` | `Float64Stamped` | obstacle_cruise_module.cpp |  |
| `~/debug/obstacle_pointcloud` | `sensor_msgs::msg::PointCloud2` | node.cpp, node.cpp |  |
| `~/debug/obstacle_slow_down/planning_info` | `Float32MultiArrayStamped` | obstacle_slow_down_module.cpp |  |
| `~/debug/obstacle_slow_down/processing_time_ms` | `Float64Stamped` | obstacle_slow_down_module.cpp |  |
| `~/debug/occ_index` | `MarkerArray` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/offset_covariance` | `Float32MultiArray` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/original_occ_map` | `OccupancyGrid` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/original_raw_map` | `Float32MultiArray` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/painted_pointcloud` | `PointCloudMsgType` | node.cpp |  |
| `~/debug/partial_pose_array` | `PoseArray` | freespace_planner_node.cpp |  |
| `~/debug/particles_marker_array` | `MarkerArray` | visualize.cpp |  |
| `~/debug/pointcloud_within_polygon` | `sensor_msgs::msg::PointCloud2` | debugger.hpp |  |
| `~/debug/pose_array` | `PoseArray` | freespace_planner_node.cpp |  |
| `~/debug/pose_with_covariance` | `PoseWithCovarianceStamped` | lidar_marker_localizer.cpp |  |
| `~/debug/predicted_trajectory_in_frenet_coordinate` | `Trajectory` | mpc.cpp |  |
| `~/debug/processing_time_detail_ms` | `autoware_utils::ProcessingTimeDetail` | multi_object_tracker_node.cpp, laserscan_based_occupancy_grid_map_node.cpp and 10 more |  |
| `~/debug/processing_time_detail_ms/obstacle_cruise` | `autoware_utils::ProcessingTimeDetail` | obstacle_cruise_module.cpp |  |
| `~/debug/processing_time_detail_ms/obstacle_slow_down` | `autoware_utils::ProcessingTimeDetail` | obstacle_slow_down_module.cpp |  |
| `~/debug/processing_time_ms` | `autoware_internal_debug_msgs::msg::Float64Stamped` | planning_evaluator_node.cpp, control_evaluator_node.cpp and 14 more |  |
| `~/debug/processing_time_tree` | `autoware::universe_utils::ProcessingTimeDetail` | test_time_keeper.cpp |  |
| `~/debug/projected_image` | `Image` | segment_filter_core.cpp |  |
| `~/debug/projected_marker` | `Marker` | lanelet2_overlay_core.cpp |  |
| `~/debug/removed_objects` | `autoware_perception_msgs::msg::DetectedObjects` | debugger.hpp |  |
| `~/debug/resampled_reference_trajectory` | `Trajectory` | mpc.cpp |  |
| `~/debug/route_marker` | `MarkerArray` | mission_planner.cpp |  |
| `~/debug/rss_distance` | `tier4_debug_msgs::msg::Float32Stamped` | node.cpp |  |
| `~/debug/run_out/accel_reason` | `Int32Stamped` | debug.cpp |  |
| `~/debug/run_out/debug_values` | `Float32MultiArrayStamped` | debug.cpp |  |
| `~/debug/run_out/filtered_pointcloud` | `PointCloud2` | debug.cpp |  |
| `~/debug/scored_cloud` | `PointCloud2` | camera_particle_corrector_core.cpp |  |
| `~/debug/scored_post_cloud` | `PointCloud2` | camera_particle_corrector_core.cpp |  |
| `~/debug/segmented_image` | `Image` | graph_segment_core.cpp |  |
| `~/debug/sign_board_marker` | `MarkerArray` | ll2_decomposer_core.cpp |  |
| `~/debug/single_frame_map` | `nav_msgs::msg::OccupancyGrid` | synchronized_grid_map_fusion_node.cpp |  |
| `~/debug/slow_down_planning_info` | `Float32MultiArrayStamped` | node.cpp |  |
| `~/debug/state_string` | `String` | camera_particle_corrector_core.cpp |  |
| `~/debug/stop_planning_info` | `Float32MultiArrayStamped` | node.cpp |  |
| `~/debug/string` | `String` | pose_estimator_arbiter_core.cpp |  |
| `~/debug/tentative_objects` | `autoware_perception_msgs::msg::TrackedObjects` | debugger.cpp |  |
| `~/debug/turn_signal_info` | `MarkerArray` | behavior_path_planner_node.cpp |  |
| `~/debug/update_occ_map` | `OccupancyGrid` | accel_brake_map_calibrator_node.cpp |  |
| `~/debug/virtual_wall` | `visualization_msgs::msg::MarkerArray` | debug_marker.cpp |  |
| `~/debug/wall_marker` | `MarkerArray` | optimization_based_planner.cpp, optimization_based_planner.cpp |  |
| `~/debug_info` | `ModeChangeBase::DebugInfo` | node.cpp |  |
| `~/debug_markers` | `visualization_msgs::msg::MarkerArray` | node.hpp |  |
| `~/doors/status` | `DoorStatusArray` | dummy_doors.cpp |  |
| `~/drivable_lanes/` | `MarkerArray` | scene_module_manager_interface.cpp |  |
| `~/expect/rois` | `tier4_perception_msgs::msg::TrafficLightRoiArray` | traffic_light_map_based_detector_node.cpp |  |
| `~/hazard_status` | `HazardStatusStamped` | converter.cpp |  |
| `~/in/rect` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` |  | bytetrack_node.cpp |
| `~/info/` | `MarkerArray` | scene_module_manager_interface.cpp |  |
| `~/initialpose` | `PoseWithCovarianceStamped` |  | initial_pose_adaptor.cpp |
| `~/input` | `DetectedObjectsWithFeature` |  | detected_object_feature_remover_node.cpp |
| `~/input/acceleration` | `AccelWithCovarianceStamped` |  | node.cpp |
| `~/input/actuation_status` | `ActuationStatusStamped` |  | node.cpp |
| `~/input/artag/image` | `Image` |  | pose_estimator_arbiter_core.cpp |
| `~/input/camera_info` | `sensor_msgs::msg::CameraInfo` |  | traffic_light_map_based_detector_node.cpp, bevfusion_node.cpp and 4 more |
| `~/input/classified/traffic_signals` | `TrafficSignalArray` |  | node.cpp |
| `~/input/compressed_image` | `sensor_msgs::msg::CompressedImage` |  | image_transport_decompressor.cpp |
| `~/input/control/control_cmd` | `Control` |  | mrm_emergency_stop_operator_core.cpp |
| `~/input/control_cmd` | `Control` |  | node.cpp, control_validator.cpp |
| `~/input/control_raw` | `Control` |  | control_performance_analysis_node.cpp |
| `~/input/current_accel` | `geometry_msgs::msg::AccelWithCovarianceStamped` |  | predicted_path_checker_node.cpp |
| `~/input/eagleye/pose_with_covariance` | `PoseCovStamped` |  | pose_estimator_arbiter_core.cpp |
| `~/input/ekf_pose` | `PoseWithCovarianceStamped` |  | ar_tag_based_localizer.cpp, lidar_marker_localizer.cpp and 1 more |
| `~/input/expand_stop_range` | `ExpandStopRange` |  | node.cpp |
| `~/input/fixed_goal` | `PoseStamped` |  | routing_adaptor.cpp |
| `~/input/ground` | `Float32Array` |  | lanelet2_overlay_core.cpp |
| `~/input/height` | `std_msgs::msg::Float32` |  | predictor.cpp, gnss_corrector_core.cpp |
| `~/input/image` | `sensor_msgs::msg::Image` |  | bevfusion_node.cpp, ar_tag_based_localizer.cpp |
| `~/input/image_raw` | `Image` |  | camera_pose_initializer_core.cpp, line_segments_overlay_core.cpp and 4 more |
| `~/input/image_raw/compressed` | `CompressedImage` |  | undistort_node.cpp |
| `~/input/imu` | `Imu` | test.cpp |  |
| `~/input/imu_raw` | `Imu` |  | gyro_bias_estimator.cpp |
| `~/input/initial_objects` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` |  | detection_by_tracker_node.cpp |
| `~/input/initialization_state` | `InitializationState` |  | pose_estimator_arbiter_core.cpp |
| `~/input/initialpose` | `PoseCovStamped` |  | predictor.cpp |
| `~/input/lanelet2_map` | `autoware_map_msgs::msg::LaneletMapBin` |  | traffic_light_recognition_marker_publisher.cpp, ar_tag_based_localizer.cpp and 1 more |
| `~/input/line_segments` | `PointCloud2` |  | line_segments_overlay_core.cpp |
| `~/input/line_segments_cloud` | `PointCloud2` |  | camera_particle_corrector_core.cpp |
| `~/input/ll2_bounding_box` | `PointCloud2` |  | camera_particle_corrector_core.cpp |
| `~/input/ll2_road_marking` | `PointCloud2` |  | camera_particle_corrector_core.cpp, lanelet2_overlay_core.cpp |
| `~/input/ll2_sign_board` | `PointCloud2` |  | lanelet2_overlay_core.cpp |
| `~/input/local/gear_cmd` | `GearCommand` |  | external_cmd_selector_node.cpp |
| `~/input/local/hazard_lights_cmd` | `HazardLightsCommand` |  | external_cmd_selector_node.cpp |
| `~/input/local/heartbeat` | `OperatorHeartbeat` |  | external_cmd_selector_node.cpp |
| `~/input/local/pedals_cmd` | `PedalsCommand` |  | external_cmd_selector_node.cpp |
| `~/input/local/steering_cmd` | `SteeringCommand` |  | external_cmd_selector_node.cpp |
| `~/input/local/turn_indicators_cmd` | `TurnIndicatorsCommand` |  | external_cmd_selector_node.cpp |
| `~/input/map` | `HADMapBin` |  | remaining_distance_time_calculator_node.cpp |
| `~/input/measured_steering` | `SteeringReport` |  | control_performance_analysis_node.cpp |
| `~/input/modified_goal` | `PoseWithUuidStamped` |  | mission_planner.cpp |
| `~/input/ndt/pointcloud` | `PointCloud2` |  | pose_estimator_arbiter_core.cpp |
| `~/input/objects` | `PredictedObjects` | test.cpp | perception_online_evaluator_node.cpp, radar_objects_adapter.cpp and 6 more |
| `~/input/obstacle_pointcloud` | `PointCloud2` |  | pointcloud_based_occupancy_grid_map_node.cpp |
| `~/input/odom` | `Odometry` |  | gyro_bias_estimator.cpp, time_synchronizer_node.cpp |
| `~/input/odometry` | `Odometry` |  | radar_tracks_msgs_converter_node.cpp, pose_instability_detector.cpp and 5 more |
| `~/input/operation_mode_availability` | `tier4_system_msgs::msg::OperationModeAvailability` |  | mrm_handler_core.cpp |
| `~/input/operation_mode_state` | `OperationModeState` |  | mission_planner.cpp |
| `~/input/path` | `Path` |  | node.cpp, node.cpp and 1 more |
| `~/input/pointcloud` | `sensor_msgs::msg::PointCloud2` | test.cpp | cuda_pointcloud_preprocessor_node.cpp, distortion_corrector_node.cpp and 3 more |
| `~/input/pointcloud_map` | `PointCloud2` |  | pose_estimator_arbiter_core.cpp |
| `~/input/pose` | `PoseStamped` |  | camera_particle_corrector_core.cpp, ground_server_core.cpp and 1 more |
| `~/input/pose_with_covariance` | `PoseCovStamped` |  | pose_estimator_arbiter_core.cpp, gnss_corrector_core.cpp |
| `~/input/predicted_particles` | `ParticleArray` |  | abstract_corrector.cpp |
| `~/input/predicted_trajectory` | `Trajectory` | test.cpp | predicted_path_checker_node.cpp |
| `~/input/projected_line_segments_cloud` | `PointCloud2` |  | lanelet2_overlay_core.cpp |
| `~/input/radar` | `RadarScan` |  | radar_scan_to_pointcloud2_node.cpp, radar_threshold_filter_node.cpp |
| `~/input/radar_info` | `autoware_sensing_msgs::msg::RadarInfo` |  | radar_objects_adapter.cpp |
| `~/input/radar_objects` | `RadarTracks` |  | radar_tracks_msgs_converter_node.cpp |
| `~/input/raw_pointcloud` | `PointCloud2` |  | pointcloud_based_occupancy_grid_map_node.cpp |
| `~/input/reference_trajectory` | `Trajectory` |  | control_performance_analysis_node.cpp, predicted_path_checker_node.cpp |
| `~/input/remote/gear_cmd` | `GearCommand` |  | external_cmd_selector_node.cpp |
| `~/input/remote/hazard_lights_cmd` | `HazardLightsCommand` |  | external_cmd_selector_node.cpp |
| `~/input/remote/heartbeat` | `OperatorHeartbeat` |  | external_cmd_selector_node.cpp |
| `~/input/remote/pedals_cmd` | `PedalsCommand` |  | external_cmd_selector_node.cpp |
| `~/input/remote/steering_cmd` | `SteeringCommand` |  | external_cmd_selector_node.cpp |
| `~/input/remote/turn_indicators_cmd` | `TurnIndicatorsCommand` |  | external_cmd_selector_node.cpp |
| `~/input/reroute` | `PoseStamped` |  | routing_adaptor.cpp |
| `~/input/rough_goal` | `PoseStamped` |  | routing_adaptor.cpp |
| `~/input/route` | `LaneletRoute` |  | node.cpp, traffic_light_map_based_detector_node.cpp and 2 more |
| `~/input/scenario` | `autoware_internal_planning_msgs::msg::Scenario` |  | remaining_distance_time_calculator_node.cpp |
| `~/input/simulation_events` | `SimulationEvents` |  | fault_injection_node.cpp |
| `~/input/steer` | `Steering` |  | steer_offset_estimator_node.cpp |
| `~/input/steering` | `Steering` |  | node.cpp |
| `~/input/tl_state` | `autoware_perception_msgs::msg::TrafficLightGroupArray` |  | node.cpp |
| `~/input/tracked_objects` | `autoware_perception_msgs::msg::TrackedObjects` |  | detection_by_tracker_node.cpp |
| `~/input/tracks` | `RadarTracks` |  | radar_tracks_noise_filter_node.cpp |
| `~/input/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` |  | traffic_light_recognition_marker_publisher.cpp |
| `~/input/trajectory` | `Trajectory` |  | planning_validator.cpp, invalid_trajectory_publisher.cpp and 2 more |
| `~/input/twist` | `nav_msgs::msg::Odometry` |  | motion_evaluator_node.cpp, kinematic_evaluator_node.cpp and 3 more |
| `~/input/twist_with_covariance` | `TwistCovStamped` |  | predictor.cpp |
| `~/input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` |  | traffic_light_multi_camera_fusion_node.cpp, node.cpp and 10 more |
| `~/input/vector_map_inside_area_filtered_pointcloud` | `sensor_msgs::msg::PointCloud2` |  | dynamic_obstacle.cpp |
| `~/input/velocity` | `VelocityReport` | test.cpp |  |
| `~/input/waypoint` | `PoseStamped` |  | routing_adaptor.cpp |
| `~/input/weighted_particles` | `ParticleArray` |  | predictor.cpp |
| `~/input/yabloc/image` | `Image` |  | pose_estimator_arbiter_core.cpp |
| `~/input/yabloc_pose` | `PoseStamped` |  | availability_module.cpp |
| `~/is_filter_activated` | `IsFilterActivated` | vehicle_cmd_gate.cpp |  |
| `~/is_filter_activated/flag` | `BoolStamped` | vehicle_cmd_gate.cpp |  |
| `~/is_filter_activated/marker` | `MarkerArray` | vehicle_cmd_gate.cpp |  |
| `~/is_filter_activated/marker_raw` | `MarkerArray` | vehicle_cmd_gate.cpp |  |
| `~/lateral/debug/processing_time_ms` | `Float64Stamped` | controller_node.cpp |  |
| `~/longitudinal/debug/processing_time_ms` | `Float64Stamped` | controller_node.cpp |  |
| `~/main/route` | `LaneletRoute` | route_selector.cpp |  |
| `~/main/state` | `RouteState` | route_selector.cpp |  |
| `~/markers` | `MarkerArray` | perception_online_evaluator_node.cpp |  |
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
| `~/out/objects` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | tensorrt_yolox_node.cpp, bytetrack_node.cpp |  |
| `~/out/objects/debug/uuid` | `tier4_perception_msgs::msg::DynamicObjectArray` | bytetrack_node.cpp |  |
| `~/output` | `autoware_perception_msgs::msg::DetectedObjects` | detection_by_tracker_node.cpp, detected_object_feature_remover_node.cpp |  |
| `~/output/actuation_cmd` | `ActuationCommandStamped` | node.cpp |  |
| `~/output/amplitude_pointcloud` | `PointCloud2` | radar_scan_to_pointcloud2_node.cpp |  |
| `~/output/artag/image` | `Image` | stopper_artag.hpp |  |
| `~/output/clear_velocity_limit` | `VelocityLimitClearCommand` | node.cpp |  |
| `~/output/control_cmd` | `autoware_control_msgs::msg::Control` | controller_node.cpp |  |
| `~/output/current_map_error` | `Float32Stamped` | accel_brake_map_calibrator_node.cpp |  |
| `~/output/current_selector_mode` | `CommandSourceMode` | external_cmd_selector_node.cpp |  |
| `~/output/debug/ground_truth_objects` | `autoware_perception_msgs::msg::TrackedObjects` | node.cpp |  |
| `~/output/debug/high_confidence/pointcloud` | `PointCloud2` | occupancy_grid_map_outlier_filter_node.cpp |  |
| `~/output/debug/low_confidence/pointcloud` | `PointCloud2` | occupancy_grid_map_outlier_filter_node.cpp |  |
| `~/output/debug/outlier/pointcloud` | `PointCloud2` | occupancy_grid_map_outlier_filter_node.cpp |  |
| `~/output/debug_marker` | `visualization_msgs::msg::MarkerArray` | controller_node.cpp |  |
| `~/output/debug_values` | `Float32MultiArrayStamped` | accel_brake_map_calibrator_node.cpp |  |
| `~/output/detections` | `autoware_perception_msgs::msg::DetectedObjects` | radar_objects_adapter.cpp |  |
| `~/output/distance` | `autoware_internal_debug_msgs::msg::Float64Stamped` | path_distance_calculator.cpp |  |
| `~/output/doppler_pointcloud` | `PointCloud2` | radar_scan_to_pointcloud2_node.cpp |  |
| `~/output/driving_status_stamped` | `DrivingMonitorStamped` | control_performance_analysis_node.cpp |  |
| `~/output/dynamic_radar_scan` | `RadarScan` | radar_static_pointcloud_filter_node.cpp |  |
| `~/output/eagleye/pose_with_covariance` | `PoseCovStamped` | stopper_eagleye.hpp |  |
| `~/output/emergency_holding` | `tier4_system_msgs::msg::EmergencyHoldingState` | mrm_handler_core.cpp |  |
| `~/output/error_stamped` | `ErrorStamped` | control_performance_analysis_node.cpp |  |
| `~/output/estimated_steer_offset` | `Float32Stamped` | mpc_lateral_controller.cpp |  |
| `~/output/filtered_objects` | `DetectedObjects` | radar_crossing_objects_noise_filter_node.cpp |  |
| `~/output/filtered_tracks` | `RadarTracks` | radar_tracks_noise_filter_node.cpp |  |
| `~/output/gear` | `autoware_vehicle_msgs::msg::GearCommand` | mrm_handler_core.cpp |  |
| `~/output/gear_cmd` | `GearCommand` | external_cmd_selector_node.cpp |  |
| `~/output/grid_map` | `grid_map_msgs::msg::GridMap` | costmap_generator.cpp |  |
| `~/output/ground` | `Float32Array` | ground_server_core.cpp |  |
| `~/output/gyro_bias` | `Vector3Stamped` | gyro_bias_estimator.cpp |  |
| `~/output/hazard` | `autoware_vehicle_msgs::msg::HazardLightsCommand` | mrm_handler_core.cpp |  |
| `~/output/hazard_lights_cmd` | `HazardLightsCommand` | behavior_path_planner_node.cpp, external_cmd_selector_node.cpp |  |
| `~/output/heartbeat` | `OperatorHeartbeat` | external_cmd_selector_node.cpp |  |
| `~/output/height` | `Float32` | ground_server_core.cpp |  |
| `~/output/high_speed_objects` | `DetectedObjects` | object_velocity_splitter_node.cpp |  |
| `~/output/image` | `sensor_msgs::msg::Image` | node.cpp |  |
| `~/output/infrastructure_commands` | `tier4_v2x_msgs::msg::InfrastructureCommandArray` | manager.cpp |  |
| `~/output/is_reroute_available` | `RerouteAvailability` | behavior_path_planner_node.cpp |  |
| `~/output/lateral_diagnostic` | `Float32MultiArrayStamped` | mpc_lateral_controller.cpp |  |
| `~/output/line_segments_cloud` | `PointCloud2` | line_segment_detector_core.cpp |  |
| `~/output/ll2_bounding_box` | `Cloud2` | ll2_decomposer_core.cpp |  |
| `~/output/ll2_road_marking` | `Cloud2` | ll2_decomposer_core.cpp |  |
| `~/output/ll2_sign_board` | `Cloud2` | ll2_decomposer_core.cpp |  |
| `~/output/longitudinal_diagnostic` | `autoware_internal_debug_msgs::msg::Float32MultiArrayStamped` | pid_longitudinal_controller.cpp |  |
| `~/output/low_speed_objects` | `DetectedObjects` | object_velocity_splitter_node.cpp |  |
| `~/output/map_error_ratio` | `Float32Stamped` | accel_brake_map_calibrator_node.cpp |  |
| `~/output/marker` | `visualization_msgs::msg::MarkerArray` | traffic_light_recognition_marker_publisher.cpp |  |
| `~/output/markers` | `visualization_msgs::msg::MarkerArray` | planning_validator.cpp, control_validator.cpp |  |
| `~/output/mask_image` | `Image` | graph_segment_core.cpp |  |
| `~/output/max_velocity` | `VelocityLimit` | node.cpp, node.cpp |  |
| `~/output/mission_remaining_distance_time` | `MissionRemainingDistanceTime` | remaining_distance_time_calculator_node.cpp |  |
| `~/output/modified_goal` | `PoseWithUuidStamped` | behavior_path_planner_node.cpp |  |
| `~/output/mrm/comfortable_stop/status` | `tier4_system_msgs::msg::MrmBehaviorStatus` | mrm_comfortable_stop_operator_core.cpp |  |
| `~/output/mrm/emergency_stop/control_cmd` | `Control` | mrm_emergency_stop_operator_core.cpp |  |
| `~/output/mrm/emergency_stop/status` | `MrmBehaviorStatus` | mrm_emergency_stop_operator_core.cpp |  |
| `~/output/mrm/state` | `autoware_adapi_v1_msgs::msg::MrmState` | mrm_handler_core.cpp |  |
| `~/output/ndt/pointcloud` | `PointCloud2` | stopper_ndt.hpp |  |
| `~/output/noise_objects` | `DetectedObjects` | radar_crossing_objects_noise_filter_node.cpp |  |
| `~/output/noise_tracks` | `RadarTracks` | radar_tracks_noise_filter_node.cpp |  |
| `~/output/objects` | `DetectedObjects` | simple_object_merger_node.cpp, radar_object_clustering_node.cpp and 9 more |  |
| `~/output/occupancy_grid` | `nav_msgs::msg::OccupancyGrid` | costmap_generator.cpp |  |
| `~/output/occupancy_grid_map` | `OccupancyGrid` | laserscan_based_occupancy_grid_map_node.cpp, pointcloud_based_occupancy_grid_map_node.cpp and 1 more |  |
| `~/output/path` | `PathWithLaneId` | behavior_path_planner_node.cpp, node.cpp and 2 more |  |
| `~/output/pedals_cmd` | `PedalsCommand` | external_cmd_selector_node.cpp |  |
| `~/output/pointcloud` | `PointCloud2` | distortion_corrector_node.cpp, occupancy_grid_map_outlier_filter_node.cpp |  |
| `~/output/pose` | `PoseStamped` | predictor.cpp |  |
| `~/output/pose_with_covariance` | `PoseWithCovarianceStamped` | ar_tag_based_localizer.cpp, lidar_marker_localizer.cpp and 1 more |  |
| `~/output/predicted_particles` | `ParticleArray` | predictor.cpp |  |
| `~/output/predicted_trajectory` | `autoware_planning_msgs::msg::Trajectory` | autoware_pure_pursuit_lateral_controller.cpp, mpc_lateral_controller.cpp |  |
| `~/output/projected_line_segments_cloud` | `PointCloud2` | segment_filter_core.cpp |  |
| `~/output/radar` | `RadarScan` | radar_threshold_filter_node.cpp |  |
| `~/output/radar_detected_objects` | `DetectedObjects` | radar_tracks_msgs_converter_node.cpp |  |
| `~/output/radar_tracked_objects` | `TrackedObjects` | radar_tracks_msgs_converter_node.cpp |  |
| `~/output/raw_image` | `sensor_msgs::msg::Image` | image_transport_decompressor.cpp |  |
| `~/output/resized_image` | `Image` | undistort_node.cpp |  |
| `~/output/resized_info` | `CameraInfo` | undistort_node.cpp |  |
| `~/output/rois` | `TrafficLightRoiArray` | traffic_light_fine_detector_node.cpp, traffic_light_map_based_detector_node.cpp |  |
| `~/output/slope_angle` | `autoware_internal_debug_msgs::msg::Float32MultiArrayStamped` | pid_longitudinal_controller.cpp |  |
| `~/output/state_array` | `VirtualTrafficLightStateArray` | dummy_infrastructure_node.cpp |  |
| `~/output/static_radar_scan` | `RadarScan` | radar_static_pointcloud_filter_node.cpp |  |
| `~/output/steering_cmd` | `SteeringCommand` | external_cmd_selector_node.cpp |  |
| `~/output/steering_offset` | `Float32Stamped` | steer_offset_estimator_node.cpp |  |
| `~/output/steering_offset_covariance` | `Float32Stamped` | steer_offset_estimator_node.cpp |  |
| `~/output/steering_status` | `Steering` | node.cpp |  |
| `~/output/stop_reason` | `DiagnosticStatus` | node.cpp |  |
| `~/output/stop_speed_exceeded` | `StopSpeedExceeded` | planner_interface.hpp |  |
| `~/output/tracks` | `autoware_perception_msgs::msg::TrackedObjects` | radar_objects_adapter.cpp |  |
| `~/output/traffic_light` | `visualization_msgs::msg::MarkerArray` | node.cpp |  |
| `~/output/traffic_signal` | `autoware_perception_msgs::msg::TrafficLightGroup` | manager.cpp |  |
| `~/output/traffic_signals` | `NewSignalArrayType` | traffic_light_multi_camera_fusion_node.cpp, node.cpp and 2 more |  |
| `~/output/traj` | `Trajectory` | elastic_band_smoother.cpp |  |
| `~/output/trajectory` | `Trajectory` | freespace_planner_node.cpp, planning_validator.cpp and 3 more |  |
| `~/output/turn_indicators` | `autoware_vehicle_msgs::msg::TurnIndicatorsCommand` | mrm_handler_core.cpp |  |
| `~/output/turn_indicators_cmd` | `TurnIndicatorsCommand` | behavior_path_planner_node.cpp, external_cmd_selector_node.cpp |  |
| `~/output/update_raw_map` | `Float32MultiArray` | accel_brake_map_calibrator_node.cpp |  |
| `~/output/update_suggest` | `std_msgs::msg::Bool` | accel_brake_map_calibrator_node.cpp |  |
| `~/output/updated_map_error` | `Float32Stamped` | accel_brake_map_calibrator_node.cpp |  |
| `~/output/validation_status` | `PlanningValidatorStatus` | planning_validator.cpp, control_validator.cpp |  |
| `~/output/velocity_limit` | `VelocityLimit` | node.cpp, mrm_comfortable_stop_operator_core.cpp |  |
| `~/output/velocity_limit/clear` | `autoware_internal_planning_msgs::msg::VelocityLimitClearCommand` | mrm_comfortable_stop_operator_core.cpp |  |
| `~/output/velocity_limit_clear_command` | `VelocityLimitClearCommand` | node.cpp, node.cpp |  |
| `~/output/weighted_particles` | `ParticleArray` | abstract_corrector.cpp |  |
| `~/output/yabloc/image` | `Image` | stopper_yabloc.hpp |  |
| `~/planner/route` | `LaneletRoute` |  | route_selector.cpp |
| `~/planner/state` | `RouteState` |  | route_selector.cpp |
| `~/pointcloud_map` | `sensor_msgs::msg::PointCloud2` |  | map_height_fitter.cpp |
| `~/processing_time/` | `autoware_utils::ProcessingTimeDetail` | scene_module_manager_interface.cpp |  |
| `~/pub/traffic_signals` | `TrafficSignalArray` | traffic_light_arbiter.cpp |  |
| `~/route` | `LaneletRoute` | mission_planner.cpp |  |
| `~/selected_pose_type` | `std_msgs::msg::String` | pose_covariance_modifier.cpp |  |
| `~/state` | `RouteState` | mission_planner.cpp |  |
| `~/sub/external_traffic_signals` | `TrafficSignalArray` |  | traffic_light_arbiter.cpp |
| `~/sub/perception_traffic_signals` | `TrafficSignalArray` |  | traffic_light_arbiter.cpp |
| `~/sub/vector_map` | `LaneletMapBin` |  | traffic_light_arbiter.cpp |
| `~/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` |  | map_height_fitter.cpp |
| `~/virtual_wall` | `visualization_msgs::msg::MarkerArray` | debug_marker.cpp, debug_marker.cpp and 5 more |  |
| `~/virtual_wall/` | `MarkerArray` | scene_module_manager_interface.cpp |  |
| `~/virtual_wall/cruise` | `MarkerArray` | node.cpp |  |
| `~/virtual_wall/slow_down` | `MarkerArray` | node.cpp |  |
| `~/virtual_wall/stop` | `MarkerArray` | node.cpp |  |
