# Common Topics

This document lists all topics used in the common category.

| Topic | Message Type | Publishers | Subscribers |
| ----- | ------------ | ---------- | ----------- |
| `/api/autoware/get/emergency` | `tier4_external_api_msgs::msg::Emergency` |  | autoware_state_panel.cpp |
| `/api/fail_safe/mrm_state` | `MRMState` |  | autoware_state_panel.cpp |
| `/api/localization/initialization_state` | `LocalizationInitializationState` |  | autoware_state_panel.cpp |
| `/api/motion/state` | `MotionState` |  | autoware_state_panel.cpp |
| `/api/operation_mode/state` | `OperationModeState` |  | autoware_state_panel.cpp, operation_mode_debug_panel.cpp |
| `/api/routing/state` | `RouteState` |  | autoware_state_panel.cpp |
| `/input_topic` | `Int32` | test_fake_test_node.cpp | test_fake_test_node.cpp |
| `/map/vector_map` | `LaneletMapBin` |  | traffic_light_publish_panel.cpp |
| `/output_topic` | `Bool` | test_fake_test_node.cpp | test_fake_test_node.cpp |
| `/perception/traffic_light_recognition/traffic_signals` | `TrafficLightGroupArray` | traffic_light_publish_panel.cpp |  |
| `/planning/scenario_planning/max_velocity_default` | `autoware_internal_planning_msgs::msg::VelocityLimit` | autoware_state_panel.cpp |  |
| `Removed_polygon` | `visualization_msgs::msg::Marker` | polygon_remover.cpp |  |
| `blockage_diag/debug/ground_blockage_ratio` | `autoware_internal_debug_msgs::msg::Float32Stamped` | blockage_diag_node.cpp |  |
| `blockage_diag/debug/ground_dust_ratio` | `autoware_internal_debug_msgs::msg::Float32Stamped` | blockage_diag_node.cpp |  |
| `blockage_diag/debug/sky_blockage_ratio` | `autoware_internal_debug_msgs::msg::Float32Stamped` | blockage_diag_node.cpp |  |
| `debug/ring_outlier_filter` | `PointCloud2` | ring_outlier_filter_node.cpp |  |
| `dual_return_outlier_filter/debug/pointcloud_noise` | `sensor_msgs::msg::PointCloud2` | dual_return_outlier_filter_node.cpp |  |
| `dual_return_outlier_filter/debug/visibility` | `autoware_internal_debug_msgs::msg::Float32Stamped` | dual_return_outlier_filter_node.cpp |  |
| `image_diag/image_state_diag` | `autoware_internal_debug_msgs::msg::Int32Stamped` | image_diagnostics_node.cpp |  |
| `in/heartbeat` | `ManualOperatorHeartbeat` |  | node.cpp |
| `in/pedals_cmd` | `PedalsCommand` |  | node.cpp |
| `input` | `sensor_msgs::msg::Imu` |  | imu_corrector_core.cpp, livox_tag_filter_node.cpp and 6 more |
| `input/pointcloud` | `PointCloud2` |  | lanelet2_map_filter_node.cpp, node.cpp |
| `input/raw_image` | `sensor_msgs::msg::Image` |  | image_diagnostics_node.cpp |
| `input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` |  | lanelet2_map_filter_node.cpp, vector_map_inside_area_filter_node.cpp and 3 more |
| `out/control_cmd` | `Control` | node.cpp |  |
| `output` | `sensor_msgs::msg::Imu` | imu_corrector_core.cpp, livox_tag_filter_node.cpp and 12 more |  |
| `pointcloud_map` | `sensor_msgs::msg::PointCloud2` |  | pcd_map_tf_generator_node.cpp |
| `processing_time` | `autoware::universe_utils::ProcessingTimeDetail` | example_time_keeper.cpp, costmap_generator.cpp |  |
| `ring_outlier_filter/debug/visibility` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ring_outlier_filter_node.cpp |  |
| `topic` | `std_msgs::msg::String` | example_polling_subscriber.cpp |  |
| `vector_map` | `autoware_map_msgs::msg::LaneletMapBin` |  | vector_map_tf_generator_node.cpp |
| `~/crop_box_polygon` | `geometry_msgs::msg::PolygonStamped` | crop_box_filter_node.cpp |  |
| `~/doors/status` | `DoorStatusArray` | dummy_doors.cpp |  |
| `~/input/compressed_image` | `sensor_msgs::msg::CompressedImage` |  | image_transport_decompressor.cpp |
| `~/input/imu_raw` | `Imu` |  | gyro_bias_estimator.cpp |
| `~/input/lanelet2_map` | `autoware_map_msgs::msg::LaneletMapBin` |  | traffic_light_recognition_marker_publisher.cpp, ar_tag_based_localizer.cpp and 1 more |
| `~/input/odom` | `Odometry` |  | gyro_bias_estimator.cpp, time_synchronizer_node.cpp |
| `~/input/pointcloud` | `sensor_msgs::msg::PointCloud2` | test.cpp | cuda_pointcloud_preprocessor_node.cpp, distortion_corrector_node.cpp and 3 more |
| `~/input/radar` | `RadarScan` |  | radar_scan_to_pointcloud2_node.cpp, radar_threshold_filter_node.cpp |
| `~/input/radar_info` | `autoware_sensing_msgs::msg::RadarInfo` |  | radar_objects_adapter.cpp |
| `~/input/simulation_events` | `SimulationEvents` |  | fault_injection_node.cpp |
| `~/input/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` |  | traffic_light_recognition_marker_publisher.cpp |
| `~/output/amplitude_pointcloud` | `PointCloud2` | radar_scan_to_pointcloud2_node.cpp |  |
| `~/output/detections` | `autoware_perception_msgs::msg::DetectedObjects` | radar_objects_adapter.cpp |  |
| `~/output/doppler_pointcloud` | `PointCloud2` | radar_scan_to_pointcloud2_node.cpp |  |
| `~/output/dynamic_radar_scan` | `RadarScan` | radar_static_pointcloud_filter_node.cpp |  |
| `~/output/gyro_bias` | `Vector3Stamped` | gyro_bias_estimator.cpp |  |
| `~/output/marker` | `visualization_msgs::msg::MarkerArray` | traffic_light_recognition_marker_publisher.cpp |  |
| `~/output/pointcloud` | `PointCloud2` | distortion_corrector_node.cpp, occupancy_grid_map_outlier_filter_node.cpp |  |
| `~/output/radar` | `RadarScan` | radar_threshold_filter_node.cpp |  |
| `~/output/raw_image` | `sensor_msgs::msg::Image` | image_transport_decompressor.cpp |  |
| `~/output/static_radar_scan` | `RadarScan` | radar_static_pointcloud_filter_node.cpp |  |
| `~/output/tracks` | `autoware_perception_msgs::msg::TrackedObjects` | radar_objects_adapter.cpp |  |
| `~/pointcloud_map` | `sensor_msgs::msg::PointCloud2` |  | map_height_fitter.cpp |
| `~/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` |  | map_height_fitter.cpp |
