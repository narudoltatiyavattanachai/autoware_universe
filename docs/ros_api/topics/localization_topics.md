# Localization Topics

This document lists all topics used in the localization category.

| Topic | Message Type | Publishers | Subscribers |
| ----- | ------------ | ---------- | ----------- |
| `/marker_array` | `MarkerArray` | particle_visualize_node.cpp |  |
| `/particle_array` | `ParticleArray` |  | particle_visualize_node.cpp |
| `/points_raw` | `sensor_msgs::msg::PointCloud2` | stub_sensor_pcd_publisher.hpp |  |
| `/pose_instability_detector/input/odometry` | `Odometry` | test_message_helper_node.hpp |  |
| `/pose_instability_detector/input/twist` | `TwistWithCovarianceStamped` | test_message_helper_node.hpp |  |
| `angular_z` | `autoware_internal_debug_msgs::msg::Float32Stamped` | pose2twist_core.cpp |  |
| `debug/ellipse_marker` | `visualization_msgs::msg::Marker` | localization_error_monitor.cpp |  |
| `debug/loaded_pointcloud_map` | `sensor_msgs::msg::PointCloud2` | map_update_module.cpp |  |
| `ekf_pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped` |  | ndt_scan_matcher_core.cpp |
| `exe_time_ms` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ndt_scan_matcher_core.cpp |  |
| `gnss_pose_cov` | `PoseWithCovarianceStamped` |  | gnss_module.cpp |
| `initial_pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped` | ndt_scan_matcher_core.cpp |  |
| `initial_to_result_distance` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ndt_scan_matcher_core.cpp |  |
| `initial_to_result_distance_new` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ndt_scan_matcher_core.cpp |  |
| `initial_to_result_distance_old` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ndt_scan_matcher_core.cpp |  |
| `initial_to_result_relative_pose` | `geometry_msgs::msg::PoseStamped` | ndt_scan_matcher_core.cpp |  |
| `input/odom` | `nav_msgs::msg::Odometry` |  | localization_error_monitor.cpp |
| `input_geo_pose` | `GeoPoseWithCovariance` |  | geo_pose_projector.cpp |
| `input_gnss_pose_with_cov_topic` | `geometry_msgs::msg::PoseWithCovarianceStamped` |  | pose_covariance_modifier.cpp |
| `input_ndt_pose_with_cov_topic` | `geometry_msgs::msg::PoseWithCovarianceStamped` |  | pose_covariance_modifier.cpp |
| `iteration_num` | `autoware_internal_debug_msgs::msg::Int32Stamped` | ndt_scan_matcher_core.cpp |  |
| `linear_x` | `autoware_internal_debug_msgs::msg::Float32Stamped` | pose2twist_core.cpp |  |
| `monte_carlo_initial_pose_marker` | `visualization_msgs::msg::MarkerArray` | ndt_scan_matcher_core.cpp |  |
| `multi_initial_pose` | `geometry_msgs::msg::PoseArray` | ndt_scan_matcher_core.cpp |  |
| `multi_ndt_pose` | `geometry_msgs::msg::PoseArray` | ndt_scan_matcher_core.cpp |  |
| `ndt_marker` | `visualization_msgs::msg::MarkerArray` | ndt_scan_matcher_core.cpp |  |
| `ndt_pose` | `geometry_msgs::msg::PoseStamped` | ndt_scan_matcher_core.cpp |  |
| `ndt_pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped` | ndt_scan_matcher_core.cpp |  |
| `nearest_voxel_transformation_likelihood` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ndt_scan_matcher_core.cpp |  |
| `no_ground_nearest_voxel_transformation_likelihood` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ndt_scan_matcher_core.cpp |  |
| `no_ground_transform_probability` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ndt_scan_matcher_core.cpp |  |
| `output_pose` | `PoseWithCovariance` | geo_pose_projector.cpp |  |
| `output_pose_with_covariance_topic` | `geometry_msgs::msg::PoseWithCovarianceStamped` | pose_covariance_modifier.cpp |  |
| `points_aligned` | `sensor_msgs::msg::PointCloud2` | ndt_scan_matcher_core.cpp |  |
| `points_aligned_no_ground` | `sensor_msgs::msg::PointCloud2` | ndt_scan_matcher_core.cpp |  |
| `points_raw` | `sensor_msgs::msg::PointCloud2` |  | ndt_scan_matcher_core.cpp |
| `pose` | `geometry_msgs::msg::PoseStamped` |  | pose2twist_core.cpp |
| `pose_reset` | `PoseWithCovarianceStamped` | pose_initializer_core.cpp |  |
| `regularization_pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped` |  | ndt_scan_matcher_core.cpp |
| `stop_check_twist` | `TwistWithCovarianceStamped` |  | stop_check_module.cpp |
| `topic_name` | `T` |  | shared_data.hpp |
| `transform_probability` | `autoware_internal_debug_msgs::msg::Float32Stamped` | ndt_scan_matcher_core.cpp |  |
| `twist` | `geometry_msgs::msg::TwistStamped` | pose2twist_core.cpp |  |
| `voxel_score_points` | `sensor_msgs::msg::PointCloud2` | ndt_scan_matcher_core.cpp |  |
| `~/debug/cost_map_image` | `Image` | camera_particle_corrector_core.cpp |  |
| `~/debug/cost_map_range` | `MarkerArray` | camera_particle_corrector_core.cpp |  |
| `~/debug/debug_line_segments` | `PointCloud2` | segment_filter_core.cpp |  |
| `~/debug/detected_tag` | `PoseArray` | ar_tag_based_localizer.cpp |  |
| `~/debug/diff_pose` | `PoseStamped` | pose_instability_detector.cpp |  |
| `~/debug/gnss_position_stddev` | `std_msgs::msg::Float64` | pose_covariance_modifier.cpp |  |
| `~/debug/gnss_range_marker` | `MarkerArray` | gnss_corrector_core.cpp |  |
| `~/debug/ground_markers` | `Marker` | ground_server_core.cpp |  |
| `~/debug/ground_status` | `String` | ground_server_core.cpp |  |
| `~/debug/image` | `Image` | ar_tag_based_localizer.cpp |  |
| `~/debug/image_with_colored_line_segments` | `Image` | line_segments_overlay_core.cpp |  |
| `~/debug/image_with_line_segments` | `Image` | line_segment_detector_core.cpp |  |
| `~/debug/init_candidates` | `MarkerArray` | marker_module.cpp |  |
| `~/debug/init_marker` | `Marker` | predictor.cpp |  |
| `~/debug/lanelet2_overlay_image` | `sensor_msgs::msg::Image` | lanelet2_overlay_core.cpp |  |
| `~/debug/mapped_tag` | `MarkerArray` | ar_tag_based_localizer.cpp |  |
| `~/debug/marker_array` | `MarkerArray` | pose_estimator_arbiter_core.cpp |  |
| `~/debug/marker_detected` | `PoseArray` | lidar_marker_localizer.cpp |  |
| `~/debug/marker_mapped` | `MarkerArray` | lidar_marker_localizer.cpp |  |
| `~/debug/marker_pointcloud` | `PointCloud2` | lidar_marker_localizer.cpp |  |
| `~/debug/match_image` | `Image` | camera_particle_corrector_core.cpp |  |
| `~/debug/ndt_position_stddev` | `std_msgs::msg::Float64` | pose_covariance_modifier.cpp |  |
| `~/debug/near_cloud` | `PointCloud2` | ground_server_core.cpp |  |
| `~/debug/particles_marker_array` | `MarkerArray` | visualize.cpp |  |
| `~/debug/pose_with_covariance` | `PoseWithCovarianceStamped` | lidar_marker_localizer.cpp |  |
| `~/debug/projected_image` | `Image` | segment_filter_core.cpp |  |
| `~/debug/projected_marker` | `Marker` | lanelet2_overlay_core.cpp |  |
| `~/debug/scored_cloud` | `PointCloud2` | camera_particle_corrector_core.cpp |  |
| `~/debug/scored_post_cloud` | `PointCloud2` | camera_particle_corrector_core.cpp |  |
| `~/debug/segmented_image` | `Image` | graph_segment_core.cpp |  |
| `~/debug/sign_board_marker` | `MarkerArray` | ll2_decomposer_core.cpp |  |
| `~/debug/state_string` | `String` | camera_particle_corrector_core.cpp |  |
| `~/debug/string` | `String` | pose_estimator_arbiter_core.cpp |  |
| `~/input/artag/image` | `Image` |  | pose_estimator_arbiter_core.cpp |
| `~/input/eagleye/pose_with_covariance` | `PoseCovStamped` |  | pose_estimator_arbiter_core.cpp |
| `~/input/ekf_pose` | `PoseWithCovarianceStamped` |  | ar_tag_based_localizer.cpp, lidar_marker_localizer.cpp and 1 more |
| `~/input/ground` | `Float32Array` |  | lanelet2_overlay_core.cpp |
| `~/input/height` | `std_msgs::msg::Float32` |  | predictor.cpp, gnss_corrector_core.cpp |
| `~/input/image_raw` | `Image` |  | camera_pose_initializer_core.cpp, line_segments_overlay_core.cpp and 4 more |
| `~/input/image_raw/compressed` | `CompressedImage` |  | undistort_node.cpp |
| `~/input/initialization_state` | `InitializationState` |  | pose_estimator_arbiter_core.cpp |
| `~/input/initialpose` | `PoseCovStamped` |  | predictor.cpp |
| `~/input/line_segments` | `PointCloud2` |  | line_segments_overlay_core.cpp |
| `~/input/line_segments_cloud` | `PointCloud2` |  | camera_particle_corrector_core.cpp |
| `~/input/ll2_bounding_box` | `PointCloud2` |  | camera_particle_corrector_core.cpp |
| `~/input/ll2_road_marking` | `PointCloud2` |  | camera_particle_corrector_core.cpp, lanelet2_overlay_core.cpp |
| `~/input/ll2_sign_board` | `PointCloud2` |  | lanelet2_overlay_core.cpp |
| `~/input/ndt/pointcloud` | `PointCloud2` |  | pose_estimator_arbiter_core.cpp |
| `~/input/pointcloud_map` | `PointCloud2` |  | pose_estimator_arbiter_core.cpp |
| `~/input/pose` | `PoseStamped` |  | camera_particle_corrector_core.cpp, ground_server_core.cpp and 1 more |
| `~/input/pose_with_covariance` | `PoseCovStamped` |  | pose_estimator_arbiter_core.cpp, gnss_corrector_core.cpp |
| `~/input/predicted_particles` | `ParticleArray` |  | abstract_corrector.cpp |
| `~/input/projected_line_segments_cloud` | `PointCloud2` |  | lanelet2_overlay_core.cpp |
| `~/input/twist_with_covariance` | `TwistCovStamped` |  | predictor.cpp |
| `~/input/weighted_particles` | `ParticleArray` |  | predictor.cpp |
| `~/input/yabloc/image` | `Image` |  | pose_estimator_arbiter_core.cpp |
| `~/input/yabloc_pose` | `PoseStamped` |  | availability_module.cpp |
| `~/output/artag/image` | `Image` | stopper_artag.hpp |  |
| `~/output/eagleye/pose_with_covariance` | `PoseCovStamped` | stopper_eagleye.hpp |  |
| `~/output/ground` | `Float32Array` | ground_server_core.cpp |  |
| `~/output/height` | `Float32` | ground_server_core.cpp |  |
| `~/output/line_segments_cloud` | `PointCloud2` | line_segment_detector_core.cpp |  |
| `~/output/ll2_bounding_box` | `Cloud2` | ll2_decomposer_core.cpp |  |
| `~/output/ll2_road_marking` | `Cloud2` | ll2_decomposer_core.cpp |  |
| `~/output/ll2_sign_board` | `Cloud2` | ll2_decomposer_core.cpp |  |
| `~/output/mask_image` | `Image` | graph_segment_core.cpp |  |
| `~/output/ndt/pointcloud` | `PointCloud2` | stopper_ndt.hpp |  |
| `~/output/pose` | `PoseStamped` | predictor.cpp |  |
| `~/output/pose_with_covariance` | `PoseWithCovarianceStamped` | ar_tag_based_localizer.cpp, lidar_marker_localizer.cpp and 1 more |  |
| `~/output/predicted_particles` | `ParticleArray` | predictor.cpp |  |
| `~/output/projected_line_segments_cloud` | `PointCloud2` | segment_filter_core.cpp |  |
| `~/output/resized_image` | `Image` | undistort_node.cpp |  |
| `~/output/resized_info` | `CameraInfo` | undistort_node.cpp |  |
| `~/output/weighted_particles` | `ParticleArray` | abstract_corrector.cpp |  |
| `~/output/yabloc/image` | `Image` | stopper_yabloc.hpp |  |
| `~/selected_pose_type` | `std_msgs::msg::String` | pose_covariance_modifier.cpp |  |
