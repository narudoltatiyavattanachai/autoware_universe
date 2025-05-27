# Perception Topics

This document lists all topics used in the perception category.

| Topic | Message Type | Publishers | Subscribers |
| ----- | ------------ | ---------- | ----------- |
| `/api/autoware/get/map/info/hash` | `tier4_external_api_msgs::msg::MapHash` |  | elevation_map_loader_node.cpp |
| `/diagnostics` | `diagnostic_msgs::msg::DiagnosticArray` | diagnostics_interface.cpp, pose_instability_detector.cpp and 3 more | fusion_node.cpp, test_diagnostics_interface.cpp and 17 more |
| `/test_ransac_ground_filter/input_cloud` | `sensor_msgs::msg::PointCloud2` | test_ransac_ground_filter.cpp |  |
| `/test_ransac_ground_filter/output_cloud` | `sensor_msgs::msg::PointCloud2` | test_ransac_ground_filter.cpp |  |
| `/test_ray_ground_filter/input_cloud` | `sensor_msgs::msg::PointCloud2` | test_ray_ground_filter.cpp |  |
| `/test_ray_ground_filter/output_cloud` | `sensor_msgs::msg::PointCloud2` | test_ray_ground_filter.cpp |  |
| `/vector_map` | `LaneletMapBin` |  | radar_object_tracker_node.cpp, map_based_prediction_node.cpp |
| `debug/clusters` | `sensor_msgs::msg::PointCloud2` | euclidean_cluster_node.cpp, voxel_grid_based_euclidean_cluster_node.cpp and 1 more |  |
| `debug/divided_objects` | `autoware_perception_msgs::msg::DetectedObjects` | debugger.hpp |  |
| `debug/downsampled_map/pointcloud` | `sensor_msgs::msg::PointCloud2` | voxel_grid_map_loader.cpp |  |
| `debug/ground/pointcloud` | `sensor_msgs::msg::PointCloud2` | node.cpp |  |
| `debug/initial_objects` | `autoware_perception_msgs::msg::DetectedObjects` | debugger.hpp |  |
| `debug/instance_pointcloud` | `sensor_msgs::msg::PointCloud2` | debugger.cpp |  |
| `debug/interpolated_sub_object` | `TrackedObjects` | decorative_tracker_merger_node.cpp |  |
| `debug/merged_objects` | `autoware_perception_msgs::msg::DetectedObjects` | debugger.hpp |  |
| `debug/plane_pose_array` | `geometry_msgs::msg::PoseArray` | node.cpp |  |
| `debug/tracked_objects` | `autoware_perception_msgs::msg::DetectedObjects` | debugger.hpp |  |
| `input/elevation_map` | `grid_map_msgs::msg::GridMap` |  | node.cpp |
| `input/main_object` | `TrackedObjects` |  | decorative_tracker_merger_node.cpp |
| `input/object` | `autoware_perception_msgs::msg::DetectedObjects` |  | object_range_splitter_node.cpp, position_filter.cpp and 2 more |
| `input/objects` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` |  | low_intensity_cluster_filter_node.cpp |
| `input/pointcloud_map` | `sensor_msgs::msg::PointCloud2` |  | elevation_map_loader_node.cpp |
| `input/pointcloud_map_metadata` | `autoware_map_msgs::msg::PointCloudMapMetaData` |  | elevation_map_loader_node.cpp |
| `input/sub_object` | `TrackedObjects` |  | decorative_tracker_merger_node.cpp |
| `kinematic_state` | `nav_msgs::msg::Odometry` |  | voxel_grid_map_loader.cpp |
| `maneuver` | `visualization_msgs::msg::MarkerArray` | map_based_prediction_node.cpp |  |
| `map` | `sensor_msgs::msg::PointCloud2` |  | voxel_grid_map_loader.cpp |
| `objects` | `DetectedObjectsWithFeature` | shape_estimation_node.cpp |  |
| `output/clusters` | `DetectedObjectsWithFeature` | cluster_merger_node.cpp |  |
| `output/dynamic_object` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | node.cpp |  |
| `output/elevation_map` | `grid_map_msgs::msg::GridMap` | elevation_map_loader_node.cpp |  |
| `output/elevation_map_cloud` | `sensor_msgs::msg::PointCloud2` | elevation_map_loader_node.cpp |  |
| `output/labeled_clusters` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | node.cpp |  |
| `output/long_range_object` | `autoware_perception_msgs::msg::DetectedObjects` | object_range_splitter_node.cpp |  |
| `output/object` | `autoware_perception_msgs::msg::DetectedObjects` | object_association_merger_node.cpp, decorative_tracker_merger_node.cpp and 2 more |  |
| `output/objects` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | low_intensity_cluster_filter_node.cpp, topic_publisher.cpp |  |
| `output/points_raw` | `sensor_msgs::msg::PointCloud2` | node.cpp |  |
| `output/short_range_object` | `autoware_perception_msgs::msg::DetectedObjects` | object_range_splitter_node.cpp |  |
| `output/traffic_rois` | `TrafficLightRoiArray` | traffic_light_selector_node.cpp |  |
| `output/traffic_signals` | `TrafficLightArray` | traffic_light_category_merger_node.cpp |  |
| `~/debug/exe_time_ms` | `autoware_internal_debug_msgs::msg::Float32Stamped` | traffic_light_fine_detector_node.cpp |  |
| `~/debug/low_confidence_objects` | `DetectedObjects` | node.cpp |  |
| `~/debug/marker` | `visualization_msgs::msg::MarkerArray` | lanelet_filter.cpp, debug_marker.cpp and 7 more |  |
| `~/debug/markers` | `visualization_msgs::msg::MarkerArray` | traffic_light_map_based_detector_node.cpp, autoware_pure_pursuit_lateral_controller.cpp and 1 more |  |
| `~/debug/neighbor_pointcloud` | `sensor_msgs::msg::PointCloud2` | debugger.hpp |  |
| `~/debug/objects_markers` | `visualization_msgs::msg::MarkerArray` | debugger.cpp |  |
| `~/debug/painted_pointcloud` | `PointCloudMsgType` | node.cpp |  |
| `~/debug/pointcloud_within_polygon` | `sensor_msgs::msg::PointCloud2` | debugger.hpp |  |
| `~/debug/processing_time_detail_ms` | `autoware_utils::ProcessingTimeDetail` | multi_object_tracker_node.cpp, laserscan_based_occupancy_grid_map_node.cpp and 10 more |  |
| `~/debug/removed_objects` | `autoware_perception_msgs::msg::DetectedObjects` | debugger.hpp |  |
| `~/debug/single_frame_map` | `nav_msgs::msg::OccupancyGrid` | synchronized_grid_map_fusion_node.cpp |  |
| `~/debug/tentative_objects` | `autoware_perception_msgs::msg::TrackedObjects` | debugger.cpp |  |
| `~/expect/rois` | `tier4_perception_msgs::msg::TrafficLightRoiArray` | traffic_light_map_based_detector_node.cpp |  |
| `~/in/rect` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` |  | bytetrack_node.cpp |
| `~/input` | `DetectedObjectsWithFeature` |  | detected_object_feature_remover_node.cpp |
| `~/input/camera_info` | `sensor_msgs::msg::CameraInfo` |  | traffic_light_map_based_detector_node.cpp, bevfusion_node.cpp and 4 more |
| `~/input/classified/traffic_signals` | `TrafficSignalArray` |  | node.cpp |
| `~/input/image` | `sensor_msgs::msg::Image` |  | bevfusion_node.cpp, ar_tag_based_localizer.cpp |
| `~/input/initial_objects` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` |  | detection_by_tracker_node.cpp |
| `~/input/objects` | `PredictedObjects` | test.cpp | perception_online_evaluator_node.cpp, radar_objects_adapter.cpp and 6 more |
| `~/input/obstacle_pointcloud` | `PointCloud2` |  | pointcloud_based_occupancy_grid_map_node.cpp |
| `~/input/odometry` | `Odometry` |  | radar_tracks_msgs_converter_node.cpp, pose_instability_detector.cpp and 5 more |
| `~/input/radar_objects` | `RadarTracks` |  | radar_tracks_msgs_converter_node.cpp |
| `~/input/raw_pointcloud` | `PointCloud2` |  | pointcloud_based_occupancy_grid_map_node.cpp |
| `~/input/route` | `LaneletRoute` |  | node.cpp, traffic_light_map_based_detector_node.cpp and 2 more |
| `~/input/tl_state` | `autoware_perception_msgs::msg::TrafficLightGroupArray` |  | node.cpp |
| `~/input/tracked_objects` | `autoware_perception_msgs::msg::TrackedObjects` |  | detection_by_tracker_node.cpp |
| `~/input/tracks` | `RadarTracks` |  | radar_tracks_noise_filter_node.cpp |
| `~/input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` |  | traffic_light_multi_camera_fusion_node.cpp, node.cpp and 10 more |
| `~/markers` | `MarkerArray` | perception_online_evaluator_node.cpp |  |
| `~/out/objects` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | tensorrt_yolox_node.cpp, bytetrack_node.cpp |  |
| `~/out/objects/debug/uuid` | `tier4_perception_msgs::msg::DynamicObjectArray` | bytetrack_node.cpp |  |
| `~/output` | `autoware_perception_msgs::msg::DetectedObjects` | detection_by_tracker_node.cpp, detected_object_feature_remover_node.cpp |  |
| `~/output/debug/ground_truth_objects` | `autoware_perception_msgs::msg::TrackedObjects` | node.cpp |  |
| `~/output/debug/high_confidence/pointcloud` | `PointCloud2` | occupancy_grid_map_outlier_filter_node.cpp |  |
| `~/output/debug/low_confidence/pointcloud` | `PointCloud2` | occupancy_grid_map_outlier_filter_node.cpp |  |
| `~/output/debug/outlier/pointcloud` | `PointCloud2` | occupancy_grid_map_outlier_filter_node.cpp |  |
| `~/output/filtered_objects` | `DetectedObjects` | radar_crossing_objects_noise_filter_node.cpp |  |
| `~/output/filtered_tracks` | `RadarTracks` | radar_tracks_noise_filter_node.cpp |  |
| `~/output/high_speed_objects` | `DetectedObjects` | object_velocity_splitter_node.cpp |  |
| `~/output/image` | `sensor_msgs::msg::Image` | node.cpp |  |
| `~/output/low_speed_objects` | `DetectedObjects` | object_velocity_splitter_node.cpp |  |
| `~/output/noise_objects` | `DetectedObjects` | radar_crossing_objects_noise_filter_node.cpp |  |
| `~/output/noise_tracks` | `RadarTracks` | radar_tracks_noise_filter_node.cpp |  |
| `~/output/objects` | `DetectedObjects` | simple_object_merger_node.cpp, radar_object_clustering_node.cpp and 9 more |  |
| `~/output/occupancy_grid_map` | `OccupancyGrid` | laserscan_based_occupancy_grid_map_node.cpp, pointcloud_based_occupancy_grid_map_node.cpp and 1 more |  |
| `~/output/radar_detected_objects` | `DetectedObjects` | radar_tracks_msgs_converter_node.cpp |  |
| `~/output/radar_tracked_objects` | `TrackedObjects` | radar_tracks_msgs_converter_node.cpp |  |
| `~/output/rois` | `TrafficLightRoiArray` | traffic_light_fine_detector_node.cpp, traffic_light_map_based_detector_node.cpp |  |
| `~/output/traffic_light` | `visualization_msgs::msg::MarkerArray` | node.cpp |  |
| `~/output/traffic_signals` | `NewSignalArrayType` | traffic_light_multi_camera_fusion_node.cpp, node.cpp and 2 more |  |
| `~/pub/traffic_signals` | `TrafficSignalArray` | traffic_light_arbiter.cpp |  |
| `~/sub/external_traffic_signals` | `TrafficSignalArray` |  | traffic_light_arbiter.cpp |
| `~/sub/perception_traffic_signals` | `TrafficSignalArray` |  | traffic_light_arbiter.cpp |
| `~/sub/vector_map` | `LaneletMapBin` |  | traffic_light_arbiter.cpp |
