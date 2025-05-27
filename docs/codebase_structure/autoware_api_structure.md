# Autoware Universe Codebase Structure

This document provides a comprehensive tree-view of the Autoware Universe codebase structure, showing:
- Main source files
- ROS 2 nodes, services, and topic publishers/subscribers
- Key public functions and class interfaces

## Repository Structure

The Autoware Universe repository is organized into several key directories:

- `common/`: Shared utilities and interfaces
- `control/`: Vehicle control-related packages
- `localization/`: Localization algorithms and nodes
- `perception/`: Perception modules (object detection, tracking, etc.)
- `planning/`: Path planning and behavioral planning modules
- `sensing/`: Sensor drivers and interfaces
- `system/`: System monitoring and management
- `launch/`: Launch files and configurations

## Detailed API Structure

```
control/
├── autoware_autonomous_emergency_braking/
│   ├── include/autoware/autonomous_emergency_braking/node.hpp → defines `AEB`
│   └── launch/ → `autoware_autonomous_emergency_braking.launch.xml`
├── autoware_collision_detector/
│   ├── include/autoware/collision_detector/node.hpp → defines `CollisionDetectorNode`
│   └── launch/ → `collision_detector.launch.xml`
├── autoware_control_performance_analysis/
│   ├── include/autoware/control_performance_analysis/control_performance_analysis_node.hpp → defines `ControlPerformanceAnalysisNode`
│   └── launch/ → `control_performance_analysis.launch.xml`
├── autoware_control_validator/
│   ├── include/autoware/control_validator/control_validator.hpp → defines `ControlValidator`
│   └── launch/ → `control_validator.launch.xml`
├── autoware_external_cmd_selector/
│   ├── include/autoware/external_cmd_selector/external_cmd_selector_node.hpp → defines `ExternalCmdSelector`
│   └── launch/ → `external_cmd_selector.launch.py`
localization/
├── autoware_geo_pose_projector/
│   └── launch/ → `geo_pose_projector.launch.xml`
├── autoware_landmark_based_localizer/
├── autoware_localization_error_monitor/
│   └── launch/ → `localization_error_monitor.launch.xml`
├── autoware_ndt_scan_matcher/
│   ├── include/autoware/ndt_scan_matcher/ndt_scan_matcher_core.hpp → defines `NDTScanMatcher`
│   ├── include/autoware/ndt_scan_matcher/map_update_module.hpp → `out_of_map_range()` function
│   ├── include/autoware/ndt_scan_matcher/hyper_parameters.hpp → `HyperParameters()` function
│   └── launch/ → `ndt_scan_matcher.launch.xml`
├── autoware_pose2twist/
│   └── launch/ → `pose2twist.launch.xml`
perception/
├── autoware_bevfusion/
│   ├── include/autoware/bevfusion/bevfusion_node.hpp → `BEVFusionNode()` function
│   ├── include/autoware/bevfusion/bevfusion_config.hpp → `by()` function
│   └── launch/ → `bevfusion.launch.xml`
├── autoware_bytetrack/
│   ├── include/autoware/bytetrack/bytetrack_visualizer_node.hpp → defines `ByteTrackVisualizerNode`
│   ├── include/autoware/bytetrack/bytetrack_node.hpp → defines `ByteTrackNode`
│   └── launch/ → `bytetrack.launch.xml`
├── autoware_cluster_merger/
│   └── launch/ → `cluster_merger.launch.xml`
├── autoware_compare_map_segmentation/
│   ├── include/autoware/compare_map_segmentation/voxel_grid_map_loader.hpp → `set_voxel_grid()` function
│   └── launch/ → `voxel_distance_based_compare_map_filter.launch.xml`
├── autoware_crosswalk_traffic_light_estimator/
│   ├── include/autoware_crosswalk_traffic_light_estimator/node.hpp → defines `CrosswalkTrafficLightEstimatorNode`
│   └── launch/ → `crosswalk_traffic_light_estimator.launch.xml`
planning/
├── autoware_costmap_generator/
│   ├── include/autoware/costmap_generator/costmap_generator.hpp → defines `CostmapGenerator`
│   ├── include/autoware/costmap_generator/utils/points_to_costmap.hpp → `makeCostmapFromPoints()` function
│   ├── include/autoware/costmap_generator/utils/objects_to_costmap.hpp → `makeCostmapFromObjects()` function
│   └── launch/ → `costmap_generator.launch.xml`
├── autoware_external_velocity_limit_selector/
│   ├── include/autoware/external_velocity_limit_selector/external_velocity_limit_selector_node.hpp → defines `ExternalVelocityLimitSelectorNode`
│   └── launch/ → `external_velocity_limit_selector.launch.xml`
├── autoware_freespace_planner/
│   ├── include/autoware/freespace_planner/freespace_planner_node.hpp → defines `FreespacePlannerNode`
│   └── launch/ → `freespace_planner.launch.xml`
├── autoware_freespace_planning_algorithms/
│   ├── include/autoware/freespace_planning_algorithms/rrtstar.hpp → `makePlan()` function
│   ├── include/autoware/freespace_planning_algorithms/astar_search.hpp → `setMap()` function
├── autoware_mission_planner_universe/
│   ├── include/autoware/mission_planner_universe/mission_planner_plugin.hpp → `initialize()` function
│   └── launch/ → `mission_planner.launch.xml`
sensing/
├── autoware_cuda_pointcloud_preprocessor/
│   ├── include/autoware/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor_node.hpp → defines `CudaPointcloudPreprocessorNode`
│   ├── include/autoware/cuda_pointcloud_preprocessor/cuda_downsample_filter/cuda_voxel_grid_downsample_filter_node.hpp → defines `CudaVoxelGridDownsampleFilterNode`
│   ├── include/autoware/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.hpp → `setCropBoxParameters()` function
│   ├── include/autoware/cuda_pointcloud_preprocessor/cuda_concatenate_data/cuda_combine_cloud_handler.hpp → `allocate_pointclouds()` function
│   └── launch/ → `cuda_voxel_grid_downsample_filter.launch.xml`
├── autoware_cuda_utils/
├── autoware_image_diagnostics/
│   └── launch/ → `image_diagnostics_node.launch.xml`
├── autoware_image_transport_decompressor/
│   ├── include/autoware/image_transport_decompressor/image_transport_decompressor.hpp → defines `ImageTransportDecompressor`
│   └── launch/ → `image_transport_decompressor.launch.xml`
├── autoware_imu_corrector/
│   └── launch/ → `imu_corrector.launch.xml`
system/
├── autoware_bluetooth_monitor/
│   ├── include/autoware/bluetooth_monitor/bluetooth_monitor.hpp → defines `BluetoothMonitor`
│   ├── include/autoware/bluetooth_monitor/service/l2ping.hpp → `run()` function
│   └── launch/ → `bluetooth_monitor.launch.xml`
├── autoware_component_monitor/
│   └── launch/ → `component_monitor.launch.xml`
├── autoware_component_state_monitor/
│   └── launch/ → `component_state_monitor.launch.py`
├── autoware_default_adapi/
│   └── launch/ → `test_default_adapi.launch.xml`
├── autoware_default_adapi_helpers/
common/
├── autoware_adapi_specs/
├── autoware_agnocast_wrapper/
├── autoware_auto_common/
│   ├── include/autoware_auto_common/helper_functions/byte_reader.hpp → `ByteReader()` function
├── autoware_boundary_departure_checker/
├── autoware_component_interface_specs_universe/
evaluator/
├── autoware_control_evaluator/
│   ├── include/autoware/control_evaluator/control_evaluator_node.hpp → defines `ControlEvaluatorNode`
│   └── launch/ → `control_evaluator.launch.xml`
├── autoware_kinematic_evaluator/
│   ├── include/autoware/kinematic_evaluator/kinematic_evaluator_node.hpp → defines `KinematicEvaluatorNode`
│   └── launch/ → `kinematic_evaluator.launch.xml`
├── autoware_localization_evaluator/
│   ├── include/autoware/localization_evaluator/localization_evaluator_node.hpp → defines `LocalizationEvaluatorNode`
│   └── launch/ → `localization_evaluator.launch.xml`
├── autoware_perception_online_evaluator/
│   ├── include/autoware/perception_online_evaluator/perception_online_evaluator_node.hpp → defines `PerceptionOnlineEvaluatorNode`
│   └── launch/ → `perception_online_evaluator.launch.xml`
├── autoware_planning_evaluator/
│   ├── include/autoware/planning_evaluator/motion_evaluator_node.hpp → defines `MotionEvaluatorNode`
│   ├── include/autoware/planning_evaluator/planning_evaluator_node.hpp → defines `PlanningEvaluatorNode`
│   ├── include/autoware/planning_evaluator/metrics_accumulator.hpp → `accumulate()` function
│   ├── include/autoware/planning_evaluator/metric_accumulators/blinker_accumulator.hpp → `update()` function
│   └── launch/ → `planning_evaluator.launch.xml`
```

## How to Use This Document

This document provides a high-level overview of the Autoware Universe codebase structure. It can be used to:
1. Understand the overall organization of the codebase
2. Identify key ROS 2 nodes and their relationships
3. Find important functions and interfaces for each module

For more detailed information, please refer to the individual package documentation and source code.
