# Autoware Universe Codebase Structure

This document provides a tree-view of the codebase APIs and functional structure, showing the main ROS 2 nodes, services, topic publishers/subscribers, and key public functions for each major component.

## Main Structure Overview

```
src/planning/
├── behavior_path_planner/
│   ├── src/scene_module/lane_change/lane_change_module.cpp → defines `LaneChangeModule`, publishes to `/lane_change_path`
│   ├── include/behavior_path_planner/path_utilities.hpp → `generatePathWithLaneId()` function
│   └── launch/ → `behavior_path_planner.launch.xml`

src/perception/
├── autoware_image_projection_based_fusion/
│   ├── include/autoware/image_projection_based_fusion/roi_detected_object_fusion/node.hpp → defines `RoiDetectedObjectFusionNode`, subscribes to `/detected_objects`
│   ├── src/roi_detected_object_fusion/node.cpp → `fuse_on_single_image()` function, processes object and ROI data
│   └── launch/ → `roi_detected_object_fusion.launch.xml`

src/control/
├── autoware_autonomous_emergency_braking/
│   ├── include/autoware/autonomous_emergency_braking/node.hpp → defines `AEB`, subscribes to `/dynamic_objects`
│   ├── src/node.cpp → `appendPointToPolygon()` function, publishes to `/emergency`
│   └── launch/ → `autoware_autonomous_emergency_braking.launch.xml`
```

## Complete Tree View

```
planning/
├── autoware_freespace_planner/
│   ├── include/autoware/freespace_planner/freespace_planner_node.hpp → defines `FreespacePlannerNode`, subscribes to `~/input/occupancy_grid`
│   ├── src/autoware_freespace_planner/freespace_planner_node.cpp → `onOccupancyGrid()` function, publishes to `~/output/trajectory`
│   └── launch/ → `freespace_planner.launch.xml`
├── autoware_mission_planner_universe/
│   ├── include/autoware/mission_planner/mission_planner.hpp → defines `MissionPlanner`, subscribes to `/map/vector_map`
│   ├── src/mission_planner/mission_planner.cpp → `planRoute()` function, publishes to `~/output/route`
│   └── launch/ → `mission_planner.launch.xml`
├── behavior_path_planner/
│   ├── include/behavior_path_planner/behavior_path_planner_node.hpp → defines `BehaviorPathPlannerNode`
│   ├── src/scene_module/lane_change/lane_change_module.cpp → `generatePathForLaneChange()` function
│   └── launch/ → `behavior_path_planner.launch.xml`

perception/
├── autoware_tensorrt_yolox/
│   ├── include/autoware/tensorrt_yolox/tensorrt_yolox_node.hpp → defines `TrtYoloXNode`, subscribes to image topics
│   ├── src/tensorrt_yolox_node.cpp → `onImage()` function, publishes to `~/out/objects`
│   └── launch/ → `yolox.launch.xml`
├── autoware_image_projection_based_fusion/
│   ├── include/autoware/image_projection_based_fusion/roi_detected_object_fusion/node.hpp → defines `RoiDetectedObjectFusionNode`
│   ├── src/roi_detected_object_fusion/node.cpp → `fuse_on_single_image()` function
│   └── launch/ → `roi_detected_object_fusion.launch.xml`
├── autoware_object_range_splitter/
│   ├── src/object_range_splitter_node.hpp → defines `ObjectRangeSplitterNode`, subscribes to detected objects
│   ├── src/object_range_splitter_node.cpp → `objectCallback()` function, publishes to long/short range objects
│   └── launch/ → `object_range_splitter.launch.xml`

control/
├── autoware_autonomous_emergency_braking/
│   ├── include/autoware/autonomous_emergency_braking/node.hpp → defines `AEB`, processes collision data
│   ├── src/node.cpp → `publishEmergencyStopSignal()` function
│   └── launch/ → `autoware_autonomous_emergency_braking.launch.xml`
├── autoware_control_validator/
│   ├── include/autoware/control_validator/control_validator.hpp → defines `ControlValidator`
│   ├── src/control_validator.cpp → `checkLongitudinal()` function
│   └── launch/ → `control_validator.launch.xml`

localization/
├── autoware_ndt_scan_matcher/
│   ├── include/autoware/ndt_scan_matcher/ndt_scan_matcher_core.hpp → defines `NDTScanMatcher`
│   ├── src/ndt_scan_matcher_core.cpp → `matchPointcloud()` function
│   └── launch/ → `ndt_scan_matcher.launch.xml`
├── autoware_pose2twist/
│   ├── include/autoware/pose2twist/pose2twist_node.hpp → defines `Pose2TwistNode`
│   ├── src/pose2twist_node.cpp → `callbackPose()` function, converts poses to velocity
│   └── launch/ → `pose2twist.launch.xml`

sensing/
├── autoware_cuda_pointcloud_preprocessor/
│   ├── include/autoware/cuda_pointcloud_preprocessor/cuda_downsample_filter/cuda_voxel_grid_downsample_filter_node.hpp → defines `CudaVoxelGridDownsampleFilterNode`
│   ├── src/cuda_downsample_filter/cuda_voxel_grid_downsample_filter_node.cpp → `filter()` function
│   └── launch/ → `cuda_voxel_grid_downsample_filter.launch.xml`
├── autoware_image_transport_decompressor/
│   ├── include/autoware/image_transport_decompressor/image_transport_decompressor.hpp → defines `ImageTransportDecompressor`
│   ├── src/image_transport_decompressor.cpp → `onCompressedImage()` function
│   └── launch/ → `image_transport_decompressor.launch.xml`

system/
├── autoware_bluetooth_monitor/
│   ├── include/autoware/bluetooth_monitor/bluetooth_monitor.hpp → defines `BluetoothMonitor`
│   ├── src/bluetooth_monitor.cpp → `checkDeviceConnection()` function
│   └── launch/ → `bluetooth_monitor.launch.xml`
├── autoware_system_monitor/
│   ├── include/autoware/system_monitor/system_monitor.hpp → defines `SystemMonitor`
│   ├── src/system_monitor_node.cpp → `diagnosticsCallback()` function
│   └── launch/ → `system_monitor.launch.xml`

common/
├── autoware_auto_common/
│   ├── include/autoware_auto_common/helper_functions/byte_reader.hpp → `ByteReader()` function
│   ├── include/autoware_auto_common/helper_functions/float_comparisons.hpp → `abs_eq()` function
│   └── include/autoware_auto_common/helper_functions/message_adapters.hpp → `get_stamp()` function
├── autoware_component_interface_utils/
│   ├── include/autoware/component_interface_utils/rclcpp/interface.hpp → defines `NodeInterface` structure
│   ├── include/autoware/component_interface_utils/rclcpp/service.hpp → `Service()` function
│   └── include/autoware/component_interface_utils/rclcpp/publisher.hpp → `Publisher()` function

launch/
├── tier4_sensing_launch/
│   ├── launch/sensing.launch.xml → main sensing launch file
│   └── README.md → explains directory structure and usage
├── tier4_localization_launch/
│   └── launch/localization.launch.xml → main localization launch file
├── tier4_perception_launch/
│   └── launch/perception.launch.xml → main perception launch file
```