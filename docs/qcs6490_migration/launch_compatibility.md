# ROS 2 Humble Launch File Compatibility for QCS6490

This document reviews launch file compatibility for Autoware on QCS6490 with ROS 2 Humble and suggests optimizations for resource-constrained platforms.

## Launch System Overview

Autoware Universe uses ROS 2 launch files extensively for composing its software stack. When migrating to QCS6490, these launch files need review for:

1. Compatibility with ROS 2 Humble launch syntax
2. Resource utilization optimization for QCS6490
3. Component-based execution optimizations

## Launch File Structure Analysis

Autoware launch files are organized hierarchically:

```
launch/
├── tier4_control_launch/
│   └── launch/
│       └── control.launch.xml
├── tier4_localization_launch/
│   └── launch/
│       └── localization.launch.xml
├── tier4_perception_launch/
│   └── launch/
│       └── perception.launch.xml
└── tier4_planning_launch/
    └── launch/
        └── planning.launch.xml
```

### Launch File Compatibility

ROS 2 Humble introduces changes in the launch system compared to earlier ROS 2 versions:

1. **Launch Format Standardization**: Humble standardizes on Python-based launch files, though XML is still supported
2. **Component Node Lifecycle**: Better support for lifecycle management of components
3. **Improved Parameters**: Enhanced parameter passing and manipulation

## Launch System Compatibility Analysis

The examined launch files appear compatible with ROS 2 Humble, using standard XML syntax:

```xml
<!-- Example from tier4_map_launch/launch/map.launch.xml -->
<launch>
  <!-- map files -->
  <arg name="pointcloud_map_path"/>
  <arg name="pointcloud_map_metadata_path"/>
  <arg name="lanelet2_map_path"/>
  <arg name="map_projector_info_path"/>

  <!-- Parameter files -->
  <arg name="pointcloud_map_loader_param_path"/>
  <arg name="lanelet2_map_loader_param_path"/>
  <arg name="map_tf_generator_param_path"/>
  <arg name="map_projection_loader_param_path"/>

  <!-- select container type -->
  <arg name="use_multithread" default="false"/>
  <let name="container_type" value="component_container" unless="$(var use_multithread)"/>
  <let name="container_type" value="component_container_mt" if="$(var use_multithread)"/>

  <group>
    <push-ros-namespace namespace="map"/>
    
    <node_container pkg="rclcpp_components" exec="$(var container_type)" name="map_container" namespace="" output="both">
      <!-- Components defined here -->
    </node_container>
  </group>
</launch>
```

This syntax is fully supported in ROS 2 Humble.

## QCS6490-Specific Optimizations

### Component Container Configuration

For QCS6490, component containers should be optimized for the heterogeneous CPU architecture:

```xml
<!-- Recommended configuration for QCS6490 -->
<arg name="use_multithread" default="true"/>  <!-- Enable multi-threading -->
<let name="container_type" value="component_container_mt"/>  <!-- Always use multi-threaded container -->

<node_container pkg="rclcpp_components" exec="$(var container_type)" name="perception_container" namespace="" output="screen">
  <!-- CPU affinity parameters for QCS6490 -->
  <param name="cpu_affinity" value="0,1,2,3"/>  <!-- Use high-performance cores -->
  <param name="thread_priority" value="95"/>     <!-- Higher real-time priority -->
  
  <!-- Component definitions -->
</node_container>
```

### Memory-Optimized Launch Configuration

For memory-constrained QCS6490 deployment:

```xml
<!-- Memory-optimized perception launch -->
<node_container pkg="rclcpp_components" exec="component_container_mt" name="perception_container">
  <!-- Enable shared memory transport -->
  <param name="use_intra_process_comms" value="true"/>
  
  <!-- Reduced buffer sizes -->
  <param name="buffer_size" value="2"/>  <!-- Default might be higher -->
  
  <!-- Lower resolution perception for constrained devices -->
  <param name="image_width" value="640"/>
  <param name="image_height" value="480"/>
  <param name="downsampling_ratio" value="0.5"/>
</node_container>
```

## Python-Based Launch Alternatives

For complex launch scenarios, Python-based launch files offer more flexibility and are recommended for ROS 2 Humble:

```python
# example_qcs6490.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Detect QCS6490 capabilities dynamically
    def configure_for_qcs6490(context):
        # Check for available GPU/DSP/NPU
        has_gpu = check_gpu_available()
        has_dsp = check_dsp_available()
        has_npu = check_npu_available()
        
        # Select appropriate perception implementation
        if has_npu:
            perception_plugin = "autoware::perception::NPUAcceleratedPerception"
            perception_params = {"use_npu": True, "precision": "fp16"}
        elif has_gpu:
            perception_plugin = "autoware::perception::GPUAcceleratedPerception"
            perception_params = {"use_gpu": True}
        else:
            perception_plugin = "autoware::perception::CPUPerception"
            perception_params = {}
            
        # Create container with appropriate components
        container = ComposableNodeContainer(
            name="perception_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                ComposableNode(
                    package="autoware_perception",
                    plugin=perception_plugin,
                    name="perception_node",
                    parameters=[perception_params]
                )
            ],
            output="screen"
        )
        
        return [container]
    
    return LaunchDescription([
        OpaqueFunction(function=configure_for_qcs6490)
    ])
```

## Launch Groups for Incremental Loading

Organize launch files to allow incremental loading based on available resources:

```xml
<!-- tiered_perception.launch.xml -->
<launch>
  <arg name="tier" default="full"/>  <!-- Options: minimal, standard, full -->
  
  <!-- Base perception components always loaded -->
  <include file="$(find-pkg-share autoware_perception_launch)/launch/base_perception.launch.xml"/>
  
  <!-- Standard perception components -->
  <group if="$(eval tier in ['standard', 'full'])">
    <include file="$(find-pkg-share autoware_perception_launch)/launch/object_detection.launch.xml"/>
  </group>
  
  <!-- Full perception components -->
  <group if="$(eval tier == 'full')">
    <include file="$(find-pkg-share autoware_perception_launch)/launch/traffic_light.launch.xml"/>
    <include file="$(find-pkg-share autoware_perception_launch)/launch/occupancy_grid.launch.xml"/>
  </group>
</launch>
```

This allows starting with minimal resource usage and adding more components as needed.

## Component Node Parameters for QCS6490

Adjust component parameters for QCS6490 constraints:

```xml
<!-- Example component with QCS6490-optimized parameters -->
<composable_node pkg="autoware_tensorrt_yolox" plugin="autoware::tensorrt_yolox::TrtYoloXNode" name="tensorrt_yolox">
  <!-- QCS6490-optimized parameters -->
  <param name="model_path" value="$(find-pkg-share autoware_tensorrt_yolox)/data/yolox_tiny.onnx"/>
  <param name="precision" value="fp16"/>
  <param name="use_gpu_preprocessor" value="true"/>
  <param name="batch_size" value="1"/>
  <param name="score_threshold" value="0.5"/> <!-- Slightly higher to reduce processing -->
</composable_node>
```

## Resource Monitoring Launch Integration

Add resource monitoring for QCS6490:

```xml
<!-- Add to main launch file -->
<node pkg="autoware_system_monitor" exec="resource_monitoring_node" name="resource_monitor">
  <param name="cpu_threshold" value="90.0"/>
  <param name="memory_threshold" value="85.0"/>
  <param name="temperature_threshold" value="85.0"/>
  
  <!-- QCS6490 specific parameters -->
  <param name="monitor_cpu_cores" value="0,1,2,3"/> <!-- Monitor high-performance cores -->
  <param name="gpu_monitoring_enabled" value="true"/>
  <param name="gpu_device_name" value="adreno"/> <!-- Adreno GPU -->
</node>
```

## Remapping for Hardware-Specific Sensors

Configure launch files to adapt to QCS6490-specific sensors:

```xml
<!-- Flexible sensor configuration -->
<arg name="camera_type" default="usb"/>  <!-- Options: usb, mipi, etc. -->
<arg name="camera_ns" default="camera"/>

<group if="$(eval camera_type == 'usb')">
  <!-- USB camera configuration -->
  <node pkg="usb_cam" exec="usb_cam_node" name="$(var camera_ns)">
    <!-- USB camera parameters -->
  </node>
</group>

<group if="$(eval camera_type == 'mipi')">
  <!-- MIPI camera configuration for QCS6490 -->
  <node pkg="camera_mipi" exec="camera_mipi_node" name="$(var camera_ns)">
    <!-- MIPI camera parameters -->
  </node>
</group>

<!-- Common remapping regardless of camera type -->
<remap from="$(var camera_ns)/image_raw" to="/sensing/camera/image_raw"/>
```

## Conditional Hardware Acceleration 

Enable hardware acceleration based on available capabilities:

```xml
<!-- In perception.launch.xml -->
<arg name="enable_gpu_acceleration" default="true"/>
<arg name="enable_dsp_acceleration" default="false"/> <!-- Enable when DSP interface is ready -->
<arg name="enable_npu_acceleration" default="false"/> <!-- Enable when NPU interface is ready -->

<!-- Object detection with conditional acceleration -->
<group>
  <push-ros-namespace namespace="perception"/>
  
  <!-- GPU accelerated path -->
  <group if="$(var enable_gpu_acceleration)">
    <include file="$(find-pkg-share autoware_perception_launch)/launch/gpu_detection.launch.xml"/>
  </group>
  
  <!-- DSP accelerated path -->
  <group if="$(var enable_dsp_acceleration)">
    <include file="$(find-pkg-share autoware_perception_launch)/launch/dsp_detection.launch.xml"/>
  </group>
  
  <!-- NPU accelerated path -->
  <group if="$(var enable_npu_acceleration)">
    <include file="$(find-pkg-share autoware_perception_launch)/launch/npu_detection.launch.xml"/>
  </group>
  
  <!-- Fallback CPU path -->
  <group unless="$(eval enable_gpu_acceleration or enable_dsp_acceleration or enable_npu_acceleration)">
    <include file="$(find-pkg-share autoware_perception_launch)/launch/cpu_detection.launch.xml"/>
  </group>
</group>
```

## ROS 2 Humble Compatible Lifecycle Management

Leverage improved lifecycle management in ROS 2 Humble:

```xml
<!-- QCS6490-optimized lifecycle management -->
<node pkg="autoware_state_monitor" exec="state_monitor_node" name="state_monitor">
  <!-- Configure ordered startup to manage resources -->
  <param name="startup_sequence" value="
    [
      {component: 'localization', timeout_sec: 10}, 
      {component: 'perception', timeout_sec: 20},
      {component: 'planning', timeout_sec: 10},
      {component: 'control', timeout_sec: 5}
    ]
  "/>
  
  <!-- Configure orderly shutdown -->
  <param name="shutdown_sequence" value="
    [
      {component: 'control', timeout_sec: 1},
      {component: 'planning', timeout_sec: 1},
      {component: 'perception', timeout_sec: 2},
      {component: 'localization', timeout_sec: 2}
    ]
  "/>
</node>
```

## Conclusion

Autoware's launch files are compatible with ROS 2 Humble on QCS6490 with minimal changes needed. By applying the recommended optimizations, the system can better utilize QCS6490's heterogeneous computing resources and overcome its memory constraints. The component-based architecture of Autoware suits resource-constrained embedded platforms well when properly configured.