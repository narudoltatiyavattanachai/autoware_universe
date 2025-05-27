# Architecture Analysis for QCS6490 Migration

This document analyzes architecture dependencies in the Autoware Universe codebase for migration to the Qualcomm QCS6490 platform (ARM64 architecture).

## Architecture Detection

Several components in the Autoware Universe codebase detect the host architecture and apply different compilation settings based on that detection. Here are the key findings:

### Architecture-Specific Code

1. **SIMD Instruction Sets**

   The primary architecture-specific code is found in packages using SIMD instructions:

   ```cmake
   # Example from localization/autoware_ndt_scan_matcher/CMakeLists.txt
   if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
     # For x86_64 architecture, SIMD instruction set is fixed below versions,
     # because the `ndt_omp` is optimized to these versions.
     add_compile_options(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
   else()
     # For other architecture, like arm64, compile flags are generally prepared by compiler
     # march=native is disabled as default for specific depending pcl libraries
     # or pre-building packages for other computers.
     if(BUILD_WITH_MARCH_NATIVE)
       add_compile_options(-march=native)
     endif()
   endif()
   ```

   **Migration action:** Ensure `-march=native` flag is enabled when building specifically for QCS6490 target hardware. For pre-built packages, use ARM64-specific optimizations.

2. **OpenMP Parallelization**

   Many compute-intensive components rely on OpenMP for parallelization:

   ```cmake
   find_package(OpenMP)
   if(OpenMP_CXX_FOUND)
     target_link_libraries(multigrid_ndt_omp OpenMP::OpenMP_CXX)
   else()
     message(WARNING "OpenMP not found")
   endif()
   ```

   **Migration action:** Ensure OpenMP is installed on the QCS6490 target and properly configured to optimize core usage across the asymmetric cores (performance vs efficiency).

## GPU Acceleration

Autoware components using CUDA and TensorRT need special attention during migration to the Adreno GPU.

### GPU Architecture Dependencies

1. **CUDA Compute Capabilities**

   The TensorRT YOLOX component has specific CUDA compute capability targets:

   ```cmake
   # From perception/autoware_tensorrt_yolox/CMakeLists.txt
   list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_75,code=sm_75")
   list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_86,code=sm_86")
   list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_87,code=sm_87")
   ```

   **Migration action:** Replace these CUDA compute capabilities with Adreno-compatible settings, or implement OpenCL alternatives for GPU acceleration.

2. **TensorRT Dependency**

   Many perception components use NVIDIA's TensorRT:

   ```cmake
   option(TRT_AVAIL "TensorRT available" OFF)
   # try to find the tensorRT modules
   find_library(NVINFER nvinfer)
   find_library(NVONNXPARSER nvonnxparser)
   ```

   **Migration action:** Replace TensorRT with Qualcomm QNN (Qualcomm Neural Network) SDK for optimal performance on the NPU. Create conditional paths for both backends.

### Hardware-Specific Optimizations

1. **TensorRT INT8 Calibration**

   Current calibration code contains warnings about Tegra platforms:

   ```cpp
   /**
    * @class Int8LegacyCalibrator
    * @brief Calibrator for Percentile
    * @warning We are confirming bug on Tegra like Xavier and Orin. We recommend use MinMax calibrator
    */
   ```

   **Migration action:** Test calibration methods on QCS6490 and determine the most effective calibrator for this platform.

## Memory Management

ARM64 platforms often have different memory alignment requirements and characteristics compared to x86_64.

### Memory Alignment Issues

1. **Eigen Library Alignment**

   NDT scan matcher notes important alignment concerns:

   ```cmake
   # In case mismatched instruction set are used, program causes a crash at its initialization
   # because of a misaligned access to the `Eigen` libraries' data structure.
   ```

   **Migration action:** Ensure consistent compiler flags across all packages that use Eigen libraries to prevent misalignment crashes.

2. **CUDA Memory Allocation**

   Several components use CUDA memory management:

   ```cpp
   CudaUniquePtr<float[]> input_d_;
   CudaUniquePtr<float[]> output_d_;
   CudaUniquePtrHost<float[]> output_h_;
   ```

   **Migration action:** Replace with Qualcomm-specific memory management or implement a compatibility layer.

## Performance-Critical Components

These components will require specific optimization for QCS6490:

1. **LiDAR Processing Pipelines**
   - Point cloud processing
   - Ground segmentation
   - Object detection

2. **Neural Network Inference**
   - YoloX object detection
   - Semantic segmentation

3. **Localization and Mapping**
   - NDT scan matching
   - Particle filter localization

## ROS 2 Component Structure

The codebase extensively uses ROS 2 Components for node composition, which is beneficial for resource-constrained environments:

```xml
<node_container pkg="rclcpp_components" exec="$(var container_type)" name="map_container" namespace="" output="both">
  <composable_node pkg="autoware_map_loader" plugin="autoware::map_loader::PointCloudMapLoaderNode" name="pointcloud_map_loader">
  <!-- Component parameters -->
  </composable_node>
</node_container>
```

**Migration action:** Continue using component-based architecture, which is well-suited for the QCS6490 platform's resource constraints. Use multi-threaded executors strategically across the asymmetric cores.

## Heterogeneous Computing Opportunities

Areas for potential acceleration using QCS6490's heterogeneous computing capabilities:

1. **Adreno GPU**
   - Neural network inference
   - Point cloud processing
   - Image processing

2. **Hexagon DSP**
   - Signal processing
   - Sensor data preprocessing
   - Fixed-point algorithms

3. **NPU (AI Engine)**
   - Object detection models
   - Semantic segmentation
   - Prediction models

## Recommended Migration Strategy

1. **Initial Compatibility Layer**
   - Create abstraction layers for hardware-accelerated components
   - Implement feature detection for available accelerators

2. **Incremental Optimization**
   - Port critical perception modules first
   - Then address planning and control components
   - Add platform-specific optimizations progressively

3. **Performance Benchmarking**
   - Establish baseline performance metrics
   - Track improvements with each optimization
   - Focus efforts on bottlenecks identified in the benchmarking

## Conclusion

The Autoware Universe codebase is largely platform-agnostic with specific areas that require attention for optimal performance on QCS6490. By focusing on GPU acceleration alternatives, memory management, and heterogeneous computing opportunities, the migration can achieve good performance on the ARM64-based QCS6490 platform.