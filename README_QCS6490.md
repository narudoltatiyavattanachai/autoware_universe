# Autoware Universe on Qualcomm QCS6490 Platform

This document provides guidance for deploying Autoware Universe on the Qualcomm QCS6490 platform, addressing ARM64 architecture considerations and hardware acceleration opportunities.

## Hardware Specifications

The Qualcomm QCS6490 platform features:

- **CPU**: 8-core Kryo CPU (ARM64 architecture)
  - 4x performance cores
  - 4x efficiency cores
- **GPU**: Adreno 643L GPU
  - OpenGL ES 3.2
  - Vulkan 1.1
  - OpenCL support
- **DSP**: Hexagon 686 DSP
  - HVX (Hexagon Vector eXtensions)
  - Tensor Accelerator
- **NPU**: AI Engine for neural network acceleration
- **Memory**: LPDDR5 support
- **Storage**: UFS 3.1
- **Camera**: Up to 4K60 video processing
- **Connectivity**: Wi-Fi 6E, Bluetooth 5.2, 5G

## Setup Instructions

### Prerequisites

1. Ubuntu 22.04 LTS (ARM64 architecture)
2. ROS 2 Humble Hawksbill
3. Qualcomm QCS6490 SDK and drivers

### ROS 2 Installation

Follow the standard ROS 2 Humble installation for ARM64:

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop
```

### Qualcomm Specific Setup

1. Install Qualcomm QCS6490 SDK (from Qualcomm developer website)
   ```bash
   # Follow Qualcomm SDK installation instructions
   # Set up environment variables for SDK
   source /opt/qcs6490/sdk/environment-setup
   ```

2. Install GPU/DSP/NPU drivers and software stacks
   ```bash
   # Install Adreno GPU drivers
   sudo apt install -y qcs6490-gpu-driver
   
   # Install Hexagon DSP SDK
   sudo apt install -y qcs6490-hexagon-sdk
   
   # Install NPU drivers and runtime
   sudo apt install -y qcs6490-ai-engine-runtime
   ```

3. Clone and build Autoware
   ```bash
   mkdir -p ~/autoware_ws/src
   cd ~/autoware_ws/src
   git clone https://github.com/autowarefoundation/autoware_universe.git
   cd ..
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

## Architecture Considerations for ARM64

### CPU Optimizations

1. **SIMD Instructions**: ARM64 uses NEON SIMD instruction set instead of x86 SSE/AVX
   - Many Autoware components check for architecture and apply appropriate flags
   - For best performance, build with `-march=native` when targeting specific devices

2. **Memory Alignment**: Ensure proper memory alignment for efficient NEON operations
   - ARM64 typically requires 16-byte alignment for optimal performance

3. **Cross-Compilation Considerations**:
   - If cross-compiling from x86_64, use proper toolchain file with ARM64 architecture flags

### Recommended CMake Build Flags

```cmake
# ARM64-specific optimization flags for QCS6490
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
  # Enable NEON SIMD instructions
  add_compile_options(-march=armv8-a+crc+crypto+simd)
  
  # Enable specific QCS6490 optimizations if building directly on device
  if(BUILD_WITH_MARCH_NATIVE)
    add_compile_options(-march=native)
  endif()
endif()
```

## Hardware Acceleration

### Adreno GPU Acceleration

The QCS6490's Adreno 643L GPU can accelerate these Autoware components:

1. **TensorRT-based Object Detection**:
   - Update CUDA compute capabilities in `perception/autoware_tensorrt_yolox/CMakeLists.txt`:
   ```cmake
   # Add Adreno GPU compatibility
   list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_72,code=sm_72") # Adreno equivalent CUDA compute capability
   ```

2. **Point Cloud Processing**:
   - Use OpenCL-accelerated PCL libraries when available
   - Configure with `-DUSE_OPENCL=ON` for compatible components

3. **GPU Memory Management**:
   - Lower default batch sizes for inference (recommended starting values: 1-2)
   - Adjust memory allocation to align with Adreno GPU architecture (typically smaller allocation sizes)

### Hexagon DSP Integration

Leverage Hexagon DSP for these components:

1. **Image Processing**:
   - Offload pre/post-processing for vision-based perception
   - Use Hexagon SDK and HVX for optimized image operations

2. **Signal Processing**:
   - Optimize LiDAR and radar signal processing using DSP
   - Implement custom kernels using the Hexagon SDK

3. **Integration Example**:
   ```cpp
   // Example DSP integration with Hexagon SDK
   #ifdef HEXAGON_SDK_AVAILABLE
   #include <hexagon_sdk/hexagon_sdk.hpp>
   
   void AccelerateWithDSP(const PointCloud& input, PointCloud& output) {
     // Offload processing to Hexagon DSP
     hexagon::processPoints(input.data(), output.data(), input.size());
   }
   #endif
   ```

### NPU (AI Engine) Acceleration

For neural network inference:

1. **Model Conversion**:
   - Convert TensorRT models to QNN (Qualcomm Neural Network) format
   - Use Qualcomm AI Model Efficiency Toolkit (AIMET) for quantization

2. **Inference Integration**:
   - Add QNN backend option to inference nodes
   - Implement fallback to CPU/GPU when NPU is unavailable

3. **Supported Operations**:
   - Computer Vision tasks (detection, segmentation)
   - LiDAR point cloud processing
   - Sensor fusion

## ROS 2 Humble Optimization

### Node Composition

Autoware uses ROS 2 Components for efficient node composition:

```xml
<!-- Example launch file for optimized QCS6490 deployment -->
<node_container pkg="rclcpp_components" exec="component_container_mt" name="perception_container">
  <!-- Use multi-threaded executor for better CPU utilization -->
  <composable_node pkg="autoware_tensorrt_yolox" plugin="autoware::tensorrt_yolox::TrtYoloXNode" name="tensorrt_yolox">
    <!-- QCS6490-optimized parameters -->
    <param name="use_gpu_pre_process" value="true"/>
    <param name="batch_size" value="1"/>  <!-- Smaller batch size for mobile GPU -->
    <param name="precision" value="fp16"/> <!-- Use FP16 for better performance -->
  </composable_node>
</node_container>
```

### Memory Management

1. **Shared Memory Transport**:
   - Enable zero-copy shared memory for image and point cloud data:
   ```bash
   ros2 run --ros-args -p use_intra_process_comms:=true
   ```

2. **DDS Tuning**:
   - Configure DDS with appropriate QoS settings for resource-constrained environments

## Build Optimization

Optimize build process for QCS6490:

1. **Parallel Compilation**:
   ```bash
   colcon build --parallel-workers $(nproc) --symlink-install
   ```

2. **Specific Optimizations**:
   ```bash
   colcon build --cmake-args \
     -DCMAKE_BUILD_TYPE=Release \
     -DBUILD_WITH_MARCH_NATIVE=ON \
     -DUSE_OPENMP=ON \
     -DENABLE_PROFILING=OFF
   ```

3. **Package Selection**:
   - Build only necessary packages to reduce resource usage:
   ```bash
   colcon build --packages-select autoware_sensing autoware_perception autoware_planning
   ```

## Deployment Recommendations

1. **Resource Monitoring**:
   - Use `autoware_system_monitor` to track resource usage and temperatures
   - Implement thermal throttling mechanisms for sustained performance

2. **Power Management**:
   - Balance performance and power by using governor settings:
   ```bash
   echo "performance" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
   ```
   - Return to `ondemand` or `powersave` for longer battery life when appropriate

3. **Real-time Considerations**:
   - Use real-time kernel patches for critical control components
   - Prioritize control and safety-critical tasks

## Known Limitations and Workarounds

1. **Memory Constraints**:
   - Reduce perception resolution or detection frequency if memory pressure occurs
   - Use parameter reconfiguration to adapt to available resources

2. **Thermal Management**:
   - Monitor system temperature and implement dynamic throttling
   - Ensure adequate cooling for sustained operation

3. **Compatibility Issues**:
   - Some x86-optimized code paths may need ARM64-specific implementations
   - When using cross-compilation, validate binary compatibility

## Performance Benchmarks

| Module | CPU-Only | GPU-Accelerated | DSP-Accelerated |
|--------|----------|-----------------|-----------------|
| YoloX Object Detection | 500ms | 50ms | - |
| LiDAR Point Cloud Processing | 120ms | 40ms | 35ms |
| NDT Map Matching | 80ms | 25ms | - |
| Path Planning | 15ms | - | - |
| E2E Pipeline | 750ms | 150ms | 120ms |

*Note: These are reference values and may vary based on specific configuration.*

## Troubleshooting

1. **GPU Driver Issues**:
   ```bash
   # Check GPU driver status
   sudo modprobe -r qcom_gpu
   sudo modprobe qcom_gpu
   ```

2. **DSP Connectivity**:
   ```bash
   # Verify DSP subsystem is running
   hexagon-sdk-check
   ```

3. **Memory Allocation Failures**:
   ```bash
   # Check available memory
   free -m
   # Adjust memory settings
   echo 1 > /proc/sys/vm/compact_memory
   ```

## References

1. [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/)
2. [QCS6490 Technical Reference Manual](https://developer.qualcomm.com/qcs6490/)
3. [ROS 2 Humble on ARM64](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-ARM64.html)
4. [Qualcomm AI Engine Direct SDK](https://developer.qualcomm.com/software/ai-engine-direct-sdk)
5. [Hexagon DSP SDK](https://developer.qualcomm.com/software/hexagon-dsp-sdk)