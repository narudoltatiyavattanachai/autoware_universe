# Build and Performance Optimizations for QCS6490

This document outlines build system and performance optimizations for deploying Autoware Universe on Qualcomm QCS6490 platforms.

## CMake Optimizations

### Processor-Specific Flags

Add ARM64-specific optimization flags to take advantage of the QCS6490 architecture:

```cmake
# Add to CMakeLists.txt or create a toolchain file
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
  # Base ARM64 optimizations
  add_compile_options(-march=armv8-a+crc+crypto+simd)
  
  # Enable Neon SIMD instructions
  add_compile_options(-mfpu=neon)
  
  # Enable link-time optimization for release builds
  if(CMAKE_BUILD_TYPE MATCHES Release)
    add_compile_options(-flto)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -flto")
  endif()
  
  # Hardware-specific optimizations when building on target
  if(BUILD_WITH_MARCH_NATIVE)
    add_compile_options(-march=native -mtune=native)
  endif()
endif()
```

### Package Organization

Organize packages to minimize dependencies and improve build time:

```cmake
# Example package organization for faster builds
set(PERCEPTION_PACKAGES
  autoware_tensorrt_yolox
  autoware_lidar_apollo_instance_segmentation
  autoware_multi_object_tracker
)

set(PLANNING_PACKAGES
  autoware_behavior_planner
  autoware_motion_planner
)

# Enable selective building
option(BUILD_PERCEPTION "Build perception modules" ON)
option(BUILD_PLANNING "Build planning modules" ON)

if(BUILD_PERCEPTION)
  foreach(package ${PERCEPTION_PACKAGES})
    add_subdirectory(${package})
  endforeach()
endif()

if(BUILD_PLANNING)
  foreach(package ${PLANNING_PACKAGES})
    add_subdirectory(${package})
  endforeach()
endif()
```

### Parallel Building

Configure colcon for optimal parallel building on QCS6490:

```bash
# For 8-core QCS6490 (4 performance + 4 efficiency)
colcon build --parallel-workers 6 --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Add to your `.bashrc`:

```bash
# Optimize for QCS6490
export COLCON_DEFAULTS_FILE=/path/to/colcon_qcs6490.yaml
```

With `colcon_qcs6490.yaml`:

```yaml
build:
  parallel-workers: 6
  symlink-install: true
  cmake-args:
    - -DCMAKE_BUILD_TYPE=Release
    - -DBUILD_WITH_MARCH_NATIVE=ON
```

## Memory Optimizations

### Reduce Memory Footprint

1. **Package Selection**

   Build only what's needed:

   ```bash
   colcon build --packages-select autoware_sensing autoware_perception autoware_planning
   ```

2. **Reduce Debug Information**

   For release builds:

   ```cmake
   if(CMAKE_BUILD_TYPE MATCHES Release)
     add_compile_options(-g1)  # Minimal debug info
   endif()
   ```

3. **Shared Libraries**

   Prefer shared libraries over static:

   ```cmake
   # Prefer this
   ament_auto_add_library(${PROJECT_NAME} SHARED ...)
   
   # Over this
   ament_auto_add_library(${PROJECT_NAME} STATIC ...)
   ```

### Memory Allocators

Consider using specialized allocators for better performance:

```cpp
// Add to high-allocation components
#include <jemalloc/jemalloc.h>

// Or use tcmalloc
#include <gperftools/tcmalloc.h>

// Implementation-specific allocator configuration
void ConfigureAllocators() {
  // Jemalloc configuration example
  mallctl("background_thread", nullptr, nullptr, &enabled, sizeof(enabled));
}
```

## ROS 2 Component Optimization

### Component Composition

Modify launch files to use optimal component configuration:

```xml
<!-- Optimize component container for QCS6490 -->
<node_container pkg="rclcpp_components" exec="component_container_mt" name="perception_container">
  <!-- Set CPU affinity to performance cores -->
  <param name="cpu_affinity" value="0,1,2,3"/> <!-- Adjust based on core layout -->
  
  <composable_node pkg="autoware_tensorrt_yolox" plugin="autoware::tensorrt_yolox::TrtYoloXNode" name="tensorrt_yolox">
    <param name="use_gpu_pre_process" value="true"/>
    <param name="precision" value="fp16"/>
  </composable_node>
</node_container>
```

### Intra-Process Communication

Enable zero-copy communication between components:

```cpp
// In component initialization
rclcpp::NodeOptions options;
options.use_intra_process_comms(true);
```

```xml
<!-- In launch files -->
<node_container pkg="rclcpp_components" exec="component_container_mt" name="sensor_fusion_container">
  <param name="use_intra_process_comms" value="true"/>
  <!-- Components -->
</node_container>
```

## DDS Tuning

Optimize DDS middleware settings for QCS6490:

```xml
<!-- Create custom middleware profiles in rmw_qos_profiles.xml -->
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="qcs6490_participant_profile">
        <rtps>
            <builtin>
                <discovery_config>
                    <leaseAnnouncement>
                        <sec>3</sec>
                        <nanosec>0</nanosec>
                    </leaseAnnouncement>
                    <leaseDuration>
                        <sec>10</sec>
                        <nanosec>0</nanosec>
                    </leaseDuration>
                </discovery_config>
                <metatrafficUnicastLocatorList>
                    <locator>
                        <udpv4>
                            <port>7400</port>
                        </udpv4>
                    </locator>
                </metatrafficUnicastLocatorList>
            </builtin>
            <name>qcs6490_optimized</name>
        </rtps>
    </participant>
    
    <data_writer profile_name="sensor_data_profile">
        <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
        <qos>
            <publishMode>
                <kind>ASYNCHRONOUS</kind>
            </publishMode>
        </qos>
    </data_writer>
</profiles>
```

And set environment variables:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/rmw_qos_profiles.xml
```

## CPU Core Management

Optimize core usage on QCS6490's heterogeneous CPU:

```bash
# Create a CPU management script
cat > /usr/local/bin/autoware-cpu-optimize.sh << 'EOF'
#!/bin/bash

# Set CPU governor to performance on big cores
for i in 0 1 2 3; do
  echo "performance" > /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor
done

# Set CPU governor to ondemand on LITTLE cores
for i in 4 5 6 7; do
  echo "ondemand" > /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor
done

# Set real-time priority for control processes
CONTROL_PIDS=$(ps -eo pid,comm | grep "autoware_control" | awk '{print $1}')
for pid in $CONTROL_PIDS; do
  chrt -f -p 80 $pid
done

# Pin perception to big cores
PERCEPTION_PIDS=$(ps -eo pid,comm | grep "autoware_perception" | awk '{print $1}')
for pid in $PERCEPTION_PIDS; do
  taskset -pc 0-3 $pid
done
EOF

chmod +x /usr/local/bin/autoware-cpu-optimize.sh
```

## I/O Optimizations

### File System Tuning

Optimize for SSD/eMMC performance:

```bash
# Add to system startup
echo "deadline" > /sys/block/mmcblk0/queue/scheduler
echo "1" > /sys/block/mmcblk0/queue/iosched/fifo_batch
```

### PCIe Bandwidth for Sensors

Optimize PCIe settings for sensor data throughput:

```bash
# Increase PCIe read buffer
echo 4096 > /sys/bus/pci/devices/0000:00:01.0/max_read_request_size

# Optimize IRQ handling for sensor interfaces
for i in $(ls -1 /proc/irq); do
  if [ -d "/proc/irq/$i" ]; then
    echo 1 > /proc/irq/$i/smp_affinity_list
  fi
done
```

## Compilation Caching

Use compiler caches to speed up rebuilds:

```bash
# Install ccache
apt install ccache

# Configure for colcon
echo 'export CC="ccache gcc"' >> ~/.bashrc
echo 'export CXX="ccache g++"' >> ~/.bashrc
echo 'export CCACHE_DIR=/path/to/cache' >> ~/.bashrc
echo 'export CCACHE_SIZE=10G' >> ~/.bashrc
```

Configure in CMake:
```cmake
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()
```

## Software Pipelining

Implement software pipelining for sensor processing:

```cpp
// Example pipelined perception implementation
class PipelinedPerception {
public:
  PipelinedPerception() : 
    preprocessing_thread_(&PipelinedPerception::PreprocessingWorker, this),
    inference_thread_(&PipelinedPerception::InferenceWorker, this),
    postprocessing_thread_(&PipelinedPerception::PostprocessingWorker, this) {}
    
  ~PipelinedPerception() {
    running_ = false;
    preprocessing_thread_.join();
    inference_thread_.join();
    postprocessing_thread_.join();
  }

private:
  void PreprocessingWorker() {
    while (running_) {
      auto data = input_queue_.pop();
      auto preprocessed = Preprocess(data);
      preprocess_queue_.push(preprocessed);
    }
  }
  
  void InferenceWorker() {
    while (running_) {
      auto preprocessed = preprocess_queue_.pop();
      auto result = RunInference(preprocessed);
      inference_queue_.push(result);
    }
  }
  
  void PostprocessingWorker() {
    while (running_) {
      auto inference = inference_queue_.pop();
      auto output = Postprocess(inference);
      PublishOutput(output);
    }
  }
  
  std::atomic<bool> running_{true};
  ThreadSafeQueue<SensorData> input_queue_;
  ThreadSafeQueue<PreprocessedData> preprocess_queue_;
  ThreadSafeQueue<InferenceResult> inference_queue_;
  
  std::thread preprocessing_thread_;
  std::thread inference_thread_;
  std::thread postprocessing_thread_;
};
```

## Performance Monitoring

Add QCS6490-specific performance monitoring:

```cpp
// Example monitoring utility
class QCS6490Monitor {
public:
  static void PrintCPUStatus() {
    for (int i = 0; i < 8; i++) {
      std::ifstream freq_file("/sys/devices/system/cpu/cpu" + std::to_string(i) + 
                             "/cpufreq/scaling_cur_freq");
      int freq;
      freq_file >> freq;
      
      std::cout << "CPU " << i << ": " << freq / 1000 << " MHz" << std::endl;
    }
  }
  
  static void PrintThermalStatus() {
    std::ifstream thermal_file("/sys/class/thermal/thermal_zone0/temp");
    int temp;
    thermal_file >> temp;
    
    std::cout << "SOC Temperature: " << temp / 1000 << "°C" << std::endl;
  }
  
  static void StartPeriodicMonitoring(int interval_ms = 5000) {
    static std::atomic<bool> monitoring_running{false};
    if (monitoring_running.exchange(true)) return;
    
    std::thread([interval_ms]() {
      while (true) {
        PrintCPUStatus();
        PrintThermalStatus();
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
      }
    }).detach();
  }
};
```

## Conclusion

Optimizing Autoware for QCS6490 requires a combination of ARM64-specific compiler flags, memory management strategies, and system-level tuning. By leveraging the platform's unique architecture and applying these optimizations, the system can achieve significantly better performance and efficiency.