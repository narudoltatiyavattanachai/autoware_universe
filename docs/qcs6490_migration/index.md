# Autoware Migration Guide for QCS6490

This guide provides comprehensive information for migrating Autoware Universe to run on Qualcomm QCS6490 platforms.

## Quick Start

To get started quickly, see [README_QCS6490.md](../../README_QCS6490.md) in the repository root directory.

## Migration Guide Contents

### 1. [Architecture Analysis](architecture_analysis.md)

Detailed analysis of architecture-dependent code in Autoware Universe and migration considerations for QCS6490's ARM64 architecture.

Key topics:
- SIMD instruction set differences
- Memory alignment considerations
- OpenMP optimizations
- Architecture-specific code paths

### 2. [Hardware Acceleration](hardware_acceleration.md)

Guide to leveraging QCS6490's heterogeneous computing capabilities for Autoware workloads.

Key topics:
- Adreno GPU acceleration replacing CUDA
- Hexagon DSP integration strategies
- NPU (AI Engine) utilization for neural networks
- Memory management across accelerators
- Benchmarking framework

### 3. [Build Optimizations](build_optimizations.md)

Build system optimizations and performance tuning for QCS6490 deployment.

Key topics:
- CMake optimizations for ARM64
- Package organization strategies
- Memory footprint reduction
- CPU core management
- Compilation caching and acceleration
- I/O optimizations

### 4. [Launch Compatibility](launch_compatibility.md)

Analysis of ROS 2 Humble launch file compatibility and optimizations for QCS6490.

Key topics:
- Component container configuration
- Memory-optimized launch settings
- Resource-aware component loading
- Hardware acceleration conditionals
- Lifecycle management for resource efficiency

## Migration Workflow

Follow this recommended workflow for migrating Autoware to QCS6490:

1. **Environment Setup**
   - Install Ubuntu 22.04 on QCS6490
   - Install ROS 2 Humble
   - Configure Qualcomm SDK and drivers

2. **Build System Configuration**
   - Set up cross-compilation environment if needed
   - Configure ARM64-specific optimization flags
   - Set up package selection for targeted building

3. **Core Functionality Migration**
   - Port localization components
   - Adapt perception for available accelerators
   - Test planning and control components

4. **Hardware Acceleration Integration**
   - Implement GPU acceleration using OpenCL/Vulkan
   - Add DSP acceleration for signal processing
   - Integrate NPU for neural network inference

5. **Performance Optimization**
   - Monitor and analyze resource usage
   - Apply CPU core optimizations
   - Implement memory management strategies
   - Fine-tune component parameters

6. **Validation and Testing**
   - Verify functional correctness
   - Benchmark performance against baseline
   - Test resource usage under load
   - Validate recovery from resource constraints

## Contributing to QCS6490 Support

Guidelines for contributing improvements to QCS6490 support:

1. Document all platform-specific changes
2. Create abstraction layers for hardware acceleration
3. Provide fallback mechanisms for missing capabilities
4. Include benchmarks comparing with and without acceleration
5. Keep the codebase maintainable by avoiding excessive #ifdefs

## Resources

- [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Qualcomm QCS6490 Developer Resources](https://developer.qualcomm.com/qcs6490/)
- [OpenCL for Adreno GPU](https://developer.qualcomm.com/software/adreno-gpu-sdk)