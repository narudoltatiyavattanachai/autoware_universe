# QCS6490 Hardware Acceleration for Autoware

This document outlines hardware acceleration opportunities for Autoware Universe on the Qualcomm QCS6490 platform, focusing on GPU, DSP, and NPU utilization.

## Hardware Accelerators on QCS6490

### Adreno 643L GPU

The QCS6490 features an Adreno 643L GPU with:

- OpenGL ES 3.2 support
- Vulkan 1.1 support
- OpenCL support
- Hardware-accelerated video encoding/decoding

### Hexagon 686 DSP

The Hexagon 686 DSP includes:

- Hexagon Vector eXtensions (HVX)
- Tensor Accelerator
- Specialized signal processing capabilities

### AI Engine (NPU)

The Qualcomm AI Engine provides:

- Neural network acceleration
- Fixed-point and floating-point operations
- Support for common ML frameworks through ONNX/QNN

## GPU Acceleration (Adreno)

### Current CUDA-Based Implementations

Autoware Universe currently uses NVIDIA CUDA for GPU acceleration:

```cpp
// Example from perception/autoware_tensorrt_yolox
cuda_add_library(${PROJECT_NAME}_gpu_preprocess
  SHARED
  src/preprocess.cu
)
```

### Migration Strategies

1. **OpenCL-Based Alternative**

   Replace CUDA implementations with OpenCL for Adreno compatibility:

   ```cpp
   // OpenCL equivalent for YoloX preprocessing
   #include <CL/cl.h>
   
   void PreprocessOpenCL(const cv::Mat& input, float* output) {
     // Initialize OpenCL context, queue, etc.
     cl_context context = CreateContext();
     cl_command_queue queue = CreateCommandQueue(context);
     
     // Load and compile OpenCL kernel
     cl_program program = LoadProgram(context, "preprocess_kernels.cl");
     cl_kernel kernel = clCreateKernel(program, "preprocess", NULL);
     
     // Execute preprocessing
     // ...
   }
   ```

2. **Vulkan Compute Implementation**

   For newer Adreno GPUs, Vulkan Compute can offer better performance:

   ```cpp
   // Vulkan compute example
   void PreprocessVulkan(const cv::Mat& input, float* output) {
     // Initialize Vulkan
     vk::Instance instance = CreateInstance();
     vk::PhysicalDevice physicalDevice = GetPhysicalDevice(instance);
     vk::Device device = CreateDevice(physicalDevice);
     
     // Set up compute pipeline
     vk::ShaderModule computeShader = LoadShader(device, "preprocess.spv");
     vk::PipelineLayout pipelineLayout = CreatePipelineLayout(device);
     vk::Pipeline pipeline = CreateComputePipeline(device, computeShader, pipelineLayout);
     
     // Execute compute operation
     // ...
   }
   ```

### Key GPU Components for Migration

1. **YoloX Object Detection**
   - Replace TensorRT with Qualcomm QNN or OpenCL-accelerated inference
   - Use Adreno-optimized image preprocessing

2. **Point Cloud Processing**
   - Port PCL GPU modules to OpenCL
   - Implement Adreno-specific optimizations for large point cloud operations

3. **Image Processing**
   - Leverage OpenCV's OpenCL backend (cv::UMat)
   - Use Vulkan for advanced image operations

## DSP Acceleration (Hexagon)

### Integration Approach

The Hexagon DSP can accelerate signal processing tasks:

1. **Hexagon SDK Integration**

   ```cpp
   // Example integration with Hexagon DSP
   #include <hexagon/hexagon_types.h>
   #include <hexagon/hexagon_sdk.h>
   
   class HexagonAccelerator {
   public:
     HexagonAccelerator() {
       // Initialize Hexagon DSP
       hexagon_init();
     }
     
     ~HexagonAccelerator() {
       // Cleanup
       hexagon_terminate();
     }
     
     bool ProcessPointCloud(const PointCloud& input, PointCloud& output) {
       // Offload processing to Hexagon DSP
       hexagon_process_points(input.data(), output.data(), input.size());
       return true;
     }
   };
   ```

2. **FastRPC Integration**

   ```cpp
   // Using FastRPC to communicate with DSP
   #include <AEEStdDef.h>
   #include <remote.h>
   
   remote_handle64 handle;
   
   bool InitializeDSP() {
     int result = remote_open("libdsp_skel.so", &handle);
     return result == 0;
   }
   
   bool ProcessOnDSP(const float* input, float* output, size_t size) {
     return remote_process(handle, input, output, size) == 0;
   }
   ```

### Key DSP Acceleration Targets

1. **LiDAR Signal Processing**
   - Point cloud filtering
   - Ground plane estimation
   - Feature extraction

2. **Sensor Data Pre-processing**
   - Noise reduction
   - Range compression
   - Temporal filtering

3. **Fixed-point Operations**
   - Geometry calculations
   - Distance computations
   - Grid-based operations

## NPU Acceleration (AI Engine)

### Qualcomm Neural Network Integration

1. **Model Conversion**

   Convert trained models to QNN format:

   ```bash
   # Example conversion workflow
   python -m qnn.converter \
     --model_path model.onnx \
     --output_path model.qnn \
     --input_layout NCHW \
     --quantization_overrides opset:10
   ```

2. **QNN Runtime Integration**

   ```cpp
   // Example QNN integration
   #include "QnnInterface.h"
   
   class QNNBackend {
   public:
     QNNBackend(const std::string& model_path) {
       // Initialize QNN
       Qnn_CreateContextParams_t contextParams;
       Qnn_CreateContext(QNN_BACKEND_HTP, &contextParams, &context);
       
       // Load model
       LoadModel(model_path);
     }
     
     bool Inference(const float* input, size_t input_size, float* output, size_t output_size) {
       // Execute inference
       Qnn_ExecuteParams_t params;
       Qnn_Execute(context, &params);
       return true;
     }
     
   private:
     Qnn_ContextHandle_t context;
     // Additional members for QNN operation
   };
   ```

### Conditional Backend Selection

Implement architecture that can select optimal backend based on available hardware:

```cpp
enum class InferenceBackend {
  CPU,
  GPU,
  DSP,
  NPU
};

std::unique_ptr<InferenceInterface> CreateInferenceBackend(
    InferenceBackend backend_type,
    const std::string& model_path) {
  switch (backend_type) {
    case InferenceBackend::CPU:
      return std::make_unique<CPUBackend>(model_path);
    case InferenceBackend::GPU:
      if (IsAdrenoGPUAvailable()) {
        return std::make_unique<OpenCLBackend>(model_path);
      }
      return std::make_unique<CPUBackend>(model_path);
    case InferenceBackend::DSP:
      if (IsHexagonDSPAvailable()) {
        return std::make_unique<HexagonBackend>(model_path);
      }
      return CreateInferenceBackend(InferenceBackend::GPU, model_path);
    case InferenceBackend::NPU:
      if (IsQNNAvailable()) {
        return std::make_unique<QNNBackend>(model_path);
      }
      return CreateInferenceBackend(InferenceBackend::DSP, model_path);
  }
  return std::make_unique<CPUBackend>(model_path);
}
```

## Benchmarking Framework

To evaluate acceleration effectiveness, implement a benchmarking system:

```cpp
template <typename AcceleratorT>
void BenchmarkAccelerator(
    const std::string& name,
    const std::vector<Payload>& payloads) {
  
  AcceleratorT accelerator;
  
  // Warm-up
  for (int i = 0; i < 5; i++) {
    accelerator.Process(payloads[0]);
  }
  
  // Benchmark
  auto start = std::chrono::high_resolution_clock::now();
  
  for (const auto& payload : payloads) {
    accelerator.Process(payload);
  }
  
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      end - start).count();
  
  std::cout << "Accelerator: " << name
            << ", Time: " << duration << " ms"
            << ", Avg: " << duration / payloads.size() << " ms/item"
            << std::endl;
}
```

## Memory Management Strategies

Efficient memory management is critical for hardware accelerators:

1. **Zero-Copy Memory**

   ```cpp
   // Zero-copy buffer allocation for Adreno
   cl_mem CreateSharedBuffer(cl_context context, size_t size, void** host_ptr) {
     cl_int err = 0;
     cl_mem_flags flags = CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR;
     
     cl_mem buffer = clCreateBuffer(context, flags, size, nullptr, &err);
     if (err != CL_SUCCESS) return nullptr;
     
     *host_ptr = clEnqueueMapBuffer(
         queue, buffer, CL_TRUE, CL_MAP_READ | CL_MAP_WRITE,
         0, size, 0, nullptr, nullptr, &err);
     
     return buffer;
   }
   ```

2. **ION Memory Allocation**

   For shared memory across CPU, GPU, DSP, and NPU:

   ```cpp
   // ION memory allocation example
   int AllocateIONMemory(size_t size, int* fd, void** vaddr) {
     int ion_fd = open("/dev/ion", O_RDONLY);
     if (ion_fd < 0) return -1;
     
     struct ion_allocation_data alloc_data = {
       .len = size,
       .heap_id_mask = ION_HEAP_SYSTEM_MASK,
       .flags = ION_FLAG_CACHED | ION_FLAG_CACHED_NEEDS_SYNC,
     };
     
     if (ioctl(ion_fd, ION_IOC_ALLOC, &alloc_data) < 0) {
       close(ion_fd);
       return -2;
     }
     
     *fd = alloc_data.fd;
     *vaddr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, *fd, 0);
     
     close(ion_fd);
     return 0;
   }
   ```

## Configuration Framework

Create a unified configuration system to control acceleration settings:

```yaml
# Example hardware_acceleration.yaml
acceleration:
  default_backend: "auto"  # auto, cpu, gpu, dsp, npu
  
  gpu:
    enabled: true
    use_vulkan: false
    opencl_device_index: 0
    memory_pool_size_mb: 512
  
  dsp:
    enabled: true
    priority: high
    runtime: "hvx"  # hvx, cdsp
    
  npu:
    enabled: true
    precision: "fp16"  # fp32, fp16, int8
    model_cache_dir: "/var/cache/autoware/models"
    
  components:
    object_detection:
      backend: "npu"
      fallback: "gpu"
      batch_size: 1
      
    point_cloud_processing:
      backend: "gpu"
      fallback: "cpu"
```

## Conclusion

Successful hardware acceleration on QCS6490 requires a multi-pronged approach leveraging all available accelerators. The heterogeneous nature of the platform necessitates careful memory management and backend selection strategies to achieve optimal performance while maintaining code flexibility.