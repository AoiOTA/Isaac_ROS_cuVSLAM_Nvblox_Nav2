#include <cuda_runtime.h>

#include <cmath>
#include <iostream>

__global__ void add_one(float* value) { value[0] += 1.0F; }

int main() {
  float* device_value = nullptr;
  float host_value = 41.0F;
  if (cudaMalloc(&device_value, sizeof(float)) != cudaSuccess) return 1;
  if (cudaMemcpy(device_value, &host_value, sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess) return 2;
  add_one<<<1, 1>>>(device_value);
  if (cudaDeviceSynchronize() != cudaSuccess) return 3;
  if (cudaMemcpy(&host_value, device_value, sizeof(float), cudaMemcpyDeviceToHost) != cudaSuccess) return 4;
  cudaFree(device_value);
  if (std::fabs(host_value - 42.0F) > 1.0e-6F) return 5;
  std::cout << "CUDA smoke result: " << host_value << '\n';
  return 0;
}
