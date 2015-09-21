#include <omp.h>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"


#define N 10000


__global__ void kernel_a()
{
	float sum = 0.0;
	for (int i = 0; i < N; i++)
	{
		sum = sum + cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1);
	}
}

__global__ void kernel_b()
{
	float sum = 0.0;
	for (int i = 0; i < N / 2; i++)
	{
		sum = sum + cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1);
	}
}

__global__ void kernel_c()
{
	float sum = 0.0;
	for (int i = 0; i < N / 4; i++)
	{
		sum = sum + cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1);
	}
}

__global__ void kernel_d()
{
	float sum = 0.0;
	for (int i = 0; i < N / 8; i++)
	{
		sum = sum + cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1)*cos(0.1);
	}
}

int main()
{

	int n_streams = 8;
	int n_threads = 4;
	cudaError_t cudaStatus;

	// Allocate and initialize an array of stream handles
	cudaStream_t *streams = (cudaStream_t *)malloc(n_streams * sizeof(cudaStream_t));
	for (int i = 0; i < n_streams; i++)
	{
		cudaStreamCreate(&(streams[i]));
	}

	// Parallel section
	for (int iter = 0; iter < 10; iter++) {
#pragma omp parallel for num_threads(n_threads)

		for (int i = 0; i < n_streams; i++)
		{

			kernel_a << <1, 1, 0, streams[i] >> >();

			kernel_b << <1, 1, 0, streams[i] >> >();

			kernel_c << <1, 1, 0, streams[i] >> >();

			kernel_d << <1, 1, 0, streams[i] >> >();
		}
	}
	

	// release all stream
	for (int i = 0; i < n_streams; i++)
	{
		cudaStreamDestroy(streams[i]);
	}
	free(streams);

	// cudaDeviceReset must be called before exiting in order for profiling and
	// tracing tools such as Nsight and Visual Profiler to show complete traces.
	cudaStatus = cudaDeviceReset();

	return 0;
}