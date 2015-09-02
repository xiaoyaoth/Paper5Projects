#include "thrust\sort.h"
#include "thrust\execution_policy.h"
#include "thrust\device_ptr.h"
#include <iostream>

using namespace std;

#define N 1024

class Worker {
public:
	static int id;
	cudaStream_t stream;
	int *keys;
	thrust::device_ptr<int> keysPtr;
	void init(int *keysHost) {
		cout << id++ << endl;
		cudaStreamCreate(&stream);
		cudaMalloc((void**)&keys, sizeof(int) * N);
		cudaMemcpy(keys, keysHost, sizeof(int) * N, cudaMemcpyHostToDevice);
		keysPtr = thrust::device_pointer_cast(keys);
	}
	void performSort() {
		thrust::sort(thrust::cuda::par.on(stream), keysPtr, keysPtr + N);
	}
};

int Worker::id = 0;

class TestThrustStream {
public:
	__host__ void test() {
		int *keysHost;
		keysHost = (int*)malloc(sizeof(int) * N);
		for (int i = 0; i < N; i++)
			keysHost[i] = N - i;

		int numWorkers = 16;
		Worker *workers = new Worker[16];
		for (int i = 0; i < numWorkers; i++)
			workers[i].init(keysHost);

		for (int i = 0; i < 100; i++)
			for (int j = 0; j < numWorkers; j++)
				workers[j].performSort();

		cudaMemcpyAsync(keysHost, workers[0].keys, sizeof(int) * N, cudaMemcpyDeviceToHost, workers[0].stream);
		for (int i = 0; i < 1024; i += N / 1024)
			cout << keysHost[i] << " ";
	}
};