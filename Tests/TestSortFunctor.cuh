#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "thrust\sort.h"

#include <stdio.h>

struct location {
	double x;
	double y;
};

class TestSortFunctor {
public:
	void test() {
		int N = 1024;
		location *locList;
		int *ids;

		cudaMalloc((void**)&locList, sizeof(location) * N);
		cudaMalloc((void**)&locList, sizeof(int) * N);
		location *locListHost = (location*)malloc(sizeof(location) * N);
		int *idsHost = (int*)malloc(sizeof(int) * N);
		for (int i = 0; i < N; i++) {
			locList[i].x = locList[i].y = N - i;
			idsHost[i] = i;
		}

		cudaMemcpy(locList, locListHost, sizeof(location) * N, cudaMemcpyHostToDevice);
		cudaMemcpy(ids, idsHost, sizeof(int) * N, cudaMemcpyHostToDevice);

		//thrust::sort_by_key()
	}
};