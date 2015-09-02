//#include "cuda_runtime.h"
//#include "device_launch_parameters.h"
#include <iostream>
//#include <curand_kernel.h>

using namespace std;

//__host__ __device__ 
void swap(int *keys, int *values, int a, int b) {
	int temp = keys[a];
	keys[a] = keys[b];
	keys[b] = temp;

	temp = values[a];
	values[a] = values[b];
	values[b] = temp;
}

//__host__ __device__ 
void quickSortDev(int *keys, int *values, int numSubproblem, int subproblemIndex, int *sortIntermediates) {
	int startIndex = subproblemIndex;
	int endIndex = subproblemIndex + 1;
	int start = sortIntermediates[startIndex];
	int end = sortIntermediates[endIndex] - 1;
	if (end - start < 2)
		return;
	int pi = start + (end - start) / 2;
	swap(keys, values, start, pi);
	int pivot = keys[start];
	int i = start + 1, j = start + 1;
	for (; j < end; j++) {
		if (keys[j] < pivot) {
			swap(keys, values, i, j);
			i++;
		}
	}
	swap(keys, values, start, i-1);
	sortIntermediates[subproblemIndex + numSubproblem] = i;
}

#define N 99

void mergeSort(int *keys, int *values, int *keysRes, int *valuesRes, int start, int end) {
	if (end - start < 2) return;
	int mid = start + (end - start) / 2;
	mergeSort(keys, values, keysRes, valuesRes, start, mid);
	mergeSort(keys, values, keysRes, valuesRes, mid, end);
	int p1 = start, p2 = mid, p = start;
	while (p1 < mid && p2 < end) {
		if (keys[p1] < keys[p2]) {
			keysRes[p] = keys[p1];
			valuesRes[p] = values[p1];
			p1++;
		}
		else {
			keysRes[p] = keys[p2];
			valuesRes[p] = values[p2];
			p2++;
		}
		p++;
	}
	while (p1 < mid) {
		keysRes[p] = keys[p1];
		valuesRes[p] = values[p1];
		p1++;
		p++;
	}
	while (p2 < end) {
		keysRes[p] = keys[p2];
		valuesRes[p] = values[p2];
		p2++;
		p++;
	}
	for (int i = start; i < end; i++) {
		keys[i] = keysRes[i];
		values[i] = valuesRes[i];
	}

	for (int i = 0; i < N; i++) {
		cout << keys[i] << " ";
	}
	cout << endl;
}
/*
__global__ void mySortKernel(int *keys, int *values, int numElem) {
	int index = threadIdx.x + blockIdx.x * blockDim.x;
	if (index == 0)
		;
		//quickSortDev(keys, values, 0, numElem);
}
*/

class Solution {
public:
	void test() {
		int *keysHost = new int[N];
		int *valuesHost = new int[N];
		int *keysResHost = new int[N];
		int *valuesResHost = new int[N];
		int *sortIntermediates = new int[N + 1];
		int *keysDev, *valuesDev;
		for (int i = 0; i < N; i++) {
			keysHost[i] = N - i;
			valuesHost[i] = i;
			sortIntermediates[i] = N + 1;
		}
		mergeSort(keysHost, valuesHost, keysResHost, valuesResHost, 0, N);

		for (int i = 0; i < N; i++) {
			cout << keysResHost[i] << " " << valuesResHost[i] << "\t";
		}
		/*
		cudaMalloc((void**)&keysDev, sizeof(int) * N);
		cudaMalloc((void**)&valuesDev, sizeof(int) * N);
		cudaMemcpy(keysDev, keysHost, sizeof(int) * N, cudaMemcpyHostToDevice);
		cudaMemcpy(valuesDev, valuesHost, sizeof(int) * N, cudaMemcpyHostToDevice);
		mySortKernel << <32, 32 >> >(keysDev, valuesDev, N);
		cudaMemcpy(keysHost, keysDev, sizeof(int) * N, cudaMemcpyDeviceToHost);
		cudaMemcpy(valuesHost, valuesDev, sizeof(int) * N, cudaMemcpyDeviceToHost);
		for (int i = 0; i < N; i++) {
			cout << keysHost[i] << " " << valuesHost[i] << "\t";
		}
		*/
	}
};