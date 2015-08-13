#include "cuda_runtime.h"

__global__ void testFunc() {

}

extern "C"
void runTest() {
	testFunc << <32, 32 >> >();
}