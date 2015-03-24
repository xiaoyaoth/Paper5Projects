
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

/* W B Langdon at MUN 10 May 2007
* Program to demonstarte use of OpenGL's glDrawPixels
*/

#ifdef _WIN32
#include <windows.h>
#endif

/*#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glut.h>*/

#include "GL/glew.h"
#include "GL/freeglut.h"

#include <iostream>
#include <sstream>
#include "math.h"

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <stdarg.h>

class AgentData {
public:
	double x;
	double y;
	double velX;
	double velY;
	double goalX;
	double goalY;
	double v0;
	double mass;
};

class Agent {
public:
	AgentData *data;
	Agent() {
		data = new AgentData();
		data->x = (double)rand() / RAND_MAX;
		data->y = (double)rand() / RAND_MAX;
	}
	double calDist(Agent *other) {
		float distSqr = (data->x - other->data->x) * (data->x - other->data->x)
			+ (data->y - other->data->y) * (data->y - other->data->y);
		return sqrt(distSqr);
	}
};

struct subjectList {
	double *xList;
	double *yList;
	double *velXList;
	double *velYList;
	double *goalXList;
	double *goalYList;
	double *v0List;
	double *massList;
};

struct neighborList {
	double *xList;
	double *yList;
	double *velXList;
	double *velYList;
	double *goalXList;
	double *goalYList;
	double *v0List;
	double *massList;
	int *nborIdList;
	int *subjIdList;
};

struct blockIndices {
	int numNbor;
	int numSubj;
	int firstSubj;
};

#define SIZE_NBOR_LIST 128
#define SIZE_SUBJ_LIST 32
#define NUM_AGENT 1024
#define DOT_R 1
#define NUM_CELL_DIM 8
#define NUM_CELL (NUM_CELL_DIM * NUM_CELL_DIM)
#define NUM_BLOCK 1
#define SIZE_BLOCK SIZE_NBOR_LIST
#define SIZE_NBOR_DATA 18
#define SIZE_SUBJ_DATA 16
unsigned int window_width = 512, window_height = 512;
const int size = window_width*window_height;
Agent **agentList;
float* pixels = new float[size * 3];
int* cidStart = new int[NUM_CELL];
int* cidEnd = new int[NUM_CELL];
int* agentCids = new int[NUM_AGENT];
int* agentIds = new int[NUM_AGENT];
blockIndices bi;
blockIndices* biDev;
int* nborData;
int* nborDataDev;
int *subjData;
int *subjDataDev;

FILE *fpOut;

__global__ void agentExecKernel(int *neighborDataDev, int *subjDataDev, blockIndices *biDev) {
	extern __shared__ int smem[];
	// load neighbor and subject indices
	int numNbor = biDev[blockIdx.x].numNbor;
	int numSubj = biDev[blockIdx.x].numSubj;
	int firstSubj = biDev[blockIdx.x].firstSubj; // the subject id of the first entry in neighbor list

	// load subject data into shared memory
	int offset = 0;
	while (offset + threadIdx.x < SIZE_SUBJ_LIST * SIZE_SUBJ_DATA) {
		smem[offset + threadIdx.x] = subjDataDev[offset + threadIdx.x];
		offset += blockDim.x;
	}
	__syncthreads();

	// set subject agent update zone in smem
	//double *fSumX = (double*)&smem[SIZE_SUBJ_LIST * SIZE_SUBJ_DATA]; //update zone for prop 1 follows reading zone
	//double *fSumY = (double*)&smem[SIZE_SUBJ_LIST * SIZE_SUBJ_DATA + SIZE_SUBJ_LIST * 2]; //update zone for prop 1 follows reading zone

	int *fSumX = &smem[SIZE_SUBJ_LIST * SIZE_SUBJ_DATA]; //update zone for prop 1 follows reading zone
	int *fSumY = &smem[SIZE_SUBJ_LIST * SIZE_SUBJ_DATA + SIZE_SUBJ_LIST]; //update zone for prop 1 follows reading zone

	offset = 0;
	while (offset + threadIdx.x < SIZE_SUBJ_LIST) {
		fSumX[offset + threadIdx.x] = 10;
		fSumY[offset + threadIdx.x] = 20;
		offset += blockDim.x;
	}
	__syncthreads();

	// obtain head address of neighbor data;
	double *xList = (double*)&neighborDataDev[0];
	double *yList = (double*)&neighborDataDev[SIZE_NBOR_LIST * 2];
	double *velXList = (double*)&neighborDataDev[SIZE_NBOR_LIST * 4];
	double *velYList = (double*)&neighborDataDev[SIZE_NBOR_LIST * 6];
	double *goalXList = (double*)&neighborDataDev[SIZE_NBOR_LIST * 8];
	double *goalYList = (double*)&neighborDataDev[SIZE_NBOR_LIST * 10];
	double *v0List = (double*)&neighborDataDev[SIZE_NBOR_LIST * 12];
	double *massList = (double*)&neighborDataDev[SIZE_NBOR_LIST * 14];
	int *nborList = &neighborDataDev[SIZE_NBOR_LIST * 16];
	int *subjList = &neighborDataDev[SIZE_NBOR_LIST * 17];

	// load neighbor data into register
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	double x = xList[threadIdx.x];
	double y = yList[threadIdx.x];
	double velX = velXList[threadIdx.x];
	double velY = velYList[threadIdx.x];
	double goalX = goalXList[threadIdx.x];
	double goalY = goalYList[threadIdx.x];
	double v0 = v0List[threadIdx.x];
	double mass = massList[threadIdx.x];
	int nbor = nborList[threadIdx.x];
	int subj = subjList[threadIdx.x];

	//do something with neighbor data
	double res = x + y + velX + velY + goalX + goalY + v0 + mass;

	//update subject agent data
	int temp = 1;
	while (++temp < 1000) {
		atomicInc((unsigned int*)&fSumX[subj - firstSubj], NUM_AGENT);
		atomicInc((unsigned int*)&fSumY[subj - firstSubj], NUM_AGENT);
	}
	__syncthreads();
	
	//printf("%d, %d %f, %f\n", threadIdx.x, subj, fSumX, fSumY);
}

namespace util {
	int zcode(int x, int y)
	{
		x &= 0x0000ffff;					// x = ---- ---- ---- ---- fedc ba98 7654 3210
		y &= 0x0000ffff;					// x = ---- ---- ---- ---- fedc ba98 7654 3210
		x = (x ^ (x << 8)) & 0x00ff00ff; // x = ---- ---- fedc ba98 ---- ---- 7654 3210
		y = (y ^ (y << 8)) & 0x00ff00ff; // x = ---- ---- fedc ba98 ---- ---- 7654 3210
		y = (y ^ (y << 4)) & 0x0f0f0f0f; // x = ---- fedc ---- ba98 ---- 7654 ---- 3210
		x = (x ^ (x << 4)) & 0x0f0f0f0f; // x = ---- fedc ---- ba98 ---- 7654 ---- 3210
		y = (y ^ (y << 2)) & 0x33333333; // x = --fe --dc --ba --98 --76 --54 --32 --10
		x = (x ^ (x << 2)) & 0x33333333; // x = --fe --dc --ba --98 --76 --54 --32 --10
		y = (y ^ (y << 1)) & 0x55555555; // x = -f-e -d-c -b-a -9-8 -7-6 -5-4 -3-2 -1-0
		x = (x ^ (x << 1)) & 0x55555555; // x = -f-e -d-c -b-a -9-8 -7-6 -5-4 -3-2 -1-0
		return x | (y << 1);
	}

	//template<class T>
	void swap(int *val, int a, int b) {
		int temp = val[a];
		val[a] = val[b];
		val[b] = temp;
	}

	//template<class T>
	void quickSort(int *key, int l, int r, int *value) {
		if (l == r)
			return;
		int randIdx = l + rand() % (r - l);
		swap(key, l, randIdx);
		swap(value, l, randIdx);
		int pivot = key[l];
		int i = l + 1, j = l + 1;
		for (; j < r; j++) {
			if (key[j] < pivot) {
				swap(key, i, j);
				swap(value, i, j);
				i++;
			}
		}
		swap(key, l, i - 1);
		swap(value, l, i - 1);
		quickSort(key, l, i - 1, value);
		quickSort(key, i, r, value);
	}
};

void neighborSearching() {
	// simulate agent moving
	for (int i = 0; i < NUM_AGENT; i++) {
		agentIds[i] = i;
		int cidX = agentList[i]->data->x * NUM_CELL_DIM;
		int cidY = agentList[i]->data->y * NUM_CELL_DIM;
		agentCids[i] = util::zcode(cidX, cidY);
	}

	// sorting agent ptr based on cell id
	util::quickSort(agentCids, 0, NUM_AGENT, agentIds);

	// construct cidStart and cidEnd
	for (int i = 0; i < NUM_AGENT; i++) {
		int cidPrev = agentCids[i];
		int cidNext = agentCids[(i + 1) % NUM_AGENT];
		if (cidPrev != cidNext) {
			cidEnd[cidPrev] = i + 1;
			cidStart[cidNext] = (i + 1) % NUM_AGENT;
		}
	}

	// manipulate nborData with nborList structure;
	neighborList nborList;
	nborList.xList = (double*)&nborData[0];
	nborList.yList = (double*)&nborData[SIZE_NBOR_LIST * 2];
	nborList.velXList = (double*)&nborData[SIZE_NBOR_LIST * 4];
	nborList.velYList = (double*)&nborData[SIZE_NBOR_LIST * 6];
	nborList.goalXList = (double*)&nborData[SIZE_NBOR_LIST * 8];
	nborList.goalYList = (double*)&nborData[SIZE_NBOR_LIST * 10];
	nborList.v0List = (double*)&nborData[SIZE_NBOR_LIST * 12];
	nborList.massList = (double*)&nborData[SIZE_NBOR_LIST * 14];
	nborList.nborIdList = &nborData[SIZE_NBOR_LIST * 16];
	nborList.subjIdList = &nborData[SIZE_NBOR_LIST * 17];

	subjectList subjList;
	subjList.xList = (double*)&subjData[0];
	subjList.yList = (double*)&subjData[SIZE_SUBJ_LIST * 2];
	subjList.velXList = (double*)&subjData[SIZE_SUBJ_LIST * 4];
	subjList.velYList = (double*)&subjData[SIZE_SUBJ_LIST * 6];
	subjList.goalXList = (double*)&subjData[SIZE_SUBJ_LIST * 8];
	subjList.goalYList = (double*)&subjData[SIZE_SUBJ_LIST * 10];
	subjList.v0List = (double*)&subjData[SIZE_SUBJ_LIST * 12];
	subjList.massList = (double*)&subjData[SIZE_SUBJ_LIST * 14];

	// simulating generate neighbor list
#define RANGE 0.05 //environment dim ranging from 0 ~ 1
	int nborCount = 0;
	int nborCountSettled = 0;
	bi.firstSubj = 0;
	for (int i = 0; i < NUM_AGENT; i++) {
		bool interrupted = false;

		// pick agent based on sorted order, and get its bounding box
		int agentId = agentIds[i];
		Agent *subj = agentList[agentId];
		double posX = subj->data->x;
		double posY = subj->data->y;

		int j = i - bi.firstSubj;
		subjList.xList[j] = subj->data->x;
		subjList.yList[j] = subj->data->y;
		subjList.velXList[j] = subj->data->velX;
		subjList.velYList[j] = subj->data->velY;
		subjList.goalXList[j] = subj->data->goalX;
		subjList.goalYList[j] = subj->data->goalY;
		subjList.v0List[j] = subj->data->v0;
		subjList.massList[j] = subj->data->mass;

		int cellXMin = (posX - RANGE) * NUM_CELL_DIM;
		int cellXMax = (posX + RANGE) * NUM_CELL_DIM;
		int cellYMin = (posY - RANGE) * NUM_CELL_DIM;
		int cellYMax = (posY + RANGE) * NUM_CELL_DIM;
		cellXMin = cellXMin < 0 ? 0 : cellXMin;
		cellXMax = cellXMax >= NUM_CELL_DIM ? NUM_CELL_DIM - 1 : cellXMax;
		cellYMin = cellYMin < 0 ? 0 : cellYMin;
		cellYMax = cellYMax >= NUM_CELL_DIM ? NUM_CELL_DIM - 1 : cellYMax;
		fprintf(fpOut, "%d, %d, (%f, %f), (%d, %d, %d, %d)\n", i, agentId, posX, posY, cellXMin, cellXMax, cellYMin, cellYMax);

		// iterate bounding box
		for (int cidY = cellYMin; cidY <= cellYMax; cidY++) {
			for (int cidX = cellXMin; cidX <= cellXMax; cidX++) {
				int cellId = util::zcode(cidX, cidY);
				for (int k = cidStart[cellId]; k < cidEnd[cellId]; k++) {
					// fill neighbor list
					Agent *nbor = agentList[agentIds[k]];
					double dist = subj->calDist(nbor);
					fprintf(fpOut, "(%d, %d, %f, %f, %f)", agentId, agentIds[k], nbor->data->x, nbor->data->y, dist);
					fflush(fpOut);

					if (dist < RANGE) {
						// detect if neighbor list is full
						if (nborCount == SIZE_NBOR_LIST) {
							interrupted = true;
							break;
						}
						fprintf(fpOut, " Y");
						fflush(fpOut);

						nborList.xList[nborCount] = nbor->data->x;
						nborList.yList[nborCount] = nbor->data->y;
						nborList.velXList[nborCount] = nbor->data->velX;
						nborList.velYList[nborCount] = nbor->data->velY;
						nborList.goalXList[nborCount] = nbor->data->goalX;
						nborList.goalYList[nborCount] = nbor->data->goalY;
						nborList.v0List[nborCount] = nbor->data->v0;
						nborList.massList[nborCount] = nbor->data->mass;
						nborList.nborIdList[nborCount] = agentIds[k];
						nborList.subjIdList[nborCount] = i;
						nborCount++;
					}
					fprintf(fpOut, "\n");
					fflush(fpOut);
				}
			}
		}

		// nborCount is temporary count, nborCountSettled is the number to be processed
		if (!interrupted)
			nborCountSettled = nborCount;
		else {
			// restore nborCount
			bi.numNbor = nborCountSettled;
			bi.numSubj = i - bi.firstSubj;

			// perform GPU processing
			cudaMemcpyAsync(nborDataDev, nborData, sizeof(int) * SIZE_NBOR_LIST * SIZE_NBOR_DATA, cudaMemcpyHostToDevice);
			cudaMemcpyAsync(subjDataDev, subjData, sizeof(int) * SIZE_SUBJ_LIST * SIZE_SUBJ_DATA, cudaMemcpyHostToDevice);
			cudaMemcpyAsync(biDev, &bi, sizeof(blockIndices), cudaMemcpyHostToDevice);
			fprintf(fpOut, "\n one batch");
			fflush(fpOut);

			size_t modZoneSize = sizeof(int) * SIZE_SUBJ_LIST * SIZE_SUBJ_DATA;
			size_t smemSize = sizeof(int) * SIZE_SUBJ_DATA * SIZE_SUBJ_LIST + modZoneSize;
			agentExecKernel<<<NUM_BLOCK, SIZE_BLOCK, smemSize>>>(nborDataDev, subjDataDev, biDev);

			cudaError_t error = cudaGetLastError();
			printf("CUDA Error: %s\n", cudaGetErrorString(error));

			//rollback the processing of one agent and
			bi.firstSubj = i;
			nborCount = 0;
			i--;
		}
	}
}

void display()
{
	//Create some nice colours (3 floats per pixel) from data -10..+10
	memset(pixels, 0, size * 3 * sizeof(float));
	for (int i = 0; i < NUM_AGENT; i++) {
		//colour(10.0 - ((i*20.0) / size), &pixels[i * 3]);
		int canvasX = agentList[i]->data->x * window_width;
		int canvasY = agentList[i]->data->y * window_height;

		for (int i = canvasX - DOT_R; i < canvasX + DOT_R; i++) {
			for (int j = canvasY - DOT_R; j < canvasY + DOT_R; j++) {
				if (i >= 0 && i < window_width && j >= 0 && j < window_height) {
					int idx = j * window_width + i;
					pixels[idx * 3] = 1;
					pixels[idx * 3 + 1] = 1;
					pixels[idx * 3 + 2] = 1;
				}
			}
		}
	}

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//http://msdn2.microsoft.com/en-us/library/ms537062.aspx
	//glDrawPixels writes a block of pixels to the framebuffer.

	glDrawPixels(window_width, window_height, GL_RGB, GL_FLOAT, pixels);

	glutSwapBuffers();
	glutPostRedisplay();

}

int main(int argc, char** argv) {
	//Initialization
	srand(0);
	agentList = (Agent**)malloc(sizeof(Agent*) * NUM_AGENT);
	for (int i = 0; i < NUM_AGENT; i++) {
		agentList[i] = new Agent();
	}
	cudaMallocHost((void**)&nborData, sizeof(int) * SIZE_NBOR_LIST * SIZE_NBOR_DATA);
	cudaMallocHost((void**)&subjData, sizeof(int) * SIZE_SUBJ_DATA * SIZE_SUBJ_LIST);

	cudaMalloc((void**)&nborDataDev, sizeof(int) * SIZE_NBOR_LIST * SIZE_NBOR_DATA);
	cudaMalloc((void**)&subjDataDev, sizeof(int) * SIZE_SUBJ_LIST * SIZE_SUBJ_DATA);
	cudaMalloc((void**)&biDev, sizeof(blockIndices));

	fpOut = fopen("output.txt", "w");

	// Visualization
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(window_width, window_height);
	glutCreateWindow("OpenGL glDrawPixels demo");

	glutDisplayFunc(display);
	//glutReshapeFunc(reshape);
	//glutMouseFunc(mouse_button);
	//glutMotionFunc(mouse_motion);
	//glutKeyboardFunc(keyboard);
	//glutIdleFunc(idle);

	glEnable(GL_DEPTH_TEST);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	//glPointSize(2);

	int tick = 0;
	while (tick++ < 3) {
		for (int i = 0; i < NUM_AGENT; i++) {
			agentList[i]->data->x = (double)rand() / RAND_MAX;
			agentList[i]->data->y = (double)rand() / RAND_MAX;
			agentList[i]->data->velX = -1;
			agentList[i]->data->velY = -1;
			agentList[i]->data->goalX = -1;
			agentList[i]->data->goalY = -1;
			agentList[i]->data->v0 = -1;
			agentList[i]->data->mass = -1;
		}
		neighborSearching();
		glutMainLoopEvent();
	}
	//glutMainLoop();
}