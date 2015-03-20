
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

struct neighborList {
	double *actualData;
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

#define SIZE_NBOR_LIST 128
#define NUM_AGENT 1024
#define DOT_R 1
#define NUM_CELL_DIM 8
#define NUM_CELL (NUM_CELL_DIM * NUM_CELL_DIM)
unsigned int window_width = 512, window_height = 512;
const int size = window_width*window_height;
Agent **agentList;
float* pixels = new float[size * 3];
int* cidStart = new int[NUM_CELL];
int* cidEnd = new int[NUM_CELL];
int* agentCids = new int[NUM_AGENT];
int* agentIds = new int[NUM_AGENT];
neighborList nborList;
neighborList nborListDev;

FILE *fpOut;

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

	for (int i = 0; i < NUM_CELL; i++)
		printf("(%d,%d)", cidStart[i], cidEnd[i]);

	// simulating generate neighbor list
#define RANGE 0.05 //environment dim ranging from 0 ~ 1
	int nborCount = 0;
	for (int i = 0; i < NUM_AGENT; i++) {
		int agentId = agentIds[i];
		Agent *agent = agentList[agentId];
		double posX = agent->data->x;
		double posY = agent->data->y;
		int cellXMin = (posX - RANGE) * NUM_CELL_DIM;
		int cellXMax = (posX + RANGE) * NUM_CELL_DIM;
		int cellYMin = (posY - RANGE) * NUM_CELL_DIM;
		int cellYMax = (posY + RANGE) * NUM_CELL_DIM;
		cellXMin = cellXMin < 0 ? 0 : cellXMin;
		cellXMax = cellXMax >= NUM_CELL_DIM ? NUM_CELL_DIM - 1 : cellXMax;
		cellYMin = cellYMin < 0 ? 0 : cellYMin;
		cellYMax = cellYMax >= NUM_CELL_DIM ? NUM_CELL_DIM - 1 : cellYMax;

		fprintf(fpOut, "(%f, %f), (%d, %d, %d, %d)\n", posX, posY, cellXMin, cellXMax, cellYMin, cellYMax);
		for (int cidY = cellYMin; cidY <= cellYMax; cidY++) {
			for (int cidX = cellXMin; cidX <= cellXMax; cidX++) {
				int cellId = util::zcode(cidX, cidY);
				for (int k = cidStart[cellId]; k < cidEnd[cellId]; k++) {
					Agent *other = agentList[agentIds[k]];
					double dist = agent->calDist(other);
					fprintf(fpOut, "(%f, %f, %f)", other->data->x, other->data->y, dist);
					fflush(fpOut);

					if (dist < RANGE) {
						fprintf(fpOut, " Y");
						fflush(fpOut);

						nborList.xList[nborCount] = other->data->x;
						nborList.yList[nborCount] = other->data->y;
						nborList.velXList[nborCount] = other->data->velX;
						nborList.velYList[nborCount] = other->data->velY;
						nborList.goalXList[nborCount] = other->data->goalX;
						nborList.goalYList[nborCount] = other->data->goalY;
						nborList.v0List[nborCount] = other->data->v0;
						nborList.massList[nborCount] = other->data->mass;
						nborList.nborIdList[nborCount] = agentIds[k];
						nborList.subjIdList[nborCount] = agentId;
						nborCount++;

						if (nborCount == SIZE_NBOR_LIST) {
							cudaMemcpy(nborListDev.actualData, nborList.actualData, sizeof(double) * SIZE_NBOR_LIST * 8, cudaMemcpyHostToDevice);
							fprintf(fpOut, "\n one batch");
							fflush(fpOut);
							nborCount = 0;
						}
					}
					fprintf(fpOut, "\n");
					fflush(fpOut);
				}
			}
		}
	}
}

int main(int argc, char** argv) {
	//Initialization
	srand(0);
	agentList = (Agent**)malloc(sizeof(Agent*) * NUM_AGENT);
	for (int i = 0; i < NUM_AGENT; i++) {
		agentList[i] = new Agent();
	}
	nborList.actualData = (double*)malloc(sizeof(double) * SIZE_NBOR_LIST * 8);
	nborList.xList = &nborList.actualData[0];
	nborList.yList = &nborList.actualData[SIZE_NBOR_LIST * 1];
	nborList.velXList = &nborList.actualData[SIZE_NBOR_LIST * 2];
	nborList.velYList = &nborList.actualData[SIZE_NBOR_LIST * 3];
	nborList.goalXList = &nborList.actualData[SIZE_NBOR_LIST * 4];
	nborList.goalYList = &nborList.actualData[SIZE_NBOR_LIST * 5];
	nborList.v0List = &nborList.actualData[SIZE_NBOR_LIST * 6];
	nborList.massList = &nborList.actualData[SIZE_NBOR_LIST * 7];

	cudaMalloc((void**)&nborListDev.actualData, sizeof(double) * SIZE_NBOR_LIST * 8);

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

	while (true) {
		for (int i = 0; i < NUM_AGENT; i++) {
			agentList[i]->data->x = (double)rand() / RAND_MAX;
			agentList[i]->data->y = (double)rand() / RAND_MAX;
		}
		neighborSearching();
		glutMainLoopEvent();
	}
	//glutMainLoop();
}