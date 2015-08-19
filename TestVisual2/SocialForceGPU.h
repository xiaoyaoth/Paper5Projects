#ifndef SOCIAL_FORCE_GPU_H
#define SOCIAL_FORCE_GPU_H

#include <vector>
#include <ctime>
#include <Windows.h>
#include <algorithm>
#include <fstream>
#include "inc\helper_math.h"

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <curand_kernel.h>

using namespace std;

#define DIST(ax, ay, bx, by) sqrt((ax-bx)*(ax-bx)+(ay-by)*(ay-by))

struct obstacleLine
{
	double sx;
	double sy;
	double ex;
	double ey;

	obstacleLine() {}

	obstacleLine(double sxx, double syy, double exx, double eyy) {
		sx = sxx;
		sy = syy;
		ex = exx;
		ey = eyy;
	}

	void init(double sxx, double syy, double exx, double eyy)
	{
		sx = sxx;
		sy = syy;
		ex = exx;
		ey = eyy;
	}

	__host__ __device__ double pointToLineDist(double2 loc)
	{
		double a, b;
		return this->pointToLineDist(loc, a, b);
	}

	__host__ __device__ double pointToLineDist(double2 loc, double &crx, double &cry)
	{
		double d = DIST(sx, sy, ex, ey);
		double t0 = ((ex - sx) * (loc.x - sx) + (ey - sy) * (loc.y - sy)) / (d * d);

		if (t0 < 0){
			d = sqrt((loc.x - sx) * (loc.x - sx) + (loc.y - sy) * (loc.y - sy));
		}
		else if (t0 > 1){
			d = sqrt((loc.x - ex) * (loc.x - ex) + (loc.y - ey) * (loc.y - ey));
		}
		else{
			d = sqrt(
				(loc.x - (sx + t0 * (ex - sx))) * (loc.x - (sx + t0 * (ex - sx))) +
				(loc.y - (sy + t0 * (ey - sy))) * (loc.y - (sy + t0 * (ey - sy)))
				);
		}
		crx = sx + t0 * (ex - sx);
		cry = sy + t0 * (ey - sy);

		//if (stepCount == 0 && id == 263) {
		//	printf("cross: (%f, %f)\n", crx, cry);
		//	printf("t0: %f, d: %f\n", t0, d);
		//}

		return d;
	}

	__host__ __device__ int intersection2LineSeg(double p0x, double p0y, double p1x, double p1y, double &ix, double &iy)
	{
		double s1x, s1y, s2x, s2y;
		s1x = p1x - p0x;
		s1y = p1y - p0y;
		s2x = ex - sx;
		s2y = ey - sy;

		double s, t;
		s = (-s1y * (p0x - sx) + s1x * (p0y - sy)) / (-s2x * s1y + s1x * s2y);
		t = (s2x * (p0y - sy) - s2y * (p0x - sx)) / (-s2x * s1y + s1x * s2y);

		if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
		{
			// Collision detected
			if (ix != NULL)
				ix = p0x + (t * s1x);
			if (iy != NULL)
				iy = p0y + (t * s1y);
			return 1;
		}
		return 0; // No collision
	}

	bool operator == (const obstacleLine & other) {
		return sx == other.sx && sy == other.sy && ex == other.ex && ey == other.ey;
	}

	bool operator != (const obstacleLine & other) {
		return !(*this == other);
	}
};
__host__ __device__ inline float dot(const double2& a, const double2& b)
{
	return a.x * b.x + a.y * b.y;
}
__host__ __device__ inline float length(const double2& v)
{
	return sqrtf(dot(v, v));
}
__host__ __device__ inline double2 operator-(const double2& a, const double2& b)
{
	return make_double2(a.x - b.x, a.y - b.y);
}

#define getLastCudaError(msg)	__getLastCudaError (msg, __FILE__, __LINE__)
inline void __getLastCudaError(const char *errorMessage, const char *file, const int line)
{
	cudaError_t err = cudaGetLastError();
	if (cudaSuccess != err) {
		fprintf(stderr, "%s(%i) : getLastCudaError() CUDA error : %s : (%d) %s.\n",
			file, line, errorMessage, (int)err, cudaGetErrorString(err));
		system("PAUSE");
		exit(-1);
	}
}

namespace util {
	template<class Type> void hostAllocCopyToDevice(Type *hostPtr, Type **devPtr)//device ptrInWorld must be double star
	{
		size_t size = sizeof(Type);
		cudaMalloc(devPtr, size);
		cudaMemcpy(*devPtr, hostPtr, size, cudaMemcpyHostToDevice);
		getLastCudaError("copyHostToDevice");
	}
}

#define BLOCK_SIZE 64
#define GRID_SIZE(n) (n%BLOCK_SIZE==0 ? n/BLOCK_SIZE : n/BLOCK_SIZE + 1)

/* application related constants */
#define	tao 0.5
#define	A 2000
#define	B 0.1
#define	k1 (1.2 * 100000)
#define k2 (2.4 * 100000) 
#define	maxv 3

#define NUM_CAP 512
#define NUM_PARAM 24
#define NUM_STEP 500
#define NUM_GOAL 7
#define ENV_DIM 64
#define NUM_CELL 16
#define CELL_DIM 4
#define RADIUS_I 5

#define NUM_WALLS 30

class SocialForceAgent;
class SocialForceClone;

typedef struct {
	double2 goal;
	double2 velocity;
	double v0;
	double mass;
	int numNeighbor;
	double2 loc;
	SocialForceAgent *agentPtr;
} SocialForceAgentData;

class SocialForceAgent {
public:
	SocialForceClone *myClone;
	//int id;
	SocialForceAgent *myOrigin;
	SocialForceAgentData data;
	SocialForceAgentData dataCopy;
	double2 goalSeq[NUM_GOAL];
	int goalIdx;

	uchar4 color;
	int contextId;
	//double gateSize;

	__device__ double correctCrossBoader(double val, double limit);
	__device__ void computeIndivSocialForceRoom(const SocialForceAgentData &myData, const SocialForceAgentData &otherData, double2 &fSum);
	__device__ void computeForceWithWall(const SocialForceAgentData &dataLocal, obstacleLine &wall, const int &cMass, double2 &fSum);
	__device__ void computeWallImpaction(const SocialForceAgentData &dataLocal, obstacleLine &wall, const double2 &newVelo, const double &tick, double &mint);
	__device__ void computeDirection(const SocialForceAgentData &dataLocal, double2 &dvt);
	__device__ void computeSocialForceRoom(SocialForceAgentData &dataLocal, double2 &fSum);
	__device__ void chooseNewGoal(const double2 &newLoc, double epsilon, double2 &newGoal);
	__device__ void step();
	__device__ void init(SocialForceClone* c, int idx);
	void initNewClone(SocialForceAgent *agent, SocialForceClone *clone);
};

extern "C"
void hookPointerAndData(SocialForceAgent** agentPtrArray, SocialForceAgent* agentArray, int numCap);
extern "C"
void initRootClone(SocialForceClone* cHost, SocialForceClone* cDev);

class AgentPool {
public:
	SocialForceAgent *agentArray;
	SocialForceAgent **agentPtrArray;
	bool *takenFlags;
	//int numElem;

	__host__ AgentPool(int numCap) {
		cudaMalloc((void**)&agentArray, sizeof(SocialForceAgent) * numCap);
		cudaMalloc((void**)&agentPtrArray, sizeof(SocialForceAgent*) * numCap);
		cudaMalloc((void**)&takenFlags, sizeof(bool) * numCap);
		cudaMemset(takenFlags, 0, sizeof(bool) * numCap);
		
		hookPointerAndData(agentPtrArray, agentArray, numCap);

		//APUtil::hookPointerAndData << <gSize, BLOCK_SIZE >> >(agentPtrArray, agentArray, numCap);
	}

	__host__ void reorder(int numElem) {
		int l = 0; int r = numElem;
		int i = l, j = l;
		for (; j < r; j++) {
			if (takenFlags[j] == true) {
				swap<SocialForceAgent*>(agentPtrArray, i, j);
				swap<bool>(takenFlags, i, j);
				i++;
			}
		}
		numElem = i;
	}

	template<class T>
	__host__ inline void swap(T * ar, int a, int b) {
		T t1 = ar[a];
		ar[a] = ar[b];
		ar[b] = t1;
	}
};

class SocialForceClone {
public:
	curandState *rState;
	int numElem;
	AgentPool *ap, *apHost;
	SocialForceAgent **context;
	bool *cloneFlag;
	int cloneParams[NUM_PARAM];
	obstacleLine walls[NUM_WALLS];
	obstacleLine gates[NUM_PARAM];
	bool takenMap[NUM_CELL * NUM_CELL];
	SocialForceClone *selfDev;

	uchar4 color;
	int cloneid;
	int parentCloneid;

	fstream fout;

	__host__ SocialForceClone(int id, int pv1[NUM_PARAM]) {
		cloneid = id;
		numElem = 0;
		apHost = new AgentPool(NUM_CAP);
		util::hostAllocCopyToDevice(apHost, &ap);
		cudaMalloc((void**)&rState, sizeof(curandState) * NUM_CAP);
		cudaMalloc((void**)&context, sizeof(SocialForceAgent*) * NUM_CAP);
		cudaMalloc((void**)&cloneFlag, sizeof(bool) * NUM_CAP);
		cudaMemset(context, 0, sizeof(void*) * NUM_CAP);
		cudaMemset(cloneFlag, 0, sizeof(bool) * NUM_CAP);
		memcpy(cloneParams, pv1, sizeof(int) * NUM_PARAM);

		int r1 = 1 + rand() % 4;
		for (int i = 0; i < r1; i++) {
			int r2 = rand() & NUM_PARAM;
			cloneParams[r2] = rand() % NUM_STEP;
		}

		double ps = 0.023; double dd = 0.25;
		for (int ix = 1; ix < 4; ix++) {
			for (int iy = 0; iy < 5; iy++) {
				int idx = (ix - 1) * 5 + iy;
				walls[idx].init(dd * ix * ENV_DIM, (dd * iy - 0.125 + ps) * ENV_DIM, dd * ix * ENV_DIM, (dd * iy + 0.125 - ps) * ENV_DIM);
			}
		}
		for (int iy = 1; iy < 4; iy++) {
			for (int ix = 0; ix < 5; ix++) {
				int idx = (iy - 1) * 5 + ix + 15;
				walls[idx].init((dd * ix - 0.125 + ps) * ENV_DIM, dd * iy * ENV_DIM, (dd * ix + 0.125 - ps) * ENV_DIM, dd * iy * ENV_DIM);
			}
		}
		
		for (int i = 0; i < NUM_PARAM; i++)
			gates[i].init(0, 0, 0, 0);

		/*
		for (int ix = 1; ix < 4; ix++) {
			for (int iy = 0; iy < 4; iy++) {
				int idx = (ix - 1) * 4 + iy;
				gates[idx].init(dd * ix * ENV_DIM, (dd * iy + 0.1) * ENV_DIM, dd * ix * ENV_DIM, (dd * (iy + 1) - 0.1) * ENV_DIM);
			}
		}
		for (int iy = 1; iy < 4; iy++) {
			for (int ix = 0; ix < 4; ix++) {
				int idx = (iy - 1) * 4 + ix + 12;
				gates[idx].init((dd * ix + 0.1) * ENV_DIM, dd * iy * ENV_DIM, (dd * (ix + 1) - 0.1) * ENV_DIM, dd * iy * ENV_DIM);
			}
		}
		*/

		util::hostAllocCopyToDevice(this, &this->selfDev);
	}
	void step(int stepCount);
	void alterGate(int stepCount);
	void swap();
	void output(int stepCount, char *s) {
		char filename[20];
		sprintf_s(filename, 20, "clone%d_%s.txt", cloneid, s);
		if (stepCount == 1)
			fout.open(filename, fstream::out);
		else
			fout.open(filename, fstream::app);
		fout << "========== stepCount: " << stepCount << " ===========" << endl;
		for (int i = 0; i < NUM_CAP; i++) {
			fout << context[i]->contextId << " [";
			fout << context[i]->data.loc.x << ",";
			fout << context[i]->data.loc.y << "] [";
			fout << context[i]->data.velocity.x << ", ";
			fout << context[i]->data.velocity.y << "] ";
			fout << endl;
		}
		fout.close();
	}
	void output2(int stepCount, char *s) {
		char filename[20];
		sprintf_s(filename, 20, "clone%d_num_%s.txt", cloneid, s);
		if (stepCount == 1)
			fout.open(filename, fstream::out);
		else
			fout.open(filename, fstream::app);
		fout << numElem << endl;
		fout.close();
	}
};


class SocialForceSimApp {
public:
	SocialForceClone **cAll;
	int paintId = 0;
	int totalClone = 1;
	int stepCount = 0;
	int rootCloneId = 0;
	int **cloneTree;

	SocialForceAgent *agentsForDraw;

	bool cloningCondition(SocialForceAgent *agent, bool *childTakenMap,
		SocialForceClone *parentClone, SocialForceClone *childClone);
	void performClone(SocialForceClone *parentClone, SocialForceClone *childClone);
	void compareAndEliminate(SocialForceClone *parentClone, SocialForceClone *childClone);
	void proc(int p, int c, bool o, char *s);
	void mst();

	int initSimClone() {
		srand(0);

		cAll = new SocialForceClone*[totalClone];
		agentsForDraw = new SocialForceAgent[NUM_CAP];
		cloneTree = new int*[2];
		int j = 0;

		int cloneParams[NUM_PARAM];
		for (int i = 0; i < NUM_PARAM; i++) {
			cloneParams[i] = rand() % NUM_STEP;
		}

		for (int i = 0; i < totalClone; i++) {
			cAll[i] = new SocialForceClone(i, cloneParams);
		}

		initRootClone(cAll[rootCloneId], cAll[rootCloneId]->selfDev);
		hookPointerAndData(cAll[rootCloneId]->context, cAll[rootCloneId]->apHost->agentArray, NUM_CAP);

		mst();

		return EXIT_SUCCESS;
	}

	void stepApp0(bool o) {
		stepCount++;
		cAll[rootCloneId]->step(stepCount);
		if (stepCount < 800 && o)
			cAll[rootCloneId]->output(stepCount, "s0");
		cAll[rootCloneId]->swap();
	}
	void stepApp1(bool o) {
		stepCount++;

		cAll[rootCloneId]->step(stepCount);
		proc(0, 1, 0, "s1");
		proc(0, 2, 0, "s1");
		proc(0, 4, 0, "s1");
		proc(1, 3, 0, "s1");
		proc(1, 5, 0, "s1");
		proc(2, 6, 0, "s1");
		proc(3, 7, 0, "s1");

		for (int j = 0; j < totalClone; j++) {
			cAll[j]->swap();
		}
	}
	void stepApp2(bool o) {
		stepCount++;
		cAll[rootCloneId]->step(stepCount);

		proc(0, 2, 0, "s2");
		proc(2, 1, 0, "s2");
		proc(2, 3, 0, "s2");
		proc(2, 4, 0, "s2");
		proc(2, 5, 0, "s2");
		proc(2, 6, 0, "s2");
		proc(2, 7, o, "s2");

		for (int j = 0; j < totalClone; j++) {
			cAll[j]->swap();
		}
	}
	void stepApp3(bool o){
		stepCount++;

		cAll[rootCloneId]->step(stepCount);
		proc(0, 1, 0, "s3");
		proc(0, 2, 0, "s3");
		proc(0, 4, 0, "s3");
		proc(0, 3, 0, "s3");
		proc(0, 5, 0, "s3");
		proc(0, 6, 0, "s3");
		proc(0, 7, o, "s3");

		for (int j = 0; j < totalClone; j++) {
			cAll[j]->swap();
		}
	}
	void stepApp4_1(bool o) {
		int **cloneDiff = new int*[totalClone];
		for (int i = 0; i < totalClone; i++) {
			cloneDiff[i] = new int[totalClone];
			for (int j = 0; j < totalClone; j++)
				cloneDiff[i][j] = 0;
		}

		for (int i = 0; i < totalClone; i++) {
			for (int j = 0; j < totalClone; j++) {
				for (int k = 0; k < NUM_PARAM; k++) {
					if (cAll[i]->cloneParams[k] != cAll[j]->cloneParams[k])
						cloneDiff[i][j]++;
				}
				wchar_t message[10];
				swprintf_s(message, 10, L"%d ", cloneDiff[i][j]);
				OutputDebugString(message);
			}
			OutputDebugString(L"\n");
		}

		for (int i = 0; i < totalClone; i++) {
			int loc = 0;
			int nDiff = 0;
			for (int j = 0; j < NUM_PARAM; j++) {
				wchar_t message[10];
				swprintf_s(message, 10, L"%d ", cAll[i]->cloneParams[j]);
				OutputDebugString(message);
			}
			OutputDebugString(L"\n");
		}


	}
	void stepApp4(bool o) {
		stepCount++;

		cAll[rootCloneId]->step(stepCount);
		proc(0, 1, 0, "s4");
		proc(0, 2, 0, "s4");
		proc(0, 3, 0, "s4");
		proc(0, 5, 0, "s4");
		proc(0, 7, 0, "s4");
		proc(3, 4, 0, "s4");
		proc(5, 6, o, "s4");

		for (int j = 0; j < totalClone; j++) {
			cAll[j]->swap();
		}
	}
	void stepApp5(bool o) {
		stepCount++;
		cAll[rootCloneId]->step(stepCount);
		for (int i = 1; i < totalClone; i++)
			proc(cloneTree[0][i], i, 0, "s5");
		for (int j = 0; j < totalClone; j++) {
			cAll[j]->swap();
		}
	}

	void stepApp6(bool o) {
		stepCount++;
		cAll[rootCloneId]->step(stepCount);
		for (int i = 1; i < totalClone; i++)
			proc(i - 1, i, 0, "s6");
		for (int j = 0; j < totalClone; j++) {
			cAll[j]->swap();
		}
	}
	void stepApp(){
		stepCount++;
		cAll[rootCloneId]->step(stepCount);
		cAll[rootCloneId]->swap();
	}

};


#endif