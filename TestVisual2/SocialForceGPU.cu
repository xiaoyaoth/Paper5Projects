#include <vector>
#include <ctime>
#include <Windows.h>
#include <algorithm>
#include <fstream>
#include "inc\helper_math.h"

#include "cuda_runtime.h"
#include "device_launch_parameters.h"

using namespace std;

#define DIST(ax, ay, bx, by) sqrt((ax-bx)*(ax-bx)+(ay-by)*(ay-by))

/* helper functions and data structures*/
#define checkCudaErrors(err)	__checkCudaErrors(err, __FILE__, __LINE__)
inline void __checkCudaErrors(cudaError err, const char *file, const int line)
{
	if (cudaSuccess != err) {
		fprintf(stderr, "%s(%i) : CUDA Runtime API error %d: %s.\n",
			file, line, (int)err, cudaGetErrorString(err));
		exit(-1);
	}
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
struct Color{
	char r, g, b;
	Color & operator = (const Color & rhs) {
		r = rhs.r;
		g = rhs.g;
		b = rhs.b;
		return *this;
	}
	Color() {
		r = rand() % 255;
		g = rand() % 255;
		b = rand() % 255;
	}
};
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
	int goalIdx = 0;

	Color color;
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
	void init(int idx);
	void initNewClone(SocialForceAgent *agent, SocialForceClone *clone);
};

namespace APUtil {
	__global__ void hookPointerAndData(SocialForceAgent** agentPtrArray, SocialForceAgent* agentArray, int numCap) {
		int index = threadIdx.x + blockIdx.x * blockDim.x;
		if (index < numCap) agentPtrArray[index] = &agentArray[index];
	}
};

class AgentPool {
public:
	SocialForceAgent *agentArray;
	SocialForceAgent **agentPtrArray;
	bool *takenFlags;
	int numElem;

	__host__ AgentPool(int numCap) {
		numElem = 0;
		cudaMalloc((void**)&agentArray, sizeof(SocialForceAgent) * numCap);
		cudaMalloc((void**)&agentPtrArray, sizeof(SocialForceAgent*) * numCap);
		cudaMalloc((void**)&takenFlags, sizeof(bool) * numCap);
		cudaMemset(takenFlags, 0, sizeof(bool) * numCap);
		int gSize = GRID_SIZE(numCap);
		//APUtil::hookPointerAndData << <gSize, BLOCK_SIZE >> >(agentPtrArray, agentArray, numCap);
	}

	__host__ void reorder() {
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
	AgentPool *ap, *apHost;
	SocialForceAgent **context;
	bool *cloneFlag;
	int cloneParams[NUM_PARAM];
	obstacleLine walls[NUM_WALLS];
	obstacleLine gates[NUM_PARAM];
	bool takenMap[NUM_CELL * NUM_CELL];

	Color color;
	int cloneid;
	int parentCloneid;

	fstream fout;

	__host__ SocialForceClone(int id, int pv1[NUM_PARAM]) {
		cloneid = id;
		apHost = new AgentPool(NUM_CAP);
		util::hostAllocCopyToDevice(apHost, &ap);
		cudaMalloc((void**)&context, sizeof(SocialForceAgent*) * NUM_CAP);
		cudaMalloc((void**)&cloneFlag, sizeof(bool) * NUM_CAP);
		cudaMemset(context, 0, sizeof(void*) * NUM_CAP);
		cudaMemset(cloneFlag, 0, sizeof(bool) * NUM_CAP);
		color = Color();
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
	}
	void step(int stepCount);
	void alterGate(int stepCount);
	void swap() {
		for (int i = 0; i < ap->numElem; i++) {
			SocialForceAgent &agent = *ap->agentPtrArray[i];
			agent.data = agent.dataCopy;
		}
	}
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
		fout << ap->numElem << endl;
		fout.close();
	}
};
__device__ double SocialForceAgent::correctCrossBoader(double val, double limit)
{
	if (val >= limit)
		return limit - 0.001;
	else if (val < 0)
		return 0;
	return val;
}
void SocialForceAgent::computeIndivSocialForceRoom(const SocialForceAgentData &myData, const SocialForceAgentData &otherData, double2 &fSum){
	double cMass = 100;
	//my data
	const double2& loc = myData.loc;
	const double2& goal = myData.goal;
	const double2& velo = myData.velocity;
	const double& v0 = myData.v0;
	const double& mass = myData.mass;
	//other's data
	const double2& locOther = otherData.loc;
	const double2& goalOther = otherData.goal;
	const double2& veloOther = otherData.velocity;
	const double& v0Other = otherData.v0;
	const double& massOther = otherData.mass;

	double d = 1e-15 + sqrt((loc.x - locOther.x) * (loc.x - locOther.x) + (loc.y - locOther.y) * (loc.y - locOther.y));
	double dDelta = mass / cMass + massOther / cMass - d;
	double fExp = A * exp(dDelta / B);
	double fKg = dDelta < 0 ? 0 : k1 *dDelta;
	double nijx = (loc.x - locOther.x) / d;
	double nijy = (loc.y - locOther.y) / d;
	double fnijx = (fExp + fKg) * nijx;
	double fnijy = (fExp + fKg) * nijy;
	double fkgx = 0;
	double fkgy = 0;
	if (dDelta > 0) {
		double tix = -nijy;
		double tiy = nijx;
		fkgx = k2 * dDelta;
		fkgy = k2 * dDelta;
		double vijDelta = (veloOther.x - velo.x) * tix + (veloOther.y - velo.y) * tiy;
		fkgx = fkgx * vijDelta * tix;
		fkgy = fkgy * vijDelta * tiy;
	}
	fSum.x += fnijx + fkgx;
	fSum.y += fnijy + fkgy;
}
__device__ void SocialForceAgent::computeForceWithWall(const SocialForceAgentData &dataLocal, obstacleLine &wall, const int &cMass, double2 &fSum) {
	double diw, crx, cry;
	const double2 &loc = dataLocal.loc;

	diw = wall.pointToLineDist(loc, crx, cry);
	double virDiw = DIST(loc.x, loc.y, crx, cry);

	if (virDiw == 0)
		return;

	double niwx = (loc.x - crx) / virDiw;
	double niwy = (loc.y - cry) / virDiw;
	double drw = dataLocal.mass / cMass - diw;
	double fiw1 = A * exp(drw / B);
	if (drw > 0)
		fiw1 += k1 * drw;
	double fniwx = fiw1 * niwx;
	double fniwy = fiw1 * niwy;

	double fiwKgx = 0, fiwKgy = 0;
	if (drw > 0)
	{
		double fiwKg = k2 * drw * (dataLocal.velocity.x * (-niwy) + dataLocal.velocity.y * niwx);
		fiwKgx = fiwKg * (-niwy);
		fiwKgy = fiwKg * niwx;
	}

	fSum.x += fniwx - fiwKgx;
	fSum.y += fniwy - fiwKgy;
}
__device__ void SocialForceAgent::computeWallImpaction(const SocialForceAgentData &dataLocal, obstacleLine &wall, const double2 &newVelo, const double &tick, double &mint){
	double crx, cry, tt;
	const double2 &loc = dataLocal.loc;
	int ret = wall.intersection2LineSeg(
		loc.x,
		loc.y,
		loc.x + 0.5 * newVelo.x * tick,
		loc.y + 0.5 * newVelo.y * tick,
		crx,
		cry
		);
	if (ret == 1)
	{
		if (fabs(crx - loc.x) > 0)
			tt = (crx - loc.x) / (newVelo.x * tick);
		else
			tt = (crx - loc.y) / (newVelo.y * tick + 1e-20);
		if (tt < mint)
			mint = tt;
	}
}
__device__ void SocialForceAgent::computeDirection(const SocialForceAgentData &dataLocal, double2 &dvt) {
	//my data
	const double2& loc = dataLocal.loc;
	const double2& goal = dataLocal.goal;
	const double2& velo = dataLocal.velocity;
	const double& v0 = dataLocal.v0;
	const double& mass = dataLocal.mass;

	dvt.x = 0;	dvt.y = 0;
	double2 diff; diff.x = 0; diff.y = 0;
	double d0 = sqrt((loc.x - goal.x) * (loc.x - goal.x) + (loc.y - goal.y) * (loc.y - goal.y));
	diff.x = v0 * (goal.x - loc.x) / d0;
	diff.y = v0 * (goal.y - loc.y) / d0;
	dvt.x = (diff.x - velo.x) / tao;
	dvt.y = (diff.y - velo.y) / tao;
}
__device__ void SocialForceAgent::computeSocialForceRoom(SocialForceAgentData &dataLocal, double2 &fSum) {
	fSum.x = 0; fSum.y = 0;
	double ds = 0;

	int neighborCount = 0;

	for (int i = 0; i < NUM_CAP; i++) {
		SocialForceAgent *other = myClone->context[i];
		SocialForceAgentData otherData = other->data;
		ds = length(otherData.loc - dataLocal.loc);
		if (ds < 6 && ds > 0) {
			neighborCount++;
			computeIndivSocialForceRoom(dataLocal, otherData, fSum);
		}
	}
	dataLocal.numNeighbor = neighborCount;
}
__device__ void SocialForceAgent::chooseNewGoal(const double2 &newLoc, double epsilon, double2 &newGoal) {
	double2 g1 = goalSeq[goalIdx];
	double2 g2 = goalSeq[goalIdx + 1];

	int x = (int)g1.x % (int)(ENV_DIM / 4);
	int y = (int)g1.y % (int)(ENV_DIM / 4);


	if (x > y && newLoc.y >= g1.y) {
		newGoal = g2;
		goalIdx++;
	}

	if (x < y && newLoc.x >= g1.x) {
		newGoal = g2;
		goalIdx++;
	}
}
__device__ void SocialForceAgent::step(){
	double cMass = 100;

	if (this->contextId == 52 && this->myClone->cloneid == 2) {
		printf("");
	}

	const double2& loc = data.loc;
	const double2& goal = data.goal;
	const double2& velo = data.velocity;
	const double& v0 = data.v0;
	const double& mass = data.mass;

	//compute the direction
	double2 dvt;
	computeDirection(data, dvt);

	//compute force with other agents
	double2 fSum;
	computeSocialForceRoom(data, fSum);

	//compute force with walls and gates
	for (int i = 0; i < NUM_WALLS; i++) {
		obstacleLine wall = myClone->walls[i];
		computeForceWithWall(data, wall, cMass, fSum);
	}
	for (int i = 0; i < NUM_PARAM; i++) {
		obstacleLine gate = myClone->gates[i];
		if (gate.sx == 0)
			continue;
		computeForceWithWall(data, gate, cMass, fSum);
	}

	//sum up
	dvt.x += fSum.x / mass;
	dvt.y += fSum.y / mass;

	double2 newVelo = data.velocity;
	double2 newLoc = data.loc;
	double2 newGoal = data.goal;

	double tick = 0.1;
	newVelo.x += dvt.x * tick * (1);// + this->random->gaussian() * 0.1);
	newVelo.y += dvt.y * tick * (1);// + this->random->gaussian() * 0.1);
	double dv = sqrt(newVelo.x * newVelo.x + newVelo.y * newVelo.y);

	if (dv > maxv) {
		newVelo.x = newVelo.x * maxv / dv;
		newVelo.y = newVelo.y * maxv / dv;
	}

	double mint = 1;
	for (int i = 0; i < NUM_WALLS; i++) {
		obstacleLine wall = myClone->walls[i];
		computeWallImpaction(data, wall, newVelo, tick, mint);
	}
	for (int i = 0; i < NUM_PARAM; i++) {
		obstacleLine gate = myClone->gates[i];
		if (gate.sx == 0)
			continue;
		computeWallImpaction(data, gate, newVelo, tick, mint);
	}

	newVelo.x *= mint;
	newVelo.y *= mint;
	newLoc.x += newVelo.x * tick;
	newLoc.y += newVelo.y * tick;

	double goalTemp = goal.x;

	chooseNewGoal(newLoc, mass / cMass, newGoal);

	newLoc.x = correctCrossBoader(newLoc.x, ENV_DIM);
	newLoc.y = correctCrossBoader(newLoc.y, ENV_DIM);

	dataCopy = data;

	dataCopy.loc = newLoc;
	dataCopy.velocity = newVelo;
	dataCopy.goal = newGoal;
}
void SocialForceAgent::init(int idx) {
	this->color = Color();
	this->contextId = idx;
	this->myOrigin = NULL;

	SocialForceAgentData & dataLocal = this->data; //= &sfModel->originalAgents->dataArray[dataSlot];
	dataLocal.agentPtr = this;
	dataLocal.loc.x = ENV_DIM * 0.0 + (float)rand() / (float)RAND_MAX * ENV_DIM;
	dataLocal.loc.y = ENV_DIM * 0.0 + (float)rand() / (float)RAND_MAX * ENV_DIM;

	dataLocal.velocity.x = 2;//4 * (this->random->uniform()-0.5);
	dataLocal.velocity.y = 2;//4 * (this->random->uniform()-0.5);

	dataLocal.v0 = 2;
	dataLocal.mass = 50;
	dataLocal.numNeighbor = 0;
	//chooseNewGoal(dataLocal.loc, 0, dataLocal.goal);

	int ix = dataLocal.loc.x / (0.25 * ENV_DIM);
	int iy = dataLocal.loc.y / (0.25 * ENV_DIM);

	this->goalSeq[NUM_GOAL - 1] = make_double2(ENV_DIM, ENV_DIM);
	for (int i = 0; i < NUM_GOAL - 1; i++) {
		this->goalSeq[i] = make_double2(ENV_DIM, ENV_DIM);
		double r = (float)rand() / (float)RAND_MAX;

		if (ix < 3) {
			if (iy < 3 && r < 0.5) {
				this->goalSeq[i] = make_double2((ix * 0.25 + 0.125) * ENV_DIM, (++iy) * 0.25 * ENV_DIM);
			}
			else {
				this->goalSeq[i] = make_double2((++ix) * 0.25 * ENV_DIM, (iy * 0.25 + 0.125) * ENV_DIM);
			}
		}
		else if (iy < 3) {
			this->goalSeq[i] = make_double2((ix * 0.25 + 0.125) * ENV_DIM, (++iy) * 0.25 * ENV_DIM);
		}
	}

	dataLocal.goal = this->goalSeq[goalIdx];
	this->dataCopy = dataLocal;
}
void SocialForceAgent::initNewClone(SocialForceAgent *parent, SocialForceClone *childClone) {
	this->color = childClone->color;
	this->contextId = parent->contextId;
	this->myOrigin = parent;
	this->myClone = childClone;
	this->goalIdx = parent->goalIdx;
	for (int i = 0; i < NUM_GOAL; i++)
		this->goalSeq[i] = parent->goalSeq[i];

	this->data = parent->data;
	this->dataCopy = parent->dataCopy;

	this->data.agentPtr = this;
}

namespace clone {
	__global__ void stepKernel(SocialForceClone *c) {
		int index = threadIdx.x + blockIdx.x * blockDim.x;
		if (index < c->ap->numElem)
			c->ap->agentPtrArray[index]->step();
	}
}

void SocialForceClone::step(int stepCount) {
	alterGate(stepCount);
	//clone::stepKernel << <GRID_SIZE(apHost->numElem), BLOCK_SIZE >> >(this);
}

void SocialForceClone::alterGate(int stepCount) {
	for (int i = 0; i < NUM_PARAM; i++) {
		if (cloneParams[i] == stepCount)
			gates[i].init(0, 0, 0, 0);
	}
}

class SocialForceSimApp {
public:
	SocialForceClone **cAll;
	int paintId = 0;
	int totalClone = 32;
	int stepCount = 0;
	int rootCloneId = 0;
	int **cloneTree;

	int initSimClone() {
		srand(0);

		cAll = new SocialForceClone*[totalClone];
		cloneTree = new int*[2];
		int j = 0;

		int cloneParams[NUM_PARAM];
		for (int i = 0; i < NUM_PARAM; i++) {
			cloneParams[i] = rand() % NUM_STEP;
		}

		for (int i = 0; i < totalClone; i++) {
			cAll[i] = new SocialForceClone(i, cloneParams);
		}

		SocialForceAgent *agents = cAll[rootCloneId]->ap->agentArray;
		SocialForceAgent **context = cAll[rootCloneId]->context;

		for (int i = 0; i < NUM_CAP; i++) {
			//float rx = (float)rand() / (float)RAND_MAX;
			//float ry = (float)rand() / (float)RAND_MAX;
			float rx = (float)(i / 32) / (float)32;
			float ry = (float)(i % 32) / (float)32;
			agents[i].myClone = cAll[rootCloneId];
			agents[i].contextId = i;
			agents[i].color = cAll[rootCloneId]->color;
			agents[i].init(i);
			context[i] = &agents[i];
		}

		cAll[rootCloneId]->ap->numElem = NUM_CAP;
		for (int j = 0; j < NUM_CAP; j++)
			cAll[rootCloneId]->cloneFlag[j] = true;

		mst();

		return EXIT_SUCCESS;
	}
	bool cloningCondition(SocialForceAgent *agent, bool *childTakenMap,
		SocialForceClone *parentClone, SocialForceClone *childClone) {

		// if agent has been cloned?
		if (childClone->cloneFlag[agent->contextId] == true)
			return false;

		// active cloning condition
		double2 &loc = agent->data.loc;
		for (int i = 0; i < NUM_PARAM; i++) {
			obstacleLine g1 = parentClone->gates[i];
			obstacleLine g2 = childClone->gates[i];
			obstacleLine g0 = obstacleLine(0, 0, 0, 0);
			if (g1 != g2) {
				obstacleLine gate = (g1 != g0) ? g1 : g2;
				if (gate.pointToLineDist(loc) < 6)
					return true;
			}
		}

		// passive cloning condition
#define MY_MAX(a, b) (a > b ? a : b)
#define MY_MIN(a, b) (a < b ? a : b)
		int minx = MY_MAX((loc.x - RADIUS_I) / CELL_DIM, 0);
		int miny = MY_MAX((loc.y - RADIUS_I) / CELL_DIM, 0);
		int maxx = MY_MIN((loc.x + RADIUS_I) / CELL_DIM, NUM_CELL - 1);
		int maxy = MY_MIN((loc.y + RADIUS_I) / CELL_DIM, NUM_CELL - 1);
		for (int i = minx; i <= maxx; i++)
			for (int j = miny; j <= maxy; j++)
				if (childTakenMap[i * NUM_CELL + j])
					return true;

		// pass all the check, don't need to be cloned
		return false;
	}
	void performClone(SocialForceClone *parentClone, SocialForceClone *childClone) {
		childClone->parentCloneid = parentClone->cloneid;

		// 1. copy the context of parent clone
		memcpy(childClone->context, parentClone->context, NUM_CAP * sizeof(void*));

		// 2. update the context with agents of its own
		for (int i = 0; i < childClone->ap->numElem; i++) {
			SocialForceAgent *agent = childClone->ap->agentPtrArray[i];
			childClone->context[agent->contextId] = agent;
		}

		// 3. construct passive cloning map
		double2 dim = make_double2(ENV_DIM, ENV_DIM);
		memset(childClone->takenMap, 0, sizeof(bool) * NUM_CELL * NUM_CELL);
		for (int i = 0; i < childClone->ap->numElem; i++) {
			const SocialForceAgent &agent = *childClone->ap->agentPtrArray[i];
			int takenId = agent.data.loc.x / CELL_DIM;
			takenId = takenId * NUM_CELL + agent.data.loc.y / CELL_DIM;
			childClone->takenMap[takenId] = true;
		}

		// 4. perform active and passive cloning (in cloningCondition checking)
		for (int i = 0; i < NUM_CAP; i++) {
			SocialForceAgent *agent = parentClone->context[i];
			if (cloningCondition(agent, childClone->takenMap, parentClone, childClone)) {
				SocialForceAgent &childAgent = *childClone->ap->agentPtrArray[childClone->ap->numElem];
				childClone->ap->takenFlags[childClone->ap->numElem] = true;
				childAgent.initNewClone(agent, childClone);
				childClone->context[childAgent.contextId] = &childAgent;
				childClone->cloneFlag[childAgent.contextId] = true;
				childClone->ap->numElem++;
			}
		}
	}
	void compareAndEliminate(SocialForceClone *parentClone, SocialForceClone *childClone) {
		wchar_t message[20];
		for (int i = 0; i < childClone->ap->numElem; i++) {
			SocialForceAgent &childAgent = *childClone->ap->agentPtrArray[i];
			SocialForceAgent &parentAgent = *childAgent.myOrigin;
			if (length(childAgent.dataCopy.velocity - parentAgent.dataCopy.velocity) == 0 &&
				length(childAgent.dataCopy.loc - parentAgent.dataCopy.loc) == 0) {
				childClone->ap->takenFlags[i] = false;
				childClone->cloneFlag[childAgent.contextId] = false;
			}
			/*else {
			if (childClone->cloneid == 4) {
			swprintf_s(message, 20, L"not false: %d\n", i);
			OutputDebugString(message);
			}
			}*/
		}
		childClone->ap->reorder();
	}
	void proc(int p, int c, bool o, char *s) {
		performClone(cAll[p], cAll[c]);
		cAll[c]->step(stepCount);
		if (o) {
			if (stepCount < 800)
				cAll[c]->output(stepCount, s);
		}
		//cAll[c]->output2(stepCount, s);
		compareAndEliminate(cAll[p], cAll[c]);
	}

	void swap(int **cloneTree, int a, int b) {
		int t1 = cloneTree[0][a];
		cloneTree[0][a] = cloneTree[0][b];
		cloneTree[0][b] = t1;

		t1 = cloneTree[1][a];
		cloneTree[1][a] = cloneTree[1][b];
		cloneTree[1][b] = t1;
	}

	void quickSort(int **cloneTree, int l, int r) {
		if (l == r)
			return;
		int pi = l + rand() % (r - l);
		swap(cloneTree, l, pi);
		int pivot = cloneTree[0][l];

		int i = l + 1, j = l + 1;
		for (; j < r; j++) {
			if (cloneTree[0][j] < pivot) {
				swap(cloneTree, i, j);
				i++;
			}
		}
		swap(cloneTree, l, i - 1);
		quickSort(cloneTree, l, i - 1);
		quickSort(cloneTree, i, r);
	}

	void mst() {
		// clone diff matrix
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
				wchar_t message[20];
				swprintf_s(message, 20, L"%d ", cloneDiff[i][j]);
				OutputDebugString(message);
			}
			OutputDebugString(L"\n");
		}
		int *parent = cloneTree[0] = new int[totalClone];
		int *child = cloneTree[1] = new int[totalClone];
		int *key = new int[totalClone];
		bool *mstSet = new bool[totalClone];

		for (int i = 0; i < totalClone; i++)
			child[i] = i, key[i] = INT_MAX, mstSet[i] = false;

		key[0] = 0;
		parent[0] = -1;
		child[0] = 0;

		int count = 0;
		while (count++ < totalClone - 1) {
			int minKey = INT_MAX;
			int minIdx;
			for (int j = 0; j < totalClone; j++)
				if (mstSet[j] == false && key[j] < minKey)
					minKey = key[j], minIdx = j;
			mstSet[minIdx] = true;

			for (int j = 0; j < totalClone; j++)
				if (cloneDiff[minIdx][j] && mstSet[j] == false && cloneDiff[minIdx][j] < key[j])
					parent[j] = minIdx, key[j] = cloneDiff[minIdx][j];
		}

		quickSort(cloneTree, 0, totalClone);

		for (int i = 0; i < totalClone; i++) {
			wchar_t message[20];
			swprintf_s(message, 20, L"%d - %d: %d\n", cloneTree[0][i], cloneTree[1][i], cloneDiff[i][parent[i]]);
			OutputDebugString(message);
		}

		delete mstSet;
		delete key;
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
		//stepApp1(0);
		stepApp5(0);
	}
};

