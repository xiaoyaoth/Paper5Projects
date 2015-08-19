#include "cuda_runtime.h"
#include <fstream>
#include "SocialForceGPU.h"

__global__ void testFunc() {

}

extern "C"
void runTest() {
	testFunc << <32, 32 >> >();
}

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

void hookPointerAndData(SocialForceAgent** agentPtrArray, SocialForceAgent* agentArray, int numCap);
void cloneStepOnGPU(SocialForceClone *c);

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
__device__ void SocialForceAgent::init(int idx) {
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

__global__ void initRootCloneKernel(SocialForceClone* c) {
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	if (idx < c->ap->numElem) {
		c->ap->agentArray[idx].init(idx);
		c->context[idx] = &c->ap->agentArray[idx];
	}
}

extern "C"
void initRootClone(SocialForceClone* cHost, SocialForceClone* cDev) {
	int gSize = GRID_SIZE(cHost->apHost->numElem);
	initRootCloneKernel << <gSize, BLOCK_SIZE >> >(cDEv);
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


void SocialForceClone::step(int stepCount) {
	alterGate(stepCount);
	cloneStepOnGPU(this);
}

void SocialForceClone::alterGate(int stepCount) {
	for (int i = 0; i < NUM_PARAM; i++) {
		if (cloneParams[i] == stepCount)
			gates[i].init(0, 0, 0, 0);
	}
}

bool SocialForceSimApp::cloningCondition(SocialForceAgent *agent, bool *childTakenMap,
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
void SocialForceSimApp::performClone(SocialForceClone *parentClone, SocialForceClone *childClone) {
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
void SocialForceSimApp::compareAndEliminate(SocialForceClone *parentClone, SocialForceClone *childClone) {
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
void SocialForceSimApp::proc(int p, int c, bool o, char *s) {
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

void SocialForceSimApp::mst() {
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

namespace clone {
	__global__ void stepKernel(SocialForceClone *c) {
		int index = threadIdx.x + blockIdx.x * blockDim.x;
		if (index < c->ap->numElem)
			c->ap->agentPtrArray[index]->step();
	}
}


void cloneStepOnGPU(SocialForceClone *c) {
	clone::stepKernel << <GRID_SIZE(c->apHost->numElem), BLOCK_SIZE >> >(c);
}

namespace APUtil {
	__global__ void hookPointerAndDataKernel(SocialForceAgent** agentPtrArray, SocialForceAgent* agentArray, int numCap) {
		int index = threadIdx.x + blockIdx.x * blockDim.x;
		if (index < numCap) agentPtrArray[index] = &agentArray[index];
	}
};

void hookPointerAndData(SocialForceAgent** agentPtrArray, SocialForceAgent* agentArray, int numCap) {
	int gSize = GRID_SIZE(numCap);
	APUtil::hookPointerAndDataKernel << <gSize, BLOCK_SIZE >> >(agentPtrArray, agentArray, numCap);
}