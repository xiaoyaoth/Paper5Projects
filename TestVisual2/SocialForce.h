#include <vector>
#include <math.h>
#include <ctime>
#include <Windows.h>

using namespace std;

#define DIST(ax, ay, bx, by) sqrt((ax-bx)*(ax-bx)+(ay-by)*(ay-by))

struct double2 {
	float x;
	float y;
	double2(float xx, float yy) : x(xx), y(yy) {}
	double2(){};
};

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

/*
class SimpleAgent {
public:
double2 loc;
double2 locNext;
double2 locDefault;
int contextId;
Color color;
};
*/

struct obstacleLine
{
	double sx;
	double sy;
	double ex;
	double ey;

	void init(double sxx, double syy, double exx, double eyy)
	{
		sx = sxx;
		sy = syy;
		ex = exx;
		ey = eyy;
	}

	double pointToLineDist(double2 loc)
	{
		double a, b;
		return this->pointToLineDist(loc, a, b);
	}

	double pointToLineDist(double2 loc, double &crx, double &cry)
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

	int intersection2LineSeg(double p0x, double p0y, double p1x, double p1y, double &ix, double &iy)
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
};

inline float dot(const double2& a, const double2& b)
{
	return a.x * b.x + a.y * b.y;
}

inline float length(const double2& v)
{
	return sqrtf(dot(v, v));
}

inline double2 operator-(const double2& a, const double2& b)
{
	return double2(a.x - b.x, a.y - b.y);
}

#define	tao 0.5
#define	A 2000
#define	B 0.1
#define	k1 (1.2 * 100000)
#define k2 (2.4 * 100000)
#define	maxv 3

#define NUM_CAP 128
#define NUM_PARAM 4
#define CELL_DIM 4
#define ENV_DIM 64
#define RADIUS_I 5

#define NUM_WALLS 2

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
	//__device__ void putDataInSmem(GAgent *ag);
} SocialForceAgentData;

class SocialForceAgent {
public:
	SocialForceClone *myClone;
	//int id;
	int cloneid;
	SocialForceAgent *myOrigin;
	SocialForceAgentData data;
	SocialForceAgentData dataCopy;

	Color color;
	int contextId;
	//double gateSize;

	SocialForceAgent() {

	}

	double correctCrossBoader(double val, double limit);
	void computeIndivSocialForceRoom(const SocialForceAgentData &myData, const SocialForceAgentData &otherData, double2 &fSum);
	void computeForceWithWall(const SocialForceAgentData &dataLocal, obstacleLine &wall, const int &cMass, double2 &fSum);
	void computeWallImpaction(const SocialForceAgentData &dataLocal, obstacleLine &wall, const double2 &newVelo, const double &tick, double &mint);
	void computeDirection(const SocialForceAgentData &dataLocal, double2 &dvt);
	void computeSocialForceRoom(SocialForceAgentData &dataLocal, double2 &fSum);
	void chooseNewGoal(const double2 &newLoc, double epsilon, double2 &newGoal);
	void step();
	void init(int idx);
};
class SocialForceClone {
public:
	SocialForceAgent *agents;
	int numElem;
	SocialForceAgent **context;
	int *pv;
	Color color;
	obstacleLine walls[2];

	SocialForceClone() {
		agents = new SocialForceAgent[NUM_CAP];
		numElem = 0;
		context = new SocialForceAgent*[NUM_CAP];
		pv = new int[NUM_PARAM];
		color = Color();
		walls[0].init(0.75 * ENV_DIM, -0.1 * ENV_DIM, 0.75 * ENV_DIM, 0.45 * ENV_DIM);
		walls[1].init(0.75 * ENV_DIM, 0.5 * ENV_DIM, 0.75 * ENV_DIM, 1.1 * ENV_DIM);
	}

	void step();
};


double SocialForceAgent::correctCrossBoader(double val, double limit)
{
	if (val > limit)
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
void SocialForceAgent::computeForceWithWall(const SocialForceAgentData &dataLocal, obstacleLine &wall, const int &cMass, double2 &fSum) {
	double diw, crx, cry;
	const double2 &loc = dataLocal.loc;

	diw = wall.pointToLineDist(loc, crx, cry);
	double virDiw = DIST(loc.x, loc.y, crx, cry);

	//if (stepCount == MONITOR_STEP && this->id == 263) {
	//	printf("dist: %f, cross: (%f, %f)\n", diw, crx, cry);
	//}

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
void SocialForceAgent::computeWallImpaction(const SocialForceAgentData &dataLocal, obstacleLine &wall, const double2 &newVelo, const double &tick, double &mint){
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
void SocialForceAgent::computeDirection(const SocialForceAgentData &dataLocal, double2 &dvt) {
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
void SocialForceAgent::computeSocialForceRoom(SocialForceAgentData &dataLocal, double2 &fSum) {
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
void SocialForceAgent::chooseNewGoal(const double2 &newLoc, double epsilon, double2 &newGoal) {
	double2 oldGoal = newGoal;
	double2 center = double2(ENV_DIM / 2, ENV_DIM / 2);
	if (newLoc.x < center.x && newLoc.y < center.y) {
		if ((newLoc.x + epsilon >= 0.5 * ENV_DIM)
			//&& (newLoc.y + epsilon > 0.3 * ENV_DIM - gateSize) 
			//&& (newLoc.y - epsilon < 0.3 * ENV_DIM + gateSize)
			)
		{
			newGoal.x = 0.9 * ENV_DIM;
			newGoal.y = 0.3 * ENV_DIM;
		}
	}
	else if (newLoc.x > center.x && newLoc.y < center.y) {
		if ((newLoc.x + epsilon >= 0.9 * ENV_DIM)
			//&& (newLoc.y + epsilon > 0.3 * ENV_DIM - gateSize) 
			//&& (newLoc.y - epsilon < 0.3 * ENV_DIM + gateSize)
			)
		{
			//newGoal.x = ENV_DIM;
			//newGoal.y = 0;
		}
	}
	else if (newLoc.x < center.x && newLoc.y > center.y) {
		if ((newLoc.y - epsilon <= 0.5 * ENV_DIM)
			//&& (newLoc.x + epsilon > 0.3 * ENV_DIM - gateSize) 
			//&& (newLoc.x - epsilon < 0.3 * ENV_DIM + gateSize)
			)
		{
			newGoal.x = 0.5 * ENV_DIM;
			newGoal.y = 0.3 * ENV_DIM;
		}
	}
	else if (newLoc.x > center.x && newLoc.y > center.y) {
		if ((newLoc.x - epsilon <= 0.5 * ENV_DIM)
			//&& (newLoc.y + epsilon > 0.7 * ENV_DIM - gateSize) 
			//&& (newLoc.y - epsilon < 0.7 * ENV_DIM + gateSize)
			)
		{
			newGoal.x = 0.3 * ENV_DIM;
			newGoal.y = 0.5 * ENV_DIM;
		}
	}
}
void SocialForceAgent::step(){
	double cMass = 100;

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

	//compute force with wall
	for (int i = 0; i < NUM_WALLS; i++) {
		obstacleLine wall = myClone->walls[i];
		//alterWall(wall, i);
		computeForceWithWall(data, wall, cMass, fSum);
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

	newVelo.x *= mint;
	newVelo.y *= mint;
	newLoc.x += newVelo.x * tick;
	newLoc.y += newVelo.y * tick;

	double goalTemp = goal.x;

	//chooseNewGoal(newLoc, mass / cMass, newGoal);

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

	this->cloneid = 0;

	this->myOrigin = NULL;

	SocialForceAgentData & dataLocal = this->data; //= &sfModel->originalAgents->dataArray[dataSlot];

	dataLocal.agentPtr = this;
	dataLocal.loc.x = (float)rand() / (float)RAND_MAX * ENV_DIM - 0.1;
	dataLocal.loc.y = (float)rand() / (float)RAND_MAX * ENV_DIM - 0.1;
	dataLocal.velocity.x = 2;//4 * (this->random->uniform()-0.5);
	dataLocal.velocity.y = 2;//4 * (this->random->uniform()-0.5);

	dataLocal.v0 = 2;
	dataLocal.mass = 50;
	dataLocal.numNeighbor = 0;

	dataLocal.goal = double2(ENV_DIM, 0.5 * ENV_DIM);
	//chooseNewGoal(dataLocal.loc, 0, dataLocal.goal);

	this->dataCopy = dataLocal;
}
/*
void initNewClone(const SocialForceAgent &agent,
SocialForceAgent *originPtr,
int dataSlot,
SocialForceClone *clone,
int cloneid)
{
this->myWorld = clone->clonedWorld;
this->color = clone->color;

this->cloneid = cloneid;
this->id = agent.id;
for (int i = 0; i < NUM_GATES; i++) {
this->cloneidArray[i] = clone->cloneidArray[i];
this->flagCloning[i] = 0; // or originPtr->flagCloning[i]?
this->flagCloned[i] = 0; // or originPtr->flagCloned[i]?
}
this->myOrigin = originPtr;

SocialForceAgentData dataLocal, dataCopyLocal;

dataLocal = *(SocialForceAgentData*)agent.data;
dataCopyLocal = *(SocialForceAgentData*)agent.dataCopy;

dataLocal.agentPtr = this;
dataCopyLocal.agentPtr = this;

if (stepCount % 2 == 0) {
this->data = clone->agents->dataInSlot(dataSlot);
this->dataCopy = clone->agents->dataCopyInSlot(dataSlot);
}
else {
this->data = clone->agents->dataCopyInSlot(dataSlot);
this->dataCopy = clone->agents->dataInSlot(dataSlot);
}

*(SocialForceAgentData*)this->data = dataLocal;
*(SocialForceAgentData*)this->dataCopy = dataCopyLocal;
}
*/

void SocialForceClone::step() {
	for (int i = 0; i < numElem; i++)
		agents[i].step();
}

class SocialForceSimApp {
public:
	SocialForceClone **cAll;
	SocialForceAgent *agents;
	SocialForceAgent **context;
	int paintId;

	/*
	void step(SocialForceClone *clone) {
	double2 c1(32, 32), c2(32, 96), c3(96, 32), c4(96, 96);
	for (int i = 0; i < clone->numElem; i++) {
	SimpleAgent &agent;// = clone->agents[i];
	if (clone->pv[0] == 1 && length(agent.loc - c1) < 4) {
	if (agent.loc.y < c1.y)
	agent.locNext.y = agent.loc.y - 0.2;
	else
	agent.locNext.y = agent.loc.y + 0.2;
	continue;
	}
	if (clone->pv[1] == 1 && length(agent.loc - c2) < 4) {
	if (agent.loc.y < c2.y)
	agent.locNext.y = agent.loc.y - 0.2;
	else
	agent.locNext.y = agent.loc.y + 0.2;
	continue;
	}
	if (clone->pv[2] == 1 && length(agent.loc - c3) < 4) {
	if (agent.loc.y < c3.y)
	agent.locNext.y = agent.loc.y - 0.2;
	else
	agent.locNext.y = agent.loc.y + 0.2;
	continue;
	}
	if (clone->pv[3] == 1 && length(agent.loc - c4) < 4) {
	if (agent.loc.y < c4.y)
	agent.locNext.y = agent.loc.y - 0.2;
	else
	agent.locNext.y = agent.loc.y + 0.2;
	continue;
	}
	if (agent.loc.y > agent.locDefault.y)
	agent.locNext.y = agent.loc.y - 0.2;
	else if (agent.loc.y < agent.locDefault.y)
	agent.locNext.y = agent.loc.y + 0.2;
	agent.locNext.x = agent.loc.x + 0.2;
	}
	}
	*/

	void swapAll(SocialForceClone *clone) {
		for (int i = 0; i < clone->numElem; i++) {
			SocialForceAgent &agent = clone->agents[i];
			agent.data = agent.dataCopy;
		}
	}

	/*
	bool cloningCondition(SocialForceAgent *agent, bool *childTakenMap,
	SocialForceClone *parentClone, SocialForceClone *childClone) {
	double2 c1(32, 32), c2(32, 96), c3(96, 32), c4(96, 96);
	// if agent has been cloned?
	if (agent != childClone->context[agent->contextId])
	return false;

	// active cloning condition
	double2 &loc = agent->loc;
	if (parentClone->pv[0] != childClone->pv[0])
	if (length(loc - c1) < 6) return true;
	if (parentClone->pv[1] != childClone->pv[1])
	if (length(loc - c2) < 8) return true;
	if (parentClone->pv[2] != childClone->pv[2])
	if (length(loc - c3) < 10) return true;
	if (parentClone->pv[3] != childClone->pv[3])
	if (length(loc - c4) < 12) return true;

	// passive cloning condition

	//int minx = max((loc.x - RADIUS_I) / (ENV_DIM / CELL_DIM), 0);
	//int miny = max((loc.y - RADIUS_I) / (ENV_DIM / CELL_DIM), 0);
	//int maxx = min((loc.x + RADIUS_I) / (ENV_DIM / CELL_DIM), CELL_DIM - 1);
	//int maxy = min((loc.y + RADIUS_I) / (ENV_DIM / CELL_DIM), CELL_DIM - 1);
	//for (int i = minx; i <= maxx; i++)
	//for (int j = miny; j <= maxy; j++)
	//if (childTakenMap[i * CELL_DIM + j])
	//return true;


	// pass all the check, don't need to be cloned
	return false;
	}
	*/

	/*
	void performClone(SocialForceClone *parentClone, SocialForceClone *childClone) {
	// 1. copy the context of parent clone
	memcpy(childClone->context, parentClone->context, NUM_CAP * sizeof(void*));

	// 2. update the context with agents of its own
	for (int i = 0; i < childClone->numElem; i++) {
	SocialForceAgent *agent = &childClone->agents[i];
	childClone->context[agent->contextId] = agent;
	}

	// 3. Perform cloning
	// 3.1 construct passive cloning map
	double2 dim(ENV_DIM, ENV_DIM);
	bool *takenMap = new bool[CELL_DIM * CELL_DIM];
	memset(takenMap, 0, sizeof(bool) * CELL_DIM * CELL_DIM);
	for (int i = 0; i < childClone->numElem; i++) {
	const SocialForceAgent &agent = childClone->agents[i];
	int takenId = agent.loc.x / ENV_DIM;
	takenId = takenId * CELL_DIM + agent.loc.y / ENV_DIM;
	takenMap[takenId] = true;
	}

	// 3.2 perform cloning
	for (int i = 0; i < NUM_CAP; i++) {
	SocialForceAgent *agent = parentClone->context[i];
	if (cloningCondition(agent, takenMap, parentClone, childClone)) {
	SocialForceAgent &childAgent = childClone->agents[childClone->numElem];
	childAgent = *agent;
	childAgent.color = childClone->color;
	childClone->context[childAgent.contextId] = &childAgent;
	childClone->numElem++;
	}
	}
	delete takenMap;
	}
	*/

	int initSimClone() {
		srand(time(NULL));
		agents = new SocialForceAgent[NUM_CAP];
		context = new SocialForceAgent*[NUM_CAP];
		paintId = 0;

		cAll = new SocialForceClone*[16];
		char message[200];
		for (int i = 0; i < 16; i++) {
			cAll[i] = new SocialForceClone();
			cAll[i]->pv[0] = i & 1;
			cAll[i]->pv[1] = (i >> 1) & 1;
			cAll[i]->pv[2] = (i >> 2) & 1;
			cAll[i]->pv[3] = (i >> 3) & 1;
			sprintf_s(message, 200, "[%d, %d, %d, %d]\n", cAll[i]->pv[0],
				cAll[i]->pv[1], cAll[i]->pv[2], cAll[i]->pv[3]);
		}
		OutputDebugStringA((LPCSTR)message);

		for (int i = 0; i < NUM_CAP; i++) {

			//float rx = (float)rand() / (float)RAND_MAX;
			//float ry = (float)rand() / (float)RAND_MAX;
			float rx = (float)(i / 32) / (float)32;
			float ry = (float)(i % 32) / (float)32;
			agents[i].myClone = cAll[0];
			agents[i].contextId = i;
			agents[i].color = cAll[0]->color;
			agents[i].init(i);
			context[i] = &agents[i];
		}

		return EXIT_SUCCESS;
	}

	void stepApp() {
		//cAll[15]->context = context;
		//performClone(cAll[15], cAll[10]);
		//cout << cAll[10]->numElem << " ";
		//for (int i = 0; i < 16; i++) {

		//int stepCount = 0;

		int i = 0;
		//for (int i = 0; i < 16; i++) {
		cAll[i]->context = context;
		cAll[i]->agents = agents;
		cAll[i]->numElem = NUM_CAP;

		//while (stepCount++ < 100) {
		cAll[i]->step();
		swapAll(cAll[i]);
		/*for (int j = 0; j < 16; j++) {
			performClone(cAll[i], cAll[j]);
			step(cAll[j]);
			cout << cAll[j]->numElem << " ";
			}
			for (int j = 0; j < 16; j++) {
			swapAll(cAll[j]);
			}
			cout << endl;
			*/

		//}
		//}

		/* start processing */
		//int stepCount = 0;
		//cAll[0]->context = context;
		//cAll[0]->agents = agents;
		//cAll[0]->numElem = NUM_CAP;
		//while (stepCount++ < 100) {
		//	step(cAll[0]);
		//	performClone(cAll[0], cAll[1]);
		//	step(cAll[1]);
		//	performClone(cAll[1], cAll[2]);
		//	step(cAll[2]);

		//	cout << cAll[2]->numElem << " " << cAll[2]->agents[0].loc.x << " "
		//		<< cAll[2]->agents[0].loc.y << endl;
		//	
		//	swapAll(cAll[0]);
		//	swapAll(cAll[1]);
		//	swapAll(cAll[2]);
		//}
	}
};
