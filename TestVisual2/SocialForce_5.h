#include <vector>
#include <math.h>
#include <ctime>
#include <Windows.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include "inc\helper_math.h"
#include <random>

using namespace std;

#define DIST(ax, ay, bx, by) sqrt((ax-bx)*(ax-bx)+(ay-by)*(ay-by))

/*
struct double2 {
	float x;
	float y;
	double2(float xx, float yy) : x(xx), y(yy) {}
	double2(){};
};*/
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

	bool operator == (const obstacleLine & other) {
		return sx == other.sx && sy == other.sy && ex == other.ex && ey == other.ey;
	}

	bool operator != (const obstacleLine & other) {
		return !(*this == other);
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
	return make_double2(a.x - b.x, a.y - b.y);
}

int g_stepCount = 0;

#define	tao 0.5
#define	A 2000
#define	B 0.1
#define	k1 (1.2 * 100000)
#define k2 (2.4 * 100000) 
#define	maxv 3

#define NUM_CAP 32
#define NUM_PARAM 8
#define NUM_STEP 500
#define NUM_GOAL 3
#define ENV_DIM 64
#define NUM_CELL 16
#define CELL_DIM 4
#define RADIUS_I 6

#define NUM_WALLS 1

default_random_engine randGen;
uniform_real_distribution<double> distr(0.0, 1.0);

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
	SocialForceAgent *myOrigin;
	SocialForceAgentData data;
	SocialForceAgentData dataCopy;
	double2 goalSeq[NUM_GOAL];
	int goalIdx = 0;

	uchar4 color;
	int contextId;
	//double gateSize;

	double correctCrossBoader(double val, double limit);
	void computeIndivSocialForceRoom(const SocialForceAgentData &myData, const SocialForceAgentData &otherData, double2 &fSum);
	void computeForceWithWall(const SocialForceAgentData &dataLocal, obstacleLine &wall, const int &cMass, double2 &fSum);
	void computeWallImpaction(const SocialForceAgentData &dataLocal, obstacleLine &wall, const double2 &newVelo, const double &tick, double &mint);
	void computeDirection(const SocialForceAgentData &dataLocal, double2 &dvt);
	void computeSocialForceRoom(SocialForceAgentData &dataLocal, double2 &fSum);
	void chooseNewGoal(const double2 &newLoc, double epsilon, double2 &newGoal);
	void step();
	void init(int idx);
	void initNewClone(SocialForceAgent *agent, SocialForceClone *clone);
};
class AgentPool {
public:
	SocialForceAgent *agentArray;
	SocialForceAgent **agentPtrArray;
	bool *takenFlags;

	AgentPool(int numCap) {
		agentArray = new SocialForceAgent[numCap];
		agentPtrArray = new SocialForceAgent*[numCap];
		takenFlags = new bool[numCap];
		for (int i = 0; i < numCap; i++) {
			agentPtrArray[i] = &agentArray[i];
			takenFlags[i] = 0;
		}
	}

	int reorder(int numElem) {
		int l = 0; int r = numElem;
		int i = l, j = l;
		for (; j < r; j++) {
			if (takenFlags[j] == true) {
				swap<SocialForceAgent*>(agentPtrArray, i, j);
				swap<bool>(takenFlags, i, j);
				i++;
			}
		}
		return i;
	}

	template<class T>
	inline void swap(T * ar, int a, int b) {
		T t1 = ar[a];
		ar[a] = ar[b];
		ar[b] = t1;
	}
};

class SocialForceClone {
public:
	AgentPool *ap;
	int numElem;
	SocialForceAgent **context;
	bool *cloneFlag;
	int cloneParams[NUM_PARAM];
	obstacleLine walls[NUM_WALLS];
	obstacleLine gates[NUM_PARAM];
	bool takenMap[NUM_CELL * NUM_CELL];
	
	uchar4 color;
	int cloneid;
	int parentCloneid;

	fstream fout;

	SocialForceClone(int id, int pv1[NUM_PARAM]) {
		numElem = 0;
		cloneid = id;
		ap = new AgentPool(NUM_CAP);
		//agents = new SocialForceAgent[NUM_CAP];
		context = new SocialForceAgent*[NUM_CAP];
		cloneFlag = new bool[NUM_CAP];
		memset(context, 0, sizeof(void*) * NUM_CAP);
		memset(cloneFlag, 0, sizeof(bool) * NUM_CAP);
		color.x = rand() % 255; color.y = rand() % 255; color.z = rand() % 255;

		double ps = 0.025; double dd = 0.25;
		walls[0].init(0, 0, 0, 0);
		
		for (int i = 0; i < NUM_PARAM; i++) {
			cloneParams[0] = 0;
			gates[i].init(0, 0, 0, 0);
		}

		id = 3;
		if (id == 1){
			for (int i = 0; i < NUM_PARAM / 2; i++)
				cloneParams[i] = 1;
			gates[0].init((0.30 - ps) * ENV_DIM, (0.50 - ps) * ENV_DIM, (0.30 + ps) * ENV_DIM, (0.50 - ps) * ENV_DIM);
			gates[2].init((0.30 - ps) * ENV_DIM, (0.50 + ps) * ENV_DIM, (0.30 + ps) * ENV_DIM, (0.50 + ps) * ENV_DIM);
			gates[1].init((0.30 - ps) * ENV_DIM, (0.50 - ps) * ENV_DIM, (0.30 - ps) * ENV_DIM, (0.50 + ps) * ENV_DIM);
			gates[3].init((0.30 + ps) * ENV_DIM, (0.50 - ps) * ENV_DIM, (0.30 + ps) * ENV_DIM, (0.50 + ps) * ENV_DIM);
		}
		else if (id == 2) {
			for (int i = 4; i < NUM_PARAM; i++)
				cloneParams[i] = 1;
			gates[4].init((0.70 - ps) * ENV_DIM, (0.50 - ps) * ENV_DIM, (0.70 + ps) * ENV_DIM, (0.50 - ps) * ENV_DIM);
			gates[5].init((0.70 - ps) * ENV_DIM, (0.50 + ps) * ENV_DIM, (0.70 + ps) * ENV_DIM, (0.50 + ps) * ENV_DIM);
			gates[6].init((0.70 - ps) * ENV_DIM, (0.50 - ps) * ENV_DIM, (0.70 - ps) * ENV_DIM, (0.50 + ps) * ENV_DIM);
			gates[7].init((0.70 + ps) * ENV_DIM, (0.50 - ps) * ENV_DIM, (0.70 + ps) * ENV_DIM, (0.50 + ps) * ENV_DIM);
		}
		else if (id == 3) {
			for (int i = 0; i < NUM_PARAM; i++)
				cloneParams[i] = 1;
			gates[0].init((0.30 - ps) * ENV_DIM, (0.50 - ps) * ENV_DIM, (0.30 + ps) * ENV_DIM, (0.50 - ps) * ENV_DIM);
			gates[2].init((0.30 - ps) * ENV_DIM, (0.50 + ps) * ENV_DIM, (0.30 + ps) * ENV_DIM, (0.50 + ps) * ENV_DIM);
			gates[1].init((0.30 - ps) * ENV_DIM, (0.50 - ps) * ENV_DIM, (0.30 - ps) * ENV_DIM, (0.50 + ps) * ENV_DIM);
			gates[3].init((0.30 + ps) * ENV_DIM, (0.50 - ps) * ENV_DIM, (0.30 + ps) * ENV_DIM, (0.50 + ps) * ENV_DIM);

			gates[4].init((0.70 - ps) * ENV_DIM, (0.50 - ps) * ENV_DIM, (0.70 + ps) * ENV_DIM, (0.50 - ps) * ENV_DIM);
			gates[5].init((0.70 - ps) * ENV_DIM, (0.50 + ps) * ENV_DIM, (0.70 + ps) * ENV_DIM, (0.50 + ps) * ENV_DIM);
			gates[6].init((0.70 - ps) * ENV_DIM, (0.50 - ps) * ENV_DIM, (0.70 - ps) * ENV_DIM, (0.50 + ps) * ENV_DIM);
			gates[7].init((0.70 + ps) * ENV_DIM, (0.50 - ps) * ENV_DIM, (0.70 + ps) * ENV_DIM, (0.50 + ps) * ENV_DIM);
		}
		/*gates[0].init(0, 0, 0, 0);
		gates[1].init(0, 0, 0, 0);
		gates[2].init(0, 0, 0, 0);
		gates[3].init(0, 0, 0, 0);*/
	}
	void step(int stepCount);
	void alterGate(int stepCount);
	void swap() {
		for (int i = 0; i < numElem; i++) {
			SocialForceAgent &agent = *ap->agentPtrArray[i];
			agent.data = agent.dataCopy;
		}
	}
	void output(int stepCount, char *s) {
		char filename[128];
		sprintf_s(filename, 128, "clone%d_%s.txt", cloneid, s);
		if (stepCount == 1)
			fout.open(filename, fstream::out);
		else
			fout.open(filename, fstream::app);
		fout << "========== stepCount: " << stepCount << " ==========="<<endl;
		int outprec = 20;
		for (int i = 0; i < NUM_CAP; i++) {
			fout << context[i]->contextId << " [";
			fout << setprecision(outprec) << context[i]->data.loc.x << ",";
			fout << setprecision(outprec) << context[i]->data.loc.y << "] [";
			fout << setprecision(outprec) << context[i]->data.velocity.x << ", ";
			fout << setprecision(outprec) << context[i]->data.velocity.y << "] [";
			fout << setprecision(outprec) << context[i]->dataCopy.loc.x << ",";
			fout << setprecision(outprec) << context[i]->dataCopy.loc.y << "] [";
			fout << setprecision(outprec) << context[i]->dataCopy.velocity.x << ", ";
			fout << setprecision(outprec) << context[i]->dataCopy.velocity.y << "] ";
			fout << context[i]->data.numNeighbor << " ";
			fout << endl;
			fout.flush();
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
		fout << numElem<<endl;
		fout.close();
	}
};
double SocialForceAgent::correctCrossBoader(double val, double limit)
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
void SocialForceAgent::computeForceWithWall(const SocialForceAgentData &dataLocal, obstacleLine &wall, const int &cMass, double2 &fSum) {
	double2 wl = make_double2(wall.ex - wall.sx, wall.ey - wall.sy);
	if (length(wl) == 0) return;
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
	wchar_t message[128];

	for (int i = 0; i < NUM_CAP; i++) {
		SocialForceAgent *other = myClone->context[i];
		SocialForceAgentData otherData = other->data;
		ds = length(otherData.loc - dataLocal.loc);
		if (contextId == 10 && g_stepCount == 70 && this->myClone->cloneid == 1) {
			swprintf_s(message, 128, L"%d: %.4f, %.4f, %.4f\n", 
				other->contextId, otherData.loc.x, otherData.loc.y, ds);
			OutputDebugString(message);
		}
		if (ds < 6 && ds > 0) {
			neighborCount++;
			computeIndivSocialForceRoom(dataLocal, otherData, fSum);
		}
	}
	dataLocal.numNeighbor = neighborCount;
}
void SocialForceAgent::chooseNewGoal(const double2 &newLoc, double epsilon, double2 &newGoal) {
	if (newLoc.x < 0.30 * ENV_DIM)
		newGoal = make_double2(0.30 * ENV_DIM, 0.20 * ENV_DIM);
	else if (newLoc.x < 0.66 * ENV_DIM) 
		newGoal = make_double2(0.70 * ENV_DIM, 0.80 * ENV_DIM);
	else
		newGoal = make_double2(ENV_DIM, 0.50 * ENV_DIM);
}
void SocialForceAgent::step(){
	double cMass = 100;

	const double2& loc = data.loc;
	const double2& goal = data.goal;
	const double2& velo = data.velocity;
	const double& v0 = data.v0;
	const double& mass = data.mass;

	if (g_stepCount == 71)
		printf("");

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
		computeWallImpaction(data, gate, newVelo, tick, mint);
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
	this->color.x = rand() % 255;
	this->color.y = rand() % 255;
	this->color.z = rand() % 255;
	this->contextId = idx;
	this->myOrigin = NULL;

	SocialForceAgentData & dataLocal = this->data; //= &sfModel->originalAgents->dataArray[dataSlot];
	dataLocal.agentPtr = this;
	dataLocal.loc.x = ( distr(randGen)) * ENV_DIM;
	dataLocal.loc.y = (0.3 + 0.4 * distr(randGen)) * ENV_DIM;

	dataLocal.velocity.x = 2;//4 * (this->random->uniform()-0.5);
	dataLocal.velocity.y = 2;//4 * (this->random->uniform()-0.5);

	dataLocal.v0 = 2;
	dataLocal.mass = 50;
	dataLocal.numNeighbor = 0;
	//dataLocal.goal = make_double2((float)rand() / (float)RAND_MAX * ENV_DIM, (float)rand() / (float)RAND_MAX * ENV_DIM);
	if (dataLocal.loc.x < ENV_DIM / 2)
		dataLocal.goal = make_double2(ENV_DIM, ENV_DIM / 2);
	else
		dataLocal.goal = make_double2(0, ENV_DIM / 2);
	//chooseNewGoal(dataLocal.loc, 0, dataLocal.goal);	
	this->dataCopy = dataLocal;
}
void SocialForceAgent::initNewClone(SocialForceAgent *parent, SocialForceClone *childClone) {
	this->color = childClone->color;
	this->contextId = parent->contextId;
	this->myOrigin = parent;
	this->myClone = childClone;
	this->goalIdx = parent->goalIdx;

	this->data = parent->data;
	this->dataCopy = parent->dataCopy;

	this->data.agentPtr = this;
}
void SocialForceClone::step(int stepCount) {
	//alterGate(stepCount);
	for (int i = 0; i < numElem; i++)
		ap->agentPtrArray[i]->step();
}

void SocialForceClone::alterGate(int stepCount) {
	for (int i = 0; i < NUM_PARAM; i++) {
		if (cloneParams[i] == stepCount)
			gates[i].init(0, 0, 0, 0);
	}
}

// exp1 cloned version
class SocialForceSimApp1 {
public:
	SocialForceClone **cAll;
	int paintId = 0;
	int totalClone = 4;
	int &stepCount = g_stepCount;
	int rootCloneId = 0;
	int **cloneTree;
	int paramWeight[NUM_PARAM];

	int initSimClone() {
		srand(0);

		cAll = new SocialForceClone*[totalClone];
		cloneTree = new int*[2];
		int j = 0;

		int cloneParams[NUM_PARAM];
		for (int i = 0; i < NUM_PARAM; i++) {
			cloneParams[i] = 1;
		}

		paramWeight[0] = 1;
		paramWeight[1] = 3;
		paramWeight[2] = 1;
		paramWeight[3] = 3;

		for (int i = 0; i < totalClone; i++) {
			for (int i = 0; i < NUM_PARAM; i++) {
				cloneParams[i] = 1;
			}
			if (i == 1) cloneParams[0] = 100;
			if (i == 2) cloneParams[1] = 100;
			if (i == 3) { cloneParams[0] = 100; cloneParams[1] = 100; }
			cAll[i] = new SocialForceClone(i, cloneParams);
		}

		SocialForceAgent *agents = cAll[rootCloneId]->ap->agentArray;
		SocialForceAgent **context = cAll[rootCloneId]->context;

		for (int i = 0; i < NUM_CAP; i++) {
			agents[i].myClone = cAll[rootCloneId];
			agents[i].contextId = i;
			agents[i].color = cAll[rootCloneId]->color;
			agents[i].init(i);
			context[i] = &agents[i];
		}

		cAll[rootCloneId]->numElem = NUM_CAP;
		for (int j = 0; j < NUM_CAP; j++)
			cAll[rootCloneId]->cloneFlag[j] = true;

		//mst();

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
			if (parentClone->cloneParams[i] == childClone->cloneParams[i])
				continue;
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
		int minx = max((loc.x - RADIUS_I) / CELL_DIM, 0);
		int miny = max((loc.y - RADIUS_I) / CELL_DIM, 0);
		int maxx = min((loc.x + RADIUS_I) / CELL_DIM, NUM_CELL - 1);
		int maxy = min((loc.y + RADIUS_I) / CELL_DIM, NUM_CELL - 1);
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
		for (int i = 0; i < childClone->numElem; i++) {
			SocialForceAgent *agent = childClone->ap->agentPtrArray[i];
			childClone->context[agent->contextId] = agent;
		}

		// 3. construct passive cloning map
		double2 dim = make_double2(ENV_DIM, ENV_DIM);
		memset(childClone->takenMap, 0, sizeof(bool) * NUM_CELL * NUM_CELL);
		for (int i = 0; i < childClone->numElem; i++) {
			const SocialForceAgent &agent = *childClone->ap->agentPtrArray[i];
			int takenId = agent.data.loc.x / CELL_DIM;
			takenId = takenId * NUM_CELL + agent.data.loc.y / CELL_DIM;
			childClone->takenMap[takenId] = true;
		}
		
		// 4. perform active and passive cloning (in cloningCondition checking)
		//for (int i = 0; i < parentClone->numElem; i++) {
		//	SocialForceAgent *agent = parentClone->ap->agentPtrArray[i];

		for (int i = 0; i < NUM_CAP; i++) {
			SocialForceAgent *agent = parentClone->context[i];
			if (cloningCondition(agent, childClone->takenMap, parentClone, childClone)) {
				SocialForceAgent &childAgent = *childClone->ap->agentPtrArray[childClone->numElem];
				childClone->ap->takenFlags[childClone->numElem] = true;
				childAgent.initNewClone(agent, childClone);
				childClone->context[childAgent.contextId] = &childAgent;
				childClone->cloneFlag[childAgent.contextId] = true;
				childClone->numElem++;
			}
		}
	}
	void compareAndEliminate(SocialForceClone *parentClone, SocialForceClone *childClone) {
		wchar_t message[20];
		for (int i = 0; i < childClone->numElem; i++) {
			SocialForceAgent &childAgent = *childClone->ap->agentPtrArray[i];
			SocialForceAgent &parentAgent = *childAgent.myOrigin;

			double velDiff = length(childAgent.dataCopy.velocity - parentAgent.dataCopy.velocity);
			double locDiff = length(childAgent.dataCopy.loc - parentAgent.dataCopy.loc);
			if (locDiff == 0 && velDiff == 0) {
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
		childClone->numElem = childClone->ap->reorder(childClone->numElem);
	}
	void proc(int p, int c, bool o, char *s) {
		performClone(cAll[p], cAll[c]);
		cAll[c]->step(stepCount);
		if (o) {
			if (stepCount < 1000)
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
					if (cAll[i]->cloneParams[k] != cAll[j]->cloneParams[k]) {
						//cloneDiff[i][j] += cAll[i]->cloneParams[k] * paramWeight[k] + cAll[j]->cloneParams[k] * paramWeight[k];
						cloneDiff[i][j]++;
					}
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
		if (stepCount < 1000 && o)
			cAll[rootCloneId]->output(stepCount, "s0");
		cAll[rootCloneId]->swap();
	}
	void stepApp1(bool o) {
		stepCount++;
		cAll[rootCloneId]->step(stepCount);
		proc(0, 1, o, "s1");
		for (int i = 0; i < totalClone; i++)
			cAll[i]->swap();
	}
	void stepApp(){
		stepCount++;
		cAll[rootCloneId]->step(stepCount);
		proc(0, 1, 0, "g1");
		proc(0, 2, 0, "g1");
		proc(2, 3, 1, "scene2_cloned");
		for (int i = 0; i < totalClone; i++)
			cAll[i]->swap();
	}
};

// exp1 stand-alone version
class SocialForceSimApp {
public:
	SocialForceClone **cAll;
	int paintId = 0;
	int totalClone = 1;
	int &stepCount = g_stepCount;
	int rootCloneId = 0;
	int **cloneTree;
	int paramWeight[NUM_PARAM];

	int initSimClone() {
		srand(0);

		cAll = new SocialForceClone*[totalClone];
		cloneTree = new int*[2];
		int j = 0;

		int cloneParams[NUM_PARAM];
		for (int i = 0; i < NUM_PARAM; i++) {
			cloneParams[i] = 1;
		}

		paramWeight[0] = 1;
		paramWeight[1] = 3;
		paramWeight[2] = 1;
		paramWeight[3] = 3;

		for (int i = 0; i < totalClone; i++) {
			for (int i = 0; i < NUM_PARAM; i++) {
				cloneParams[i] = 1;
			}
			if (i == 1) cloneParams[0] = 100;
			if (i == 2) cloneParams[1] = 100;
			if (i == 3) { cloneParams[0] = 100; cloneParams[1] = 100; }
			cAll[i] = new SocialForceClone(i, cloneParams);
		}

		SocialForceAgent *agents = cAll[rootCloneId]->ap->agentArray;
		SocialForceAgent **context = cAll[rootCloneId]->context;

		for (int i = 0; i < NUM_CAP; i++) {
			agents[i].myClone = cAll[rootCloneId];
			agents[i].contextId = i;
			agents[i].color = cAll[rootCloneId]->color;
			agents[i].init(i);
			context[i] = &agents[i];
		}

		cAll[rootCloneId]->numElem = NUM_CAP;
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
			if (parentClone->cloneParams[i] == childClone->cloneParams[i])
				continue;
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
		int minx = max((loc.x - RADIUS_I) / CELL_DIM, 0);
		int miny = max((loc.y - RADIUS_I) / CELL_DIM, 0);
		int maxx = min((loc.x + RADIUS_I) / CELL_DIM, NUM_CELL - 1);
		int maxy = min((loc.y + RADIUS_I) / CELL_DIM, NUM_CELL - 1);
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
		for (int i = 0; i < childClone->numElem; i++) {
			SocialForceAgent *agent = childClone->ap->agentPtrArray[i];
			childClone->context[agent->contextId] = agent;
		}

		// 3. construct passive cloning map
		double2 dim = make_double2(ENV_DIM, ENV_DIM);
		memset(childClone->takenMap, 0, sizeof(bool) * NUM_CELL * NUM_CELL);
		for (int i = 0; i < childClone->numElem; i++) {
			const SocialForceAgent &agent = *childClone->ap->agentPtrArray[i];
			int takenId = agent.data.loc.x / CELL_DIM;
			takenId = takenId * NUM_CELL + agent.data.loc.y / CELL_DIM;
			childClone->takenMap[takenId] = true;
		}

		// 4. perform active and passive cloning (in cloningCondition checking)
		for (int i = 0; i < NUM_CAP; i++) {
			SocialForceAgent *agent = parentClone->context[i];
			if (cloningCondition(agent, childClone->takenMap, parentClone, childClone)) {
				SocialForceAgent &childAgent = *childClone->ap->agentPtrArray[childClone->numElem];
				childClone->ap->takenFlags[childClone->numElem] = true;
				childAgent.initNewClone(agent, childClone);
				childClone->context[childAgent.contextId] = &childAgent;
				childClone->cloneFlag[childAgent.contextId] = true;
				childClone->numElem++;
			}
		}
	}
	void compareAndEliminate(SocialForceClone *parentClone, SocialForceClone *childClone) {
		wchar_t message[20];
		for (int i = 0; i < childClone->numElem; i++) {
			SocialForceAgent &childAgent = *childClone->ap->agentPtrArray[i];
			SocialForceAgent &parentAgent = *childAgent.myOrigin;

			double velDiff = length(childAgent.dataCopy.velocity - parentAgent.dataCopy.velocity);
			double locDiff = length(childAgent.dataCopy.loc - parentAgent.dataCopy.loc);
			if (locDiff == 0 && velDiff == 0) {
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
		childClone->numElem = childClone->ap->reorder(childClone->numElem);
	}
	void proc(int p, int c, bool o, char *s) {
		performClone(cAll[p], cAll[c]);
		cAll[c]->step(stepCount);
		if (o) {
			if (stepCount < 1000)
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
					if (cAll[i]->cloneParams[k] != cAll[j]->cloneParams[k]) {
						//cloneDiff[i][j] += cAll[i]->cloneParams[k] * paramWeight[k] + cAll[j]->cloneParams[k] * paramWeight[k];
						cloneDiff[i][j]++;
					}
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
		if (stepCount < 1000 && o)
			cAll[rootCloneId]->output(stepCount, "s0");
		cAll[rootCloneId]->swap();
	}
	void stepApp1(bool o) {
		stepCount++;
		cAll[rootCloneId]->step(stepCount);
		proc(0, 1, o, "s1");
		for (int i = 0; i < totalClone; i++)
			cAll[i]->swap();
	}
	void stepApp(){
		stepCount++;
		cAll[rootCloneId]->step(stepCount);
		if (stepCount < 300)
			cAll[rootCloneId]->output(stepCount, "scene2_single");
		cAll[rootCloneId]->swap();
	}
};