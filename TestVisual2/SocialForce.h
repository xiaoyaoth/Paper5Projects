#include <vector>
#include <math.h>
#include <ctime>
#include <Windows.h>
#include <algorithm>
#include <fstream>

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
#define NUM_PARAM 3
#define ENV_DIM 64
#define NUM_CELL 8
#define CELL_DIM 8
#define RADIUS_I 5

#define NUM_WALLS 6

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

	Color color;
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
	int numElem;

	AgentPool(int numCap) {
		numElem = 0;
		agentArray = new SocialForceAgent[numCap];
		agentPtrArray = new SocialForceAgent*[numCap];
		takenFlags = new bool[numCap];
		for (int i = 0; i < numCap; i++) {
			agentPtrArray[i] = &agentArray[i];
			takenFlags[i] = 0;
		}
	}

	void reorder() {
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
	inline void swap(T * ar, int a, int b) {
		T t1 = ar[a];
		ar[a] = ar[b];
		ar[b] = t1;
	}
};

class SocialForceClone {
public:
	AgentPool *ap;
	SocialForceAgent **context;
	bool *cloneFlag;
	int pv[NUM_PARAM];
	obstacleLine walls[6];
	bool takenMap[NUM_CELL * NUM_CELL];
	
	Color color;
	int cloneid;
	int parentCloneid;

	fstream fout;

	SocialForceClone(int id, int pv1[NUM_PARAM]) {
		cloneid = id;
		ap = new AgentPool(NUM_CAP);
		//agents = new SocialForceAgent[NUM_CAP];
		context = new SocialForceAgent*[NUM_CAP];
		cloneFlag = new bool[NUM_CAP];
		memset(context, 0, sizeof(void*) * NUM_CAP);
		memset(cloneFlag, 0, sizeof(bool) * NUM_CAP);
		color = Color();
		memcpy(pv, pv1, sizeof(int) * NUM_PARAM);
		walls[0].init(0.25 * ENV_DIM, -0.10 * ENV_DIM, 0.25 * ENV_DIM, (0.45 - pv[0] * 0.05) * ENV_DIM);
		walls[1].init(0.25 * ENV_DIM,  0.50 * ENV_DIM, 0.25 * ENV_DIM, 1.10 * ENV_DIM);
		walls[2].init(0.50 * ENV_DIM, -0.10 * ENV_DIM, 0.50 * ENV_DIM, (0.25 - pv[1] * 0.05) * ENV_DIM);
		walls[3].init(0.50 * ENV_DIM,  0.30 * ENV_DIM, 0.50 * ENV_DIM, 1.10 * ENV_DIM);
		walls[4].init(0.75 * ENV_DIM, -0.10 * ENV_DIM, 0.75 * ENV_DIM, (0.70 - pv[2] * 0.05) * ENV_DIM);
		walls[5].init(0.75 * ENV_DIM,  0.75 * ENV_DIM, 0.75 * ENV_DIM, 1.10 * ENV_DIM);

		char filename[20];
		sprintf_s(filename, 20, "clone%d.txt", cloneid);
		fout.open(filename, fstream::out);
		fout.close();
	}
	void step();
	void swap() {
		for (int i = 0; i < ap->numElem; i++) {
			SocialForceAgent &agent = *ap->agentPtrArray[i];
			agent.data = agent.dataCopy;
		}
	}
	void output(int stepCount) {
		char filename[20];
		sprintf_s(filename, 20, "clone%d.txt", cloneid);
		fout.open(filename, fstream::app);
		fout << "========== stepCount: " << stepCount << " ==========="<<endl;
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
	if (newLoc.x < 0.25 * ENV_DIM) {
		newGoal.x = 0.26 * ENV_DIM;
		newGoal.y = 0.48 * ENV_DIM;
	}
	else if (newLoc.x < 0.5 * ENV_DIM) {
		newGoal.x = 0.51 * ENV_DIM;
		newGoal.y = 0.27 * ENV_DIM;
	}
	else if (newLoc.x < 0.75 * ENV_DIM) {
		newGoal.x = 0.76 * ENV_DIM;
		newGoal.y = 0.73 * ENV_DIM;
	}
	else {
		newGoal.x = 1.00 * ENV_DIM;
		newGoal.y = 0.50 * ENV_DIM;
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
	dataLocal.loc.x = (float)rand() / (float)RAND_MAX * ENV_DIM * 0.1;
	dataLocal.loc.y = (float)rand() / (float)RAND_MAX * ENV_DIM;
	//dataLocal.loc.x = ENV_DIM * 0.24 - 0.1;
	//dataLocal.loc.y = ENV_DIM * 0.50 - 0.1;
	dataLocal.velocity.x = 2;//4 * (this->random->uniform()-0.5);
	dataLocal.velocity.y = 2;//4 * (this->random->uniform()-0.5);

	dataLocal.v0 = 2;
	dataLocal.mass = 50;
	dataLocal.numNeighbor = 0;
	dataLocal.goal = double2(ENV_DIM, 0.5 * ENV_DIM);
	//chooseNewGoal(dataLocal.loc, 0, dataLocal.goal);

	this->dataCopy = dataLocal;
}
void SocialForceAgent::initNewClone(SocialForceAgent *parent, SocialForceClone *childClone) {
	this->color = childClone->color;
	this->contextId = parent->contextId;
	this->myOrigin = parent;
	this->myClone = childClone;

	this->data = parent->data;
	this->dataCopy = parent->dataCopy;

	this->data.agentPtr = this;
	this->data.agentPtr = this;
}
void SocialForceClone::step() {
	for (int i = 0; i < ap->numElem; i++)
		ap->agentPtrArray[i]->step();
}

class SocialForceSimApp {
public:
	SocialForceClone **cAll;
	int paintId = 7;
	int totalClone = 8;
	int stepCount = 0;
	int rootCloneId = 0;

	int initSimClone() {
		srand(0);

		cAll = new SocialForceClone*[totalClone];
		WCHAR message[200];
		int j = 0;
		for (int i = 0; i < totalClone; i++) {
			int pv[NUM_PARAM];
			pv[0] = i & 1;			pv[1] = (i >> 1) & 1;
			pv[2] = (i >> 2) & 1;
			cAll[i] = new SocialForceClone(i, pv);
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

		return EXIT_SUCCESS;
	}
	bool cloningCondition(SocialForceAgent *agent, bool *childTakenMap,
		SocialForceClone *parentClone, SocialForceClone *childClone) {
		double2 
			c1(0.25 * ENV_DIM, 0.50 * ENV_DIM),
			c2(0.50 * ENV_DIM, 0.25 * ENV_DIM),
			c3(0.75 * ENV_DIM, 0.75 * ENV_DIM);

		// if agent has been cloned?
		if (childClone->cloneFlag[agent->contextId] == true)
			return false;

		// active cloning condition
		double2 &loc = agent->data.loc;
		if (parentClone->pv[0] != childClone->pv[0])
			if (length(loc - c1) < 6) return true;
		if (parentClone->pv[1] != childClone->pv[1])
			if (length(loc - c2) < 6) return true;
		if (parentClone->pv[2] != childClone->pv[2])
			if (length(loc - c3) < 6) return true;

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
		for (int i = 0; i < childClone->ap->numElem; i++) {
			SocialForceAgent *agent = childClone->ap->agentPtrArray[i];
			childClone->context[agent->contextId] = agent;
		}

		// 3. construct passive cloning map
		double2 dim(ENV_DIM, ENV_DIM);
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
	void proc(int p, int c, bool o) {
		performClone(cAll[p], cAll[c]);
		cAll[c]->step();
		if (o) {
			if (stepCount < 800)
				cAll[c]->output(stepCount);
		}
		compareAndEliminate(cAll[p], cAll[c]);
	}
	void stepApp0() {
		stepCount++;
		cAll[rootCloneId]->step();
		if (stepCount < 800)
			cAll[rootCloneId]->output(stepCount);
		cAll[rootCloneId]->swap();
	}
	void stepApp1() {
		stepCount++;

		cAll[rootCloneId]->step();

		performClone(cAll[0], cAll[1]);
		cAll[1]->step();
		compareAndEliminate(cAll[0], cAll[1]);

		performClone(cAll[0], cAll[2]);
		cAll[2]->step();
		compareAndEliminate(cAll[0], cAll[2]);

		performClone(cAll[0], cAll[4]);
		cAll[4]->step();
		compareAndEliminate(cAll[0], cAll[4]);

		performClone(cAll[1], cAll[3]);
		cAll[3]->step();
		compareAndEliminate(cAll[1], cAll[3]);

		performClone(cAll[1], cAll[5]);
		cAll[5]->step();
		compareAndEliminate(cAll[1], cAll[5]);

		performClone(cAll[2], cAll[6]);
		cAll[6]->step();
		compareAndEliminate(cAll[2], cAll[6]);

		performClone(cAll[3], cAll[7]);
		cAll[7]->step();
		compareAndEliminate(cAll[3], cAll[7]);
		// debug output
		if (stepCount < 800)
			cAll[7]->output(stepCount);

		for (int j = 0; j < totalClone; j++) {
			cAll[j]->swap();
		}
		
		/*
		// perform clone
		for (int j = 0; j < totalClone; j++) {
			if (j == rootCloneId)
				continue;
			performClone(cAll[rootCloneId], cAll[j]);
		}

		// step
		for (int j = 0; j < totalClone; j++) {
			cAll[j]->step();
		}

		// swap
		for (int j = 0; j < totalClone; j++) {
			swapAll(cAll[j]);
		}

		// compare and eliminate
		for (int j = 0; j < totalClone; j++) {
			if (j == rootCloneId)
				continue;
			compareAndEliminate(cAll[rootCloneId], cAll[j]);
		}
		*/
	}
	void stepApp() {
		stepCount++;
		cAll[rootCloneId]->step();

		for (int i = 0; i < 6; i++)
			proc(i, i + 1, 0);
		proc(6, 7, 1);

		for (int j = 0; j < totalClone; j++) {
			cAll[j]->swap();
		}
	}
};
