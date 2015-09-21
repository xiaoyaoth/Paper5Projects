#include "inc\helper_math.h"
#include <fstream>
#include <string>
#include <iostream>

using namespace std;
#define ENV_DIM 1024

class ForestFireClone;

class ForestFireAgent {
public:
	int2 loc;
	ForestFireClone *myClone;
	void step();
};

class AgentPool {
public:
	ForestFireAgent *agentArray;
	ForestFireAgent **agentPtrArray;
	bool *takenFlags;
	int numElem;

	AgentPool(int numCap) {
		numElem = 0;
		agentArray = new ForestFireAgent[numCap];
		agentPtrArray = new ForestFireAgent*[numCap];
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
				swap<ForestFireAgent*>(agentPtrArray, i, j);
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

class ForestFireClone{
public:
	AgentPool *ap;
	/*landscape*/
	int **elevMap;
	int **slopeMap;
	int **aspectMap;
	int **fuelMap;
	int **canopyMap;
	int **heightMap;
	int **cbhMap; // crown base height;
	int **cbdMap; // crown bulk density;
	int **duffMap;
	int **cwdMap; // coarse woody map

	/*Adjustments*/
	/*Moistures*/
	/* custom model */
	/* coarse woody */
	/* weather file */
	/* wind files */
	/* burn period */

	ForestFireClone() {

	}
};

void ForestFireAgent::step() {
	int cellIndex = loc.x * ENV_DIM + loc.y;
}

class ForestFireSimApp {
public:
	float **elevMap, maxElev;
	float **slopeMap, maxSlope;
	float **aspectMap, maxAspect;
	float **fuelMap, maxFuel;
	float **canopyMap, maxCanopy;
	float **heightMap, maxHeight;
	int nrows;
	int ncols;
	int paintLayer = 0;
	float **paintMap, paintMax;

	void readFile(char *filename, float** &map, float &maxVal, int nrows, int ncols, int skip) {
		wifstream fin;
		fin.open(filename);
		if (!fin.is_open()) {
			OutputDebugString(L"not open\n");
			return;
		}
		for (int i = 0; i < skip; i++)
			getline(fin, wstring());

		maxVal = 0;
		map = new float*[nrows];
		for (int i = 0; i < nrows; i++) {
			map[i] = new float[ncols];
			for (int j = 0; j < ncols; j++) {
				fin >> map[i][j];
				if (map[i][j] > maxVal) 
					maxVal = map[i][j];
			}
		}

		fin.close();
		wchar_t message[256];
		wsprintf(message, L"%d\n", map[nrows - 1][ncols - 1]);
		OutputDebugString(message);
	}

	void init() {
		nrows = 266, ncols = 382;
		readFile("Ashley\\ash_elev.asc", elevMap, maxElev, nrows, ncols, 6);
		readFile("Ashley\\ash_slope.asc", slopeMap, maxSlope, nrows, ncols, 6);
		readFile("Ashley\\ash_aspect.asc", aspectMap, maxAspect, nrows, ncols, 6);
		readFile("Ashley\\ash_fuel.asc", fuelMap, maxFuel, nrows, ncols, 6);
		readFile("Ashley\\ash_canopy.asc", canopyMap, maxCanopy, nrows, ncols, 6);
		readFile("Ashley\\ash_height.asc", heightMap, maxHeight, nrows, ncols, 6);
		paintMap = elevMap;
		paintMax = maxElev;
	}
	void step() {

	}
};