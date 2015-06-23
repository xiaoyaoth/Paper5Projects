//#include "../TestVisual2/SocialForce.h"
#include <iostream>
#include <ctime>

using namespace std;

bool *takenFlags;
int *indexes;

template<class T>
inline void swap(T * ar, int a, int b) {
	T t1 = ar[a];
	ar[a] = ar[b];
	ar[b] = t1;
}

void reorder(int l, int r) {
	int i = l, j = l;
	for (; j < r; j++) {
		if (takenFlags[j] == false) {
			swap<int>(indexes, i, j);
			swap<bool>(takenFlags, i, j);
			i++;
		}
	}
}

int main() {
	//srand(time(0));
	int NUM = 16;
	takenFlags = new bool[NUM];
	indexes = new int[NUM];
	for (int i = 0; i < NUM; i++) {
		takenFlags[i] = rand() % 2;
		indexes[i] = i;
	}
	reorder(0, NUM);
	for (int i = 0; i < NUM; i++) {
		cout << takenFlags[i] << " " << indexes[i] << endl;
	}
	return 0;
}

//int main1(int argc, char **argv) {
//	SocialForceSimApp app;
//	app.initSimClone();
//
//	//for (int i = 0; i < 100; i++) {
//	while (true) {
//		app.stepApp();
//	}
//}