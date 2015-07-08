#include "../TestVisual2/SocialForce.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
	
	fstream fout;
	fout.open("time.txt", fstream::out);

	DWORD st, et;
	st = GetTickCount();
	SocialForceSimApp app1;
	app1.initSimClone();
	for (int i = 0; i < 800; i++) {
		app1.stepApp1(0);
		fout << GetTickCount() << endl;
	}
	et = GetTickCount();
	cout <<"step1: " << (et - st) << endl;

	fout << endl;
	st = GetTickCount();
	SocialForceSimApp app2;
	app2.initSimClone();
	for (int i = 0; i < 800; i++) {
		app2.stepApp2(0);
		fout << GetTickCount() << endl;
	}
	et = GetTickCount();
	cout << "step2: " << (et - st) << endl;

	fout << endl;
	st = GetTickCount();
	SocialForceSimApp app3;
	app3.initSimClone();
	for (int i = 0; i < 800; i++) {
		app3.stepApp3(0);
		fout << GetTickCount() << endl;
	}
	et = GetTickCount();
	cout << "step3: " << (et - st) << endl;
}