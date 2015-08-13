#include "../TestVisual2/SocialForce.h"
#include <iostream>

using namespace std;

double PCFreq = 0.0;
__int64 CounterStart = 0;

void StartCounter()
{
	LARGE_INTEGER li;
	if (!QueryPerformanceFrequency(&li))
		cout << "QueryPerformanceFrequency failed!\n";

	PCFreq = double(li.QuadPart) / 1000.0;

	QueryPerformanceCounter(&li);
	CounterStart = li.QuadPart;
}
double GetCounter()
{
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	return double(li.QuadPart - CounterStart) / PCFreq;
}

int main(int argc, char **argv) {
	fstream fout;
	fout.open("time.txt", fstream::out);

	SocialForceSimApp app1;
	app1.initSimClone();
	StartCounter();
	for (int i = 0; i < 1000; i++) {
		app1.stepApp5(0);
		fout << GetCounter() << "\t";
		//for (int i = 0; i < app1.totalClone; i++)
		//	fout << app1.cAll[i]->ap->numElem << "\t";
		fout << endl;
	}
	cout << "step1: " << (GetCounter()) << endl;

	fout << endl;
	SocialForceSimApp app2;
	app2.initSimClone();
	StartCounter();
	for (int i = 0; i < 1000; i++) {
		app2.stepApp6(0);
		fout << GetCounter() << "\t";
		//for (int i = 0; i < app2.totalClone; i++)
		//	fout << app2.cAll[i]->ap->numElem << "\t";
		fout << endl;
	}
	cout << "step2: " << GetCounter() << endl;

	return 0;
}

int main1(int argc, char **argv) {
	
	fstream fout;
	fout.open("time.txt", fstream::out);

	DWORD st, et;
	st = GetTickCount();
	SocialForceSimApp app1;
	app1.initSimClone();
	for (int i = 0; i < 100; i++) {
		app1.stepApp1(0);
		fout << GetTickCount() << endl;
	}
	et = GetTickCount();
	cout <<"step1: " << (et - st) << endl;

	fout << endl;
	st = GetTickCount();
	SocialForceSimApp app2;
	app2.initSimClone();
	for (int i = 0; i < 100; i++) {
		app2.stepApp2(0);
		fout << GetTickCount() << endl;
	}
	et = GetTickCount();
	cout << "step2: " << (et - st) << endl;

	fout << endl;
	st = GetTickCount();
	SocialForceSimApp app3;
	app3.initSimClone();
	for (int i = 0; i < 100; i++) {
		app3.stepApp3(0);
		fout << GetTickCount() << endl;
	}
	et = GetTickCount();
	cout << "step3: " << (et - st) << endl;

	return 0;
}