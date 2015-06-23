#include "../TestVisual2/SocialForce.h"

int main(int argc, char **argv) {
	SocialForceSimApp app;
	app.initSimClone();

	//for (int i = 0; i < 100; i++) {
	while (true) {
		app.stepApp();
	}
}