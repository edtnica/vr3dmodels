#include "sfm.h"

int main() {
	structureFromMotion sfm;

	sfm.loadImages();

	sfm.getCameraMatrix("C:/Programs and Stuff/vr3dmodels/calibration/cameraMatrix2.xml");

	sfm.getFeatures();

	sfm.baseReconstruction();

	// sfm.addViews();

}
