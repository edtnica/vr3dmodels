#include "sfm.h"

int main() {
	structureFromMotion sfm;

	sfm.setLogging();

	sfm.loadImages();

	sfm.getCameraMatrix();

	sfm.getFeatures();

	sfm.createFeatureMatchMatrix();

	sfm.baseReconstruction();

	// sfm.addViews();

}
