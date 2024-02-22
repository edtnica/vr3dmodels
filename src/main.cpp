#include "../include/sfm.h"

int main() {
	structureFromMotion sfm;

	sfm.setLogging();

	sfm.loadImages();

	sfm.getCameraMatrix();

	sfm.getFeatures();

	sfm.createFeatureMatchMatrix();

	sfm.baseReconstruction();

	sfm.addViews();

	sfm.pointcloud_to_ply("fountain1");

}
