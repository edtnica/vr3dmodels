#include "../include/sfm.h"

int main() {
	structureFromMotion sfm;

	sfm.run_reconstruction();

	// sfm.pointcloud_to_ply("fountainfinal");
	
	// sfm.PMVS2();

	// bool dense_reconstruction_success = std::system("/home/csimage/GitRepos/3rdYear/vr3dmodels/pmvs2 denseCloud/ options.txt");

}
