#include "../include/sfm.h"
#include "../include/mesh.h"

int main(int argc, char *argv[]) {
	std::string reconstructionName = argv[1];
	
	//structureFromMotion sfm(reconstructionName);

	//sfm.run_reconstruction();

	//sfm.pointcloud_to_ply("../reconstructions/" + reconstructionName + "/sparse/pointCloud");
	
	//sfm.PMVS2();

	//bool dense_reconstruction_success = std::system("../tools/pmvs2 denseCloud/ options.txt");

	//if (dense_reconstruction_success > 0) {
		//std::cout << "Failed. pmvs2 did not succeed\n";
		//std::exit(-1);
	//}
	
	// ply_to_pcd("../build/denseCloud/models/options.txt.ply", reconstructionName);
	
	Mesh mesh;
	mesh.create_mesh("../reconstructions/fountain_ORB_BF/dense/pointCloud.pcd", "mesh");


	return 0;
}
