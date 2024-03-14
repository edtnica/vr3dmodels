#include "../include/sfm.h"
#include "../include/mesh.h"

int main(int argc, char *argv[]) {
	
	if (argc == 2 && (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0)) {
		std::cout << "Virtual 3D Models user manual\n\n";
		std::cout << "To run the program add the arguments in the following order:\n";
		std::cout << "1. argv[1]: reconstruction name\n";
		exit(0);
	}
	
	else if (argc != 2) {
		std::cout << "ERROR: Invalid number of arguments\n";
		exit(0);
	}
	
	std::string reconstructionName = argv[1];
	
	structureFromMotion sfm(reconstructionName);

	sfm.run_reconstruction();

	sfm.pointcloud_to_ply("../reconstructions/" + reconstructionName + "/sparse/pointCloud");
	
	sfm.PMVS2();

	bool dense_reconstruction_success = std::system("../tools/pmvs2 denseCloud/ options.txt");

	if (dense_reconstruction_success > 0) {
		std::cout << "Failed. pmvs2 did not succeed\n";
		std::exit(-1);
	}
	
	ply_to_pcd("../build/denseCloud/models/options.txt.ply", reconstructionName);
	
	Mesh mesh;
	mesh.create_mesh("../reconstructions/" + reconstructionName + "/dense/pointCloud.pcd", reconstructionName);


	return 0;
}
