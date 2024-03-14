#include <iostream>
#include <thread>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "Point Cloud");
    viewer->initCameraParameters();
    return (viewer);
}

void visualize_mesh(pcl::PolygonMesh mesh) {
	pcl::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    
	viewer->setBackgroundColor(0, 0, 0);
    viewer->addPolygonMesh(mesh, "meshes");
    viewer->initCameraParameters();

    std::cout << "Press q to exit." << std::endl;
    while (!viewer->wasStopped()) {
        viewer->spin();
    }

}
	

int main(int argc, char *argv[]) {
	std::string inputFile = argv[1];
	std::string fileType = argv[2];
	
	if (fileType == "PLY" || fileType == "ply") {
	
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(inputFile, *cloud) == -1) {
			PCL_ERROR("Couldn't read file .ply \n");
			return -1;
		}

		pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis(cloud);

		while (!viewer->wasStopped()) {
			viewer->spin();
		}
	}

	else if (fileType == "MESH" || fileType == "mesh") {
		pcl::PolygonMesh mesh;
        pcl::io::loadPLYFile(inputFile, mesh);
		visualize_mesh(mesh);
	}
	
	else {
		std::cout << "ERROR: Invalid file type\n";
	}

    return 0;
}
