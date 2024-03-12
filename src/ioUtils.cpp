#include "../include/ioUtils.h"

void ply_to_pcd(std::string plyFilePath, std::string pcdFilePath) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Load the .ply file
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(plyFilePath, *cloud) == -1) {
        PCL_ERROR("Could not read .ply file\n");
    }
	
	pcl::io::savePLYFile("../reconstructions/" + pcdFilePath + "/dense/pointCloud.ply", *cloud);
	
	pcl::io::savePCDFile("../reconstructions/" + pcdFilePath + "/dense/pointCloud.pcd", *cloud);
	
}