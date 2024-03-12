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
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    //viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters();
    return (viewer);
}

int main() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("../../reconstructions/fountain_ORB_BF/dense/pointCloud.ply", *cloud) == -1) {
        PCL_ERROR("Couldn't read file .ply \n");
        return -1;
    }

    pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis(cloud);

    while (!viewer->wasStopped()) {
        // viewer->spinOnce(100);
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
	viewer->spin();
    }

    return 0;
}
