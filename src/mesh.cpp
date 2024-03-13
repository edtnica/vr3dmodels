#include "../include/mesh.h"

void Mesh::filter_and_remove_points(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filterCloud) {
	pcl::PassThrough<pcl::PointXYZ> pass_through;
	pass_through.setInputCloud(cloud);
	pass_through.setFilterFieldName("x");
	pass_through.setFilterLimits(0.003, 0.83);
	pass_through.filter(*filterCloud);
	
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier_removal;
	radius_outlier_removal.setInputCloud(cloud);
	radius_outlier_removal.setRadiusSearch(0.07);
	radius_outlier_removal.setMinNeighborsInRadius(150);
	radius_outlier_removal.filter(*filterCloud);
}

void Mesh::saveMeshToPLY(const pcl::PolygonMesh& mesh, const std::string& filename) {
    if (pcl::io::savePLYFile(filename, mesh, false) == 0) {
        std::cout << "Mesh saved successfully to " << filename << "\n";
    }
	else {
        std::cout << "Error saving mesh to " << filename << "\n";
    }
}

void Mesh::create_mesh(std::string input_file, std::string output_file) {
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(input_file, *cloud);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::PolygonMesh mesh;
	
	filter_and_remove_points(cloud, filtered_cloud);
	
	
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setNumberOfThreads(8);
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod(tree);
	ne.setKSearch(10); //20
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
	ne.compute(*cloud_normals);

	for(std::size_t i = 0; i < cloud_normals->size (); ++i){
		cloud_normals->points[i].normal_x *= -1;
		cloud_normals->points[i].normal_y *= -1;
		cloud_normals->points[i].normal_z *= -1;
	}
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::concatenateFields (*cloud, *cloud_normals, *cloud_smoothed_normals);//x

	pcl::Poisson<pcl::PointNormal> poisson;

	poisson.setDepth(7);//9
	poisson.setInputCloud(cloud_smoothed_normals);
	poisson.setPointWeight(4);//4
	poisson.setSamplesPerNode(1.5);//1.5
	poisson.setScale(1.1);//1.1
	poisson.setIsoDivide(8);//8
	poisson.setConfidence(1);
	poisson.setManifold(0);
	poisson.setOutputPolygons(0);
	poisson.setSolverDivide(8);//8
	poisson.reconstruct(mesh);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::io::loadPCDFile(input_file, *cloudRGB);
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloudRGB);

    // Prepare a PointCloud with the same structure as the mesh vertices but to store RGB information
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(mesh.cloud, *mesh_cloud);

    // For each vertex in the mesh, find the nearest neighbor in the original cloud and assign its color
    for (size_t i = 0; i < mesh_cloud->points.size(); ++i) {
        pcl::PointXYZRGB& mesh_point = mesh_cloud->points[i];
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        // Find the nearest neighbor
        if (kdtree.nearestKSearch(mesh_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            // Assign the color from the nearest neighbor
            const pcl::PointXYZRGB& nearest_point = cloudRGB->points[pointIdxNKNSearch[0]];
            mesh_point.r = nearest_point.r;
            mesh_point.g = nearest_point.g;
            mesh_point.b = nearest_point.b;
        }
	}

	pcl::toPCLPointCloud2(*mesh_cloud, mesh.cloud);
	//saveMeshToPLY(mesh, output_file);

	pcl::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPolygonMesh(mesh, "meshes");
    viewer->initCameraParameters();

    std::cout << "Press q to exit." << std::endl;
    while (!viewer->wasStopped()) {
        viewer->spin();
    }

}