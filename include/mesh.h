#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/poisson.h>
#include <pcl/common/common.h>
// #include <pcl/PolygonMesh.h>

class Mesh {
	
	public:
		void filter_and_remove_points(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filterCloud);
		void saveMeshToPLY(const pcl::PolygonMesh& mesh, const std::string& filename);
		void create_mesh(std::string input_file, std::string output_file);

};