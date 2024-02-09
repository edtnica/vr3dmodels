#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "utilities.h"

class BundleAdjustment {

  private:

  public:
    BundleAdjustment(){

    }

    ~BundleAdjustment(){

    }

    static void adjustBundle(std::vector<Point3D>& pointCloud, std::vector<cv::Matx34d>& cameraPoses, Intrinsics& intrinsics,const std::vector<std::vector<cv::Point2d>>& image2dFeatures);

};