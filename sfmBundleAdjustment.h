#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "utilities.h"

class sfmBundleAdjustment {

  private:

  public:
    sfmBundleAdjustment(){

    }

    ~sfmBundleAdjustment(){

    }

    static void adjustBundle(std::vector<Point3D>& pointCloud, std::vector<cv::Matx34d>& cameraPoses, intrinsics& intrinsics,const std::vector<std::vector<cv::Point2d>>& image2dFeatures);

};