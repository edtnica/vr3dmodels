#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

enum detectorType {
	ORB,
	SURF,
	SIFT,
	FAST,
	AKAZE,
	BRISK
};

enum matcherType {
	FLANN,
	BF
};

struct intrinsics {
	cv::Mat_<double> K;
	cv::Mat_<double> distCoef;
};

struct Point3D {
    cv::Point3d pt;
    std::map<const int,int> idxImage;
    std::map<const int,cv::Point2d> pt2D;

};