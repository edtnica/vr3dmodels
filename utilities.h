enum detectorType {
	ORB,
	SURF
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