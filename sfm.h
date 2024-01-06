#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include "cameraCalibration.h"
#include "utilities.h"

class structureFromMotion {
    private:
        std::vector<cv::Mat> images;
        intrinsics camMatrix;
        std::vector<std::vector<cv::KeyPoint>> imagesKps;
        std::vector<cv::Mat> imagesDesc;
        std::vector<std::vector<cv::Point2d>> imagesPts2D;
        std::vector<cv::Matx34d> cameraPoses;
        std::set<int> doneViews;
        std::set<int> goodViews;
    public:
        std::vector<Point3D> reconstructionCloud;

        void loadImages();
        void featureDetect(detectorType detectorName, cv::Mat img, int imgIdx);
        void getFeatures();
        void featureMatch(matcherType matcherName, cv::Mat &descriptors_1, cv::Mat &descriptors_2, std::vector<cv::DMatch> &good_matches);
        void convert_to_float(cv::Mat &descriptors);
        void getCameraMatrix(const std::string path);
        bool CheckCoherentRotation(const cv::Mat_<double>& R);
        void kpsToPts(std::vector<cv::KeyPoint> keypoints, std::vector<cv::Point2d> &points2D);
        void alignPointsFromMatches(std::vector<cv::KeyPoint> kp_query, std::vector<cv::KeyPoint> kp_train, std::vector<cv::DMatch> matches, std::vector<cv::Point2d> &alignedQueryPts, std::vector<cv::Point2d> &alignedTrainPts);
        void backReferences(std::vector<int> &idLeftOrigen, std::vector<int> &idRightOrigen, std::vector<cv::DMatch> matches);
        int findHomographyInliers(std::vector<cv::KeyPoint> kp_query, std::vector<cv::KeyPoint> kp_train, std::vector<cv::DMatch> matches);
        std::map<float, std::pair<int, int>> findBestPair();
        void findCameraMatrices (intrinsics intrinsics, const int queryImageIdx, const int trainImageIdx, std::vector<cv::DMatch> matches, std::vector<cv::KeyPoint> kpLeft, std::vector<cv::KeyPoint> kpRight, cv::Matx34d& Pleft, cv::Matx34d& Pright);
        void triangulateViews(std::vector<cv::KeyPoint> kpQuery, std::vector<cv::KeyPoint> kpTrain, std::vector<cv::DMatch> matches, cv::Matx34d P1, cv::Matx34d P2, intrinsics camMatrix, std::pair<int, int> imgIdxPair, std::vector<Point3D> &pointcloud);
        void baseReconstruction();
        void addViews();
        void export_to_json(std:: string filename, cv::Mat matrix);
};