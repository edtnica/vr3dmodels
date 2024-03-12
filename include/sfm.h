#include <iostream>
#include <fstream>
#include <thread>
#include <filesystem>
// #include <opencv2/features2d.hpp>
// #include <opencv2/xfeatures2d.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/calib3d.hpp>
#include <opencv2/core/utils/logger.hpp>
#include "ioUtils.h"
#include "cameraCalibration.h"
// #include "utilities.h"
#include "sfmBundleAdjustment.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

class structureFromMotion {
    typedef std::map<int, image2D3DPair> image2D3DMatches;

    private:
		std::string reconstructionName;
        std::vector<cv::Mat> images;
        intrinsics camMatrix;
        std::vector<std::vector<cv::KeyPoint>> imagesKps;
        std::vector<cv::Mat> imagesDesc;
        std::vector<std::vector<cv::Point2d>> imagesPts2D;
        std::vector<std::string> imagesPaths;
        std::vector<std::vector<std::vector<cv::DMatch>>> featureMatchMatrix;
        std::vector<cv::Matx34d> cameraPoses;
        std::set<int> doneViews;
        std::set<int> goodViews;

        int MIN_POINT_CT = 100;
        float POSE_INLIERS_MINIMAL_RATIO = 0.5;
		
    public:
        std::vector<Point3D> reconstructionCloud;

		structureFromMotion(std::string reconstructionName);
		
        void loadImages();
        void showFeatures(cv::Mat img, std::vector<cv::KeyPoint> kps);
        void showMatches(int rightIdx, int leftIdx, std::vector<cv::DMatch> matches);
        void featureDetect(detectorType detectorName, cv::Mat img, int imgIdx);
        void getFeatures();
        std::vector<cv::DMatch> pruneMatches(cv::Mat &desc_1, std::vector<cv::DMatch> matches);
        std::vector<cv::DMatch> pruneMatchesWithLowe(std::vector<std::vector<cv::DMatch>> knnMatches);
        void featureMatch(matcherType matcherName, cv::Mat &descriptors_1, cv::Mat &descriptors_2, std::vector<cv::DMatch> &good_matches);
        std::vector<cv::DMatch> featureMatch(matcherType matcherName, cv::Mat &descriptors_1, cv::Mat &descriptors_2);
        void createFeatureMatchMatrix();
        void convert_to_float(cv::Mat &descriptors);
        void getCameraMatrix();
        bool CheckCoherentRotation(const cv::Mat_<double>& R);
        void kpsToPts(std::vector<cv::KeyPoint> keypoints, std::vector<cv::Point2d> &points2D);
        void alignPointsFromMatches(std::vector<cv::KeyPoint> kp_query, std::vector<cv::KeyPoint> kp_train, std::vector<cv::DMatch> matches, std::vector<cv::Point2d> &alignedQueryPts, std::vector<cv::Point2d> &alignedTrainPts);
        void backReferences(std::vector<int> &idLeftOrigen, std::vector<int> &idRightOrigen, std::vector<cv::DMatch> matches);
        int findHomographyInliers(std::vector<cv::Point2d> alignedL, std::vector<cv::Point2d> alignedR, std::vector<cv::DMatch> matches);
        std::map<float, std::pair<int, int>> findBestPair();
        void pruneWithE(std::vector<cv::DMatch> matches, std::vector<cv::DMatch> &prunedMatches, cv::Mat mask);
        void findCameraMatrices(intrinsics intrinsics, const int queryImageIdx, const int trainImageIdx, std::vector<cv::DMatch> matches, std::vector<cv::KeyPoint> kpLeft, std::vector<cv::KeyPoint> kpRight, cv::Matx34d& Pleft, cv::Matx34d& Pright, std::vector<cv::DMatch> &prunedMatches);
        void triangulateViews(std::vector<cv::KeyPoint> kpQuery, std::vector<cv::KeyPoint> kpTrain, std::vector<cv::DMatch> matches, cv::Matx34d P1, cv::Matx34d P2, intrinsics cameraMatrix, std::pair<int, int> imgIdxPair, std::vector<Point3D> &pointcloud);
        void baseReconstruction();
        // void find2d3dmatches(int newView, std::vector<cv::Point3d> &points3D, std::vector<cv::Point2d> &points2D, int &doneView);
        image2D3DMatches find2d3dmatches();
        bool findCameraPosePNP(intrinsics cameraMatrix, std::vector<cv::Point3d> pts3D, std::vector<cv::Point2d> pts2D, cv::Matx34d &P);
        void addPoints(std::vector<Point3D> newPtCloud);
        void addViews();
		void run_reconstruction();
        void pointcloud_to_ply(const std::string &filename);
		// void ply_to_pcd(std::string plyFilePath, std::string pcdFilePath);
        void PMVS2();
        void export_to_json(std:: string filename, cv::Mat matrix);
        void setLogging();
};
