#include "sfm.h"

int main() {
	structureFromMotion sfm;

	sfm.loadImages();

	sfm.getCameraMatrix("C:/Programs and Stuff/vr3dmodels/calibration/cameraMatrix2.xml");

	sfm.getFeatures();

	sfm.baseReconstruction();

	// sfm.addViews();
	// cameraCalibration clb;
	// cv::Mat cameraMatrix;
	// cv::Mat distCoeffs;
	// cv::Mat R;
	// cv::Mat T;
	// clb.calibCamera(cameraMatrix, distCoeffs, R, T);

	// std::cout<<cameraMatrix<<"\n"<<distCoeffs<<"\n";

}

// int main() {

// 	cv::Mat img_1 = cv::imread("C:/Programs and Stuff/vr3dmodels/images/fountainLowRes/0003.jpg", cv::IMREAD_COLOR);
// 	cv::Mat img_2 = cv::imread("C:/Programs and Stuff/vr3dmodels/images/fountainLowRes/0008.jpg", cv::IMREAD_COLOR);

// 	std::vector<cv::KeyPoint> kp_1, kp_2;
// 	cv::Mat ds_1, ds_2;

// 	cv::Ptr<cv::xfeatures2d::SurfFeatureDetector> detector = cv::xfeatures2d::SurfFeatureDetector::create();
// 	detector->detectAndCompute(img_1, cv::noArray(), kp_1, ds_1);
// 	detector->detectAndCompute(img_2, cv::noArray(), kp_2, ds_2);

// 	cv::BFMatcher matcher(cv::NORM_L2);

// 	std::vector<std::vector<cv::DMatch>> knn_matches;

// 	matcher.knnMatch(ds_1, ds_2, knn_matches, 2);

// 	std::vector<cv::DMatch> good_matches;
// 	// const float ratio_thresh = 0.7f;
// 	const float ratio_thresh = 0.75;
// 	for (size_t i = 0; i < knn_matches.size(); i++) {
// 		if (knn_matches[i][0].distance <= ratio_thresh * knn_matches[i][1].distance) {
// 			good_matches.push_back(knn_matches[i][0]);
// 		}
// 	}

// 	cv::Mat img_matches;
// 	drawMatches(img_1, kp_1, img_2, kp_2,
// 				good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
// 				std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
// 	cv::imshow("Matches", img_matches);
// 	cv::waitKey(0);

// 	cv::Mat img_1 = cv::imread("C:/Programs and Stuff/vr3dmodels/images/S20Imgs/beer3.jpg", cv::IMREAD_COLOR);
// 	cv::Mat img_2 = cv::imread("C:/Programs and Stuff/vr3dmodels/images/S20Imgs/beer4.jpg", cv::IMREAD_COLOR);

// 	std::vector<cv::KeyPoint> keypoints_1;
// 	std::vector<cv::KeyPoint> keypoints_2;

// 	cv::Mat descriptors_1;
// 	cv::Mat descriptors_2;

// 	featureDetect(ORB, img_1, keypoints_1, descriptors_1);
// 	featureDetect(ORB, img_2, keypoints_2, descriptors_2);

// 	// // Convert the binary descriptors to floating point
//     // convert_to_float(descriptors_1);
// 	// convert_to_float(descriptors_2);

// 	std::vector<cv::DMatch> good_matches;

// 	featureMatch(BF, descriptors_1, descriptors_2, good_matches);

// 	// For ORB below
// 	// cv::Mat outputImg;
// 	// // Draw only keypoints location
// 	// drawKeypoints(img_1, keypoints_1, outputImg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
// 	// cv::imshow("ORBFeatureDetection", outputImg);
// 	// cv::waitKey(0);

// 	// // For BF below
// 	// cv::Mat img_matches;
// 	// drawMatches(img_1, keypoints_1, img_2, keypoints_2,
// 	// good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
// 	// std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	
// 	// // For FLANN below
// 	// cv::Mat img_matches;
// 	// drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_matches, cv::Scalar::all(-1),
// 	// 	cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

// 	// cv::imshow("Matches", img_matches);
// 	// cv::waitKey(0);

// 	// cameraCalibration calib;
// 	// cv::Mat K, dist, R, T;
// 	// calib.calibCamera(K, dist, R, T);

// 	std::vector<cv::Point2f> pts1, pts2;

// 	for (size_t i=0; i<good_matches.size(); i++) { // ?????
// 		pts1.push_back(keypoints_1[good_matches[i].queryIdx].pt);
// 		pts2.push_back(keypoints_2[good_matches[i].trainIdx].pt);
// 	}

// 	intrinsics camMatrix;
// 	getCameraMatrix("C:/Programs and Stuff/vr3dmodels/calibration/cameraMatrix.xml", camMatrix.K, camMatrix.distCoef);
// 	// std::cout<<camMatrix.K;

// 	cv::Mat cam_matrix = cv::Mat(camMatrix.K);
// 	cv::Mat mask;

// 	cv::Mat essential_mat = cv::findEssentialMat(pts1, pts2, cam_matrix, cv::RANSAC, 0.999, 1.0, mask);

// 	cv::Mat R, T;

// 	cv::recoverPose(essential_mat, pts1, pts2, cam_matrix, R, T, mask);

// 	bool success_rotation;
// 	success_rotation = CheckCoherentRotation(R);

// 	if(!success_rotation){
//       std::cerr<<"Invalid rotation.\n";
//   	}

// 	// std::cout<<R<<"\n";
// 	// std::cout<<T;

// 	cv::Matx34d P1 = cv::Matx34d::eye();
// 	cv::Matx34d P2 = cv::Matx34d(R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),T.at<double>(0),
//                   				 R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),T.at<double>(1),
//                   				 R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),T.at<double>(2));

// 	// std::cout<<P1<<"\n";
// 	// std::cout<<P2;

// 	cv::Mat normalizedLeftPts,normalizedRightPts;
//   	cv::undistortPoints(pts1, normalizedLeftPts, camMatrix.K, camMatrix.distCoef);
// 	cv::undistortPoints(pts2, normalizedRightPts, camMatrix.K, camMatrix.distCoef);

// 	cv::Mat pts3dHomogeneous;
// 	cv::triangulatePoints(P1, P2, normalizedLeftPts, normalizedRightPts, pts3dHomogeneous);

// 	cv::Mat pts3d;
// 	cv::convertPointsFromHomogeneous(pts3dHomogeneous.t(), pts3d);

// 	// std::cout<<pts3d;

// 	// export_to_json("matrix3dptsFixed", pts3d);

// }
