#include "sfm.h"

void structureFromMotion::loadImages() {

	std::cout << "Loading images...\n";
	std::vector<cv::String> imagesPaths;
    cv::glob("C:/Programs and Stuff/vr3dmodels/images/paintingLowRes/*.jpg", imagesPaths);

	std::sort(imagesPaths.begin(), imagesPaths.end());

	for (auto const &imagePath : imagesPaths) {
		cv::Mat image = cv::imread(imagePath);

		if (image.empty()) {
			std::cerr << "Could not load image " << imagePath << "\n";
		}

		cv::Mat image_copy = image.clone();
		cv::Mat resize;

		if (image_copy.rows > 480 && image_copy.cols > 640) {
			cv::resize(image_copy,resize,cv::Size(),0.60,0.60); //Define a size of 640x480
			images.push_back(resize);
		}
		else {
			images.push_back(image_copy);
		}
		// images.push_back(image_copy);
	}

	if (images.size() < 2) {
      std::cerr << "Not enough images\n";
	}

	// for (auto image : images) {
	// 	cv::imshow("Image", image);
	// 	cv::waitKey(0);
	// }

}

void structureFromMotion::featureDetect(detectorType detectorName, cv::Mat img, int imgIdx) {
	switch (detectorName) {
		// case ORB: {
		// 	cv::Ptr<cv::ORB> detector= cv::ORB::create();

		// 	std::vector<cv::KeyPoint> keypoints;
        // 	cv::Mat descriptors;
        // 	detector->detectAndCompute(img, cv::noArray(), keypoints, descriptors, false);

		// 	std::vector<cv::Point2d> pts2d;
        // 	kpsToPts(keypoints,pts2d);

        // 	imagesKps[imgIdx] = keypoints;
        // 	imagesDesc[imgIdx] = descriptors;
        // 	imagesPts2D[imgIdx] = pts2d;

		// 	cv::Mat outputImg;
		// 	// Draw only keypoints location
		// 	drawKeypoints(img, keypoints, outputImg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
		// 	cv::imshow("ORBFeatureDetection", outputImg);
		// 	cv::waitKey(0);

		// 	break;
		// }
		case ORB: {
			cv::Ptr<cv::FeatureDetector> orbDetector = cv::ORB::create();
			cv::Ptr<cv::DescriptorExtractor> orbDescriptor = cv::ORB::create();
			
			std::vector<cv::KeyPoint> keypoints;
			cv::Mat descriptors;

			orbDetector->detect(img, keypoints);
			orbDescriptor->compute(img, keypoints, descriptors);

			std::vector<cv::Point2d> pts2d;
        	kpsToPts(keypoints,pts2d);

        	imagesKps[imgIdx] = keypoints;
        	imagesDesc[imgIdx] = descriptors;
        	imagesPts2D[imgIdx] = pts2d;

			cv::Mat outputImg;
			// Draw only keypoints location
			drawKeypoints(img, keypoints, outputImg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
			cv::imshow("ORBFeatureDetection", outputImg);
			cv::waitKey(0);

			break;
		}
		case SURF: {
			// cv::Ptr<cv::xfeatures2d::SurfFeatureDetector> detector = cv::xfeatures2d::SurfFeatureDetector::create();
			// detector->detectAndCompute(img_1, cv::noArray(), keypoints, descriptors);
			break;
		}
	}
}

void structureFromMotion::getFeatures() {
	std::cout<<"Getting image features...\n";
	// maybe use .insert() for vectors
	cameraPoses.resize(images.size());
	imagesKps.resize(images.size(),std::vector<cv::KeyPoint>());
  	imagesDesc.resize(images.size(),cv::Mat());
  	imagesPts2D.resize(images.size(),std::vector<cv::Point2d>());

	for (int i=0; i<images.size(); i++) {
		const cv::Mat img = images.at(i);
		featureDetect(ORB, img, i);
	}
}

void structureFromMotion::featureMatch(matcherType matcherName, cv::Mat &descriptors_1, cv::Mat &descriptors_2, std::vector<cv::DMatch> &good_matches) {
	switch (matcherName) {
		case FLANN: {
			// Match descriptors with a FLANN based matcher
			cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
			// cv::Ptr<cv::DescriptorMatcher> matcher = new cv::BFMatcher(cv::NORM_L2, false);
			std::vector<std::vector<cv::DMatch>> knn_matches;
			matcher->knnMatch(descriptors_1, descriptors_2, knn_matches, 2);

			// Filter matches using the Lowe's ratio test
			const float ratio_thresh = 0.7f;
			for (size_t i = 0; i < knn_matches.size(); i++) {
				if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
					good_matches.push_back(knn_matches[i][0]);
				}
			}
			break;
		}
		case BF: {
			cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
			std::vector<cv::DMatch> matches;
			matcher->match(descriptors_1, descriptors_2, matches);

			double max_dist = 0;
			double min_dist = 100;

			for (int i=0; i<descriptors_1.rows; i++) {
				double dist = matches[i].distance;
				if (dist < min_dist)
					min_dist = dist;
				if (dist > max_dist)
					max_dist = dist;
			}

			for (int i=0; i<descriptors_1.rows; i++) {
				if (matches[i].distance <= std::max(2 * min_dist, 0.02))
					good_matches.push_back(matches[i]);
			}
			break;
		}
	}
}

void structureFromMotion::convert_to_float(cv::Mat &descriptors) {
	// Convert the descriptors to floating point
	if(descriptors.type()!=CV_32F) {
		descriptors.convertTo(descriptors, CV_32F);
	}
}

// change name to getIntrinsics
void structureFromMotion::getCameraMatrix(const std::string path) {
	std::cout << "Getting camera matrix...\n";
	cv::Mat intrinsics;
	cv::Mat distCoeffs;

	// Read .xml file
	cv::FileStorage fs(path, cv::FileStorage::READ);

	// Get data from .xml file with Camera_Matrix tag
	fs["Camera_Matrix"] >> intrinsics;
	fs["Distortion_Coefficients"] >> distCoeffs;

    double fx = intrinsics.at<double>(0,0);
    double fy = intrinsics.at<double>(1,1);
    double cx = intrinsics.at<double>(0,2);
    double cy = intrinsics.at<double>(1,2);

	cv::Mat_<double> cam_matrix = (cv::Mat_<double>(3, 3)<< fx, 0, cx,
    														0, fy, cy,
                                            				0,  0,  1);
	
	// cv::Mat_<double> cam_matrix = (cv::Mat_<double>(3, 3)<< 2759.48, 0, 1520.69,
    // 														0, 2764.16, 1006.81,
    //                                         				0,  0,  1);

	// cv::Mat_<double> cam_matrix = (cv::Mat_<double>(3, 3)<< 1379.74, 0, 760.34,
    // 														0, 1382.08, 503.40,
    //                                         				0,  0,  1);

	camMatrix.K = cam_matrix;

	double k1 = distCoeffs.at<double>(0,0);
    double k2 = distCoeffs.at<double>(0,1);
    double k3 = distCoeffs.at<double>(0,2);
    double p1 = distCoeffs.at<double>(0,3);
    double p2 = distCoeffs.at<double>(0,4);

    cv::Mat_<double> dist_coeffs = (cv::Mat_<double>(1, 5) << k1, k2, k3, p1, p2);
	// cv::Mat_<double> dist_coeffs = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);

	camMatrix.distCoef = dist_coeffs;
}

bool structureFromMotion::CheckCoherentRotation(const cv::Mat_<double>& R) {

    if (fabsf(determinant(R)) - 1.0 > 1e-07) {
        std::cout<<"Invalid rotation matrix\n";
        return false;
    }
    return true;
}

void structureFromMotion::kpsToPts(std::vector<cv::KeyPoint> keypoints, std::vector<cv::Point2d> &points2D){

	points2D.clear();
	for (const cv::KeyPoint& kp: keypoints) {
    	points2D.push_back(kp.pt);
	}
}

void structureFromMotion::alignPointsFromMatches(std::vector<cv::KeyPoint> kp_query, std::vector<cv::KeyPoint> kp_train, std::vector<cv::DMatch> matches, std::vector<cv::Point2d> &alignedQueryPts, std::vector<cv::Point2d> &alignedTrainPts) {
	// maybe Point2f instead of Point2d
	for (size_t i=0; i<matches.size(); i++) {
		alignedQueryPts.push_back(kp_query[matches[i].queryIdx].pt);
		alignedTrainPts.push_back(kp_train[matches[i].trainIdx].pt);
	}
}

void structureFromMotion::backReferences(std::vector<int> &idLeftOrigin, std::vector<int> &idRightOrigin, std::vector<cv::DMatch> matches) {
	for(size_t i=0; i<matches.size(); i++) {
        idLeftOrigin.push_back(matches[i].queryIdx);
        idRightOrigin.push_back(matches[i].trainIdx);
      }
}

int structureFromMotion::findHomographyInliers(std::vector<cv::KeyPoint> kp_query, std::vector<cv::KeyPoint> kp_train, std::vector<cv::DMatch> matches) {
	std::vector<cv::Point2d> alignedLeft;
  	std::vector<cv::Point2d> alignedRight;

	alignPointsFromMatches(kp_query, kp_train, matches, alignedLeft, alignedRight);

	cv::Mat inliersMask;
	cv::Mat homography;

	if (matches.size() >= 4) {
		homography = cv::findHomography(alignedLeft, alignedRight, cv::RANSAC, 3.0, inliersMask);
	}

	if (matches.size() < 4 || homography.empty()) {
		return 0;
	}

	return cv::countNonZero(inliersMask);
}

std::map<float, std::pair<int, int>> structureFromMotion::findBestPair() {
	// std::pair<cv::Mat, cv::Mat> imagePair;
	std::map<float,std::pair<int,int>> numInliersMap;
	// std::vector<cv::Mat> images;
	// loadImages(images);

	// intrinsics camMatrix;
	// getCameraMatrix("C:/Programs and Stuff/vr3dmodels/calibration/cameraMatrix.xml", camMatrix.K, camMatrix.distCoef);

	const size_t numOfImgs = images.size();

	cv::Mat queryImg;
	cv::Mat trainImg;

	// std::vector<cv::KeyPoint> kp_query;
	// std::vector<cv::KeyPoint> kp_train;

	// cv::Mat descriptors_query;
	// cv::Mat descriptors_train;

	for (int i=0; i<numOfImgs-1; i++) {
		for (int j=i+1; j<numOfImgs; j++) {
			queryImg = images[i];
			trainImg = images[j];

			// featureDetect(ORB, queryImg, kp_query, descriptors_query);
			// featureDetect(ORB, trainImg, kp_train, descriptors_train);

			std::vector<cv::DMatch> good_matches;

			featureMatch(BF, imagesDesc[i], imagesDesc[j], good_matches);

			std::vector<cv::Point2d> alignedLeft;
  			std::vector<cv::Point2d> alignedRight;

			alignPointsFromMatches(imagesKps[i], imagesKps[j], good_matches, alignedLeft, alignedRight);

			cv::Mat cam_matrix = cv::Mat(camMatrix.K);
			cv::Mat mask;

			cv::Mat essential_mat = cv::findEssentialMat(alignedLeft, alignedRight, cam_matrix, cv::RANSAC, 0.999, 1.0, mask);

			const int numInliers = findHomographyInliers(imagesKps[i], imagesKps[j], good_matches);

			std::vector<cv::DMatch> prunedMatches;

        	for (unsigned i=0; i<mask.rows; i++) {
        		if (mask.at<uchar>(i)) {
                	prunedMatches.push_back(good_matches[i]);
            	}
        	}

			float poseInliersRatio = (float)prunedMatches.size() / (float)good_matches.size();

			numInliersMap[poseInliersRatio] = std::make_pair(i, j);

		}
	}

	return numInliersMap;

}

void structureFromMotion::findCameraMatrices (intrinsics intrinsics, const int queryImageIdx, const int trainImageIdx, std::vector<cv::DMatch> matches, std::vector<cv::KeyPoint> kpLeft, std::vector<cv::KeyPoint> kpRight, cv::Matx34d& Pleft, cv::Matx34d& Pright) {
	std::vector<cv::DMatch> prunedMatches;
	// prunedWithHom() ...

	std::vector<cv::Point2d> alignedLeft;
  	std::vector<cv::Point2d> alignedRight;

	alignPointsFromMatches(kpLeft, kpRight, matches, alignedLeft, alignedRight);

	cv::Mat cam_matrix = cv::Mat(intrinsics.K);
	cv::Mat mask;

	cv::Mat essential_mat = cv::findEssentialMat(alignedLeft, alignedRight, cam_matrix, cv::RANSAC, 0.999, 1.0, mask);

	cv::Mat R, T;

	cv::recoverPose(essential_mat, alignedLeft, alignedRight, cam_matrix, R, T, mask);

	bool success_rotation;
	success_rotation = CheckCoherentRotation(R);

	if(!success_rotation){
      std::cerr<<"Invalid rotation.\n";
  	}

	// std::cout<<R<<"\n";
	// std::cout<<T;

	Pleft = cv::Matx34d::eye();
	Pright = cv::Matx34d(R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),T.at<double>(0),
                  		 R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),T.at<double>(1),
                  		 R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),T.at<double>(2));


}

void structureFromMotion::triangulateViews(std::vector<cv::KeyPoint> kpQuery, std::vector<cv::KeyPoint> kpTrain, std::vector<cv::DMatch> matches, cv::Matx34d P1, cv::Matx34d P2, intrinsics camMatrix, std::pair<int, int> imgIdxPair, std::vector<Point3D> &pointcloud) {
	std::cout<<"Triangulating views...\n";

	pointcloud.clear();

	std::vector<cv::Point2d> alignedLeft;
  	std::vector<cv::Point2d> alignedRight;

	alignPointsFromMatches(kpQuery, kpTrain, matches, alignedLeft, alignedRight);

	std::vector<int> leftBackReference;
	std::vector<int> rightBackReference;

	backReferences(leftBackReference, rightBackReference, matches);

	cv::Mat normalizedLeftPts,normalizedRightPts;
  	cv::undistortPoints(alignedLeft, normalizedLeftPts, camMatrix.K, camMatrix.distCoef);
	cv::undistortPoints(alignedRight, normalizedRightPts, camMatrix.K, camMatrix.distCoef);

	cv::Mat pts3dHomogeneous;
	cv::triangulatePoints(P1, P2, normalizedLeftPts, normalizedRightPts, pts3dHomogeneous);

	cv::Mat pts3d;
	cv::convertPointsFromHomogeneous(pts3dHomogeneous.t(), pts3d);

	std::cout<<pts3d;

	// export_to_json("matrix3dpts3", pts3d);

	for(int i=0; i<pts3d.rows; i++) {
		Point3D p;
        p.pt = cv::Point3d(pts3d.at<double>(i, 0),
                           pts3d.at<double>(i, 1),
                           pts3d.at<double>(i, 2));

		p.idxImage[imgIdxPair.first]  = leftBackReference[i];
        p.idxImage[imgIdxPair.second] = rightBackReference[i];
		p.pt2D[imgIdxPair.first]=imagesPts2D.at(imgIdxPair.first).at(leftBackReference[i]);
        p.pt2D[imgIdxPair.second]=imagesPts2D.at(imgIdxPair.second).at(rightBackReference[i]);
		
		pointcloud.push_back(p);
	}

}

void structureFromMotion::baseReconstruction() {

	getFeatures();
	// std::vector<cv::Mat> images;
	// loadImages(images);

	// intrinsics intrins;
	// getCameraMatrix("C:/Programs and Stuff/vr3dmodels/calibration/cameraMatrix.xml", intrins.K, intrins.distCoef);

	cv::Mat queryImg;
	cv::Mat trainImg;

	// std::vector<cv::KeyPoint> kp_query;
	// std::vector<cv::KeyPoint> kp_train;

	// cv::Mat descriptors_query;
	// cv::Mat descriptors_train;

	std::map<float, std::pair<int, int>> bestViews = findBestPair();

	if (bestViews.size() <= 0) {
		std::cerr << "Could not obtain a good pair for the base reconstruction\n";
	}

	for (std::pair<const float, std::pair<int, int>>& bestPair : bestViews) {
		int queryImageIdx = bestPair.second.first;
    	int trainImageIdx = bestPair.second.second;
		std::cout<<"Base reconstruction with pair: "<<queryImageIdx<<" "<<trainImageIdx<<".\n";

		queryImg = images[queryImageIdx];
		trainImg = images[trainImageIdx];

		// featureDetect(ORB, queryImg, kp_query, descriptors_query);
		// featureDetect(ORB, trainImg, kp_train, descriptors_train);

		std::vector<cv::DMatch> good_matches;

		featureMatch(BF, imagesDesc[queryImageIdx], imagesDesc[trainImageIdx], good_matches);

		cv::Mat img_matches;
		drawMatches(queryImg, imagesKps[queryImageIdx], trainImg, imagesKps[trainImageIdx],
					good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
					std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		cv::imshow("Matches", img_matches);
		cv::waitKey(0);

		cv::Matx34d Pleft  = cv::Matx34d::eye();
    	cv::Matx34d Pright = cv::Matx34d::eye();

		findCameraMatrices(camMatrix, queryImageIdx, trainImageIdx, good_matches, imagesKps[queryImageIdx], imagesKps[trainImageIdx], Pleft, Pright);

		std::vector<Point3D> pointCloud;
		triangulateViews(imagesKps[queryImageIdx], imagesKps[trainImageIdx], good_matches, Pleft, Pright, camMatrix, std::make_pair(queryImageIdx,trainImageIdx), pointCloud);
		
		reconstructionCloud = pointCloud;
		std::cout<<"pule1\n";
		cameraPoses[queryImageIdx] = Pleft;
        cameraPoses[trainImageIdx] = Pright;
		std::cout<<"pule2\n";
        doneViews.insert(queryImageIdx);
        doneViews.insert(trainImageIdx);
		std::cout<<"pule3\n";
        goodViews.insert(queryImageIdx);
        goodViews.insert(trainImageIdx);
		std::cout<<"pule4\n";
		break;

	}
}

void structureFromMotion::addViews() {
	while (doneViews.size() < images.size()) {
		std::set<int> newFrames;

		for (int viewsIdx:doneViews) {
			int view1;
			int view2;

			if (viewsIdx == 0) {
                view1 = viewsIdx;
                view2 = viewsIdx+1;
            }
			else if (viewsIdx == images.size()-1) {
                view1 = viewsIdx-1;
                view2 = viewsIdx;
            }
			else {
            	view1 = viewsIdx-1;
                view2 = viewsIdx+1;
            }

			if (doneViews.count(view1) == 0) newFrames.insert(view1);
			if (doneViews.count(view2) == 0) newFrames.insert(view2);

		}

		for (int frame : newFrames) {
			if (doneViews.count(frame) == 1) continue;

			
		}
	}
}

void structureFromMotion::export_to_json(std:: string filename, cv::Mat matrix) {
    cv::FileStorage fs("C:/Programs and Stuff/vr3dmodels/" + filename + ".json", cv::FileStorage::WRITE);
    fs<<"Matrix"<<matrix;
    fs.release();
}