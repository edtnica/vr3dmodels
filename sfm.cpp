#include "sfm.h"

void structureFromMotion::loadImages() {

	std::cout<<"Loading images...\n";
	std::cout<<"----------------------------------------\n";
	std::vector<cv::String> imagesPaths;
    cv::glob("C:/Programs and Stuff/vr3dmodels/images/fountainLowRes/*.jpg", imagesPaths);

	std::sort(imagesPaths.begin(), imagesPaths.end());

	for (auto const &imagePath : imagesPaths) {
		cv::Mat image = cv::imread(imagePath);

		if (image.empty()) {
			std::cerr << "Could not load image " << imagePath << "\n";
			continue;
		}

		// cv::Mat image_copy = image.clone();
		// cv::Mat resize;

		// if (image_copy.rows > 480 && image_copy.cols > 640) {
		// 	cv::resize(image_copy,resize,cv::Size(),0.60,0.60); //Define a size of 640x480
		// 	images.push_back(resize);
		// }
		// else {
		// 	images.push_back(image_copy);
		// }
		// images.push_back(image_copy);
		images.push_back(image);
	}

	if (images.size() < 2) {
      std::cerr << "Not enough images\n";
	}

	// for (auto image : images) {
	// 	cv::imshow("Image", image);
	// 	cv::waitKey(0);
	// }

}

void structureFromMotion::showFeatures(cv::Mat img, std::vector<cv::KeyPoint> kps) {
	cv::Mat outputImg;
	// Draw only keypoints location
	drawKeypoints(img, kps, outputImg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
	cv::resize(outputImg, outputImg, cv::Size(1920, 1080));
	cv::imshow("FeatureDetection", outputImg);
	cv::waitKey(0);
}

void structureFromMotion::showMatches(int rightIdx, int leftIdx, std::vector<cv::DMatch> matches) {
	cv::Mat img_matches;
	drawMatches(images[rightIdx], imagesKps[rightIdx], images[leftIdx], imagesKps[leftIdx],
				matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
				std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	cv::resize(img_matches, img_matches, cv::Size(1920, 1080));
	cv::imshow("Matches", img_matches);
	cv::waitKey(0);
}

void structureFromMotion::featureDetect(detectorType detectorName, cv::Mat img, int imgIdx) {
	std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
	
	switch (detectorName) {
		case ORB: {
			cv::Ptr<cv::ORB> detector = cv::ORB::create(5000);
        	detector->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
			break;
		}
		case SURF: {
			cv::Ptr<cv::xfeatures2d::SurfFeatureDetector> detector = cv::xfeatures2d::SurfFeatureDetector::create();
			detector->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
			break;
		}
		case SIFT: {
			cv::Ptr<cv::Feature2D> detector = cv::SIFT::create();
			detector->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
			break;
		}
		case FAST: {
			cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
			detector->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
			break;
		}
		case AKAZE: {
			cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create();
    		detector->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
			break;
		}
		case BRISK: {
			cv::Ptr<cv::BRISK> detector = cv::BRISK::create();
			detector->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
			break;
		}
	}
	std::vector<cv::Point2d> pts2d;
	kpsToPts(keypoints,pts2d);

	imagesKps[imgIdx] = keypoints;
	imagesDesc[imgIdx] = descriptors;
	imagesPts2D[imgIdx] = pts2d;

	showFeatures(img, keypoints);
}

void structureFromMotion::getFeatures() {
	std::cout<<"Getting image features...\n";
	std::cout<<"----------------------------------------\n";
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

std::vector<cv::DMatch> structureFromMotion::pruneMatches(cv::Mat &desc_1, std::vector<cv::DMatch> matches) {
	std::vector<cv::DMatch> good_matches;
	
	double max_dist = 0;
	double min_dist = 100;

	for (int i=0; i<desc_1.rows; i++) {
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	for (int i=0; i<desc_1.rows; i++) {
		if (matches[i].distance <= std::max(2 * min_dist, 0.02))
			good_matches.push_back(matches[i]);
	}
	
	return good_matches;
}

std::vector<cv::DMatch> structureFromMotion::pruneMatchesWithLowe(std::vector<std::vector<cv::DMatch>> knnMatches) {
	std::vector<cv::DMatch> good_matches;
	// const float ratio_thresh = 0.7f;
	// const float ratio_thresh = 0.8f;
	const float ratio_thresh = 0.75;
	for (size_t i = 0; i < knnMatches.size(); i++) {
		if (knnMatches[i][0].distance <= ratio_thresh * knnMatches[i][1].distance) {
			good_matches.push_back(knnMatches[i][0]);
		}
	}
	return good_matches;
}

// void structureFromMotion::featureMatch(matcherType matcherName, cv::Mat &descriptors_1, cv::Mat &descriptors_2, std::vector<cv::DMatch> &good_matches) {
// 	switch (matcherName) {
// 		case FLANN: {
// 			cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
// 			std::vector<std::vector<cv::DMatch>> knn_matches;
			
// 			matcher->knnMatch(descriptors_1, descriptors_2, knn_matches, 2);

// 			good_matches = pruneMatchesWithLowe(knn_matches);

// 			break;
// 		}
// 		case BF: {
// 			// if NORM_L@2 or NORM_HAMMING !!!!!!!!!!!!!!

// 			// cv::BFMatcher matcher(cv::NORM_HAMMING);
// 			cv::BFMatcher matcher(cv::NORM_L2);
// 			// cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_L2);
// 			// std::vector<cv::DMatch> matches;
// 			// matcher->match(descriptors_1, descriptors_2, matches);

// 			std::vector<std::vector<cv::DMatch>> knn_matches;
// 			// matcher->knnMatch(descriptors_1, descriptors_2, knn_matches, 2);
// 			matcher.knnMatch(descriptors_1, descriptors_2, knn_matches, 2);

// 			good_matches = pruneMatchesWithLowe(knn_matches);

// 			break;
// 		}
// 	}
// }

std::vector<cv::DMatch> structureFromMotion::featureMatch(matcherType matcherName, cv::Mat &descriptors_1, cv::Mat &descriptors_2) {
	std::vector<cv::DMatch> good_matches;
	
	switch (matcherName) {
		case FLANN: {
			cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
			std::vector<std::vector<cv::DMatch>> knn_matches;
			
			matcher->knnMatch(descriptors_1, descriptors_2, knn_matches, 2);

			good_matches = pruneMatchesWithLowe(knn_matches);

			break;
		}
		case BF: {
			// cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
			cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_L2);
			// std::vector<cv::DMatch> matches;
			// matcher->match(descriptors_1, descriptors_2, matches);

			std::vector<std::vector<cv::DMatch>> knn_matches;
			matcher->knnMatch(descriptors_1, descriptors_2, knn_matches, 2);

			good_matches = pruneMatchesWithLowe(knn_matches);

			break;
		}
	}

	return good_matches;
}

void structureFromMotion::createFeatureMatchMatrix() {
	std::cout<<"Create feature match matrix...\n";
	std::cout<<"----------------------------------------\n";

	const size_t imagesNum = images.size();
	featureMatchMatrix.resize(imagesNum, std::vector<std::vector<cv::DMatch>>(imagesNum));

	std::vector<std::pair<int, int>> imagePairs;
    for (int i=0; i<imagesNum-1; i++) {
    	for (int j=i+1; j<imagesNum; j++) {
        	imagePairs.push_back({i, j});
    	}
    }

	size_t pairsNum = imagePairs.size();

	std::vector<std::thread> threads;

	const int threadsNum = std::thread::hardware_concurrency() - 1;
	
	int pairsPerThread;
	if (threadsNum > imagePairs.size()) pairsPerThread = 1;
	else pairsPerThread = (int)ceilf((float)(pairsNum)/threadsNum);

	std::cout<<"\n----------------------------------------\n";
	std::cout<<"Launch "<<threadsNum<<" threads with "<<pairsPerThread<<" pairs per thread\n";
	std::cout<<"----------------------------------------\n";

	for (int threadId = 0; threadId < MIN(threadsNum, pairsNum); threadId++) {
        threads.push_back(std::thread([&, threadId] {
            const int startingPair = pairsPerThread * threadId;

            for (int j = 0; j < pairsPerThread; j++) {
                int pairId = startingPair + j;
                if (pairId >= pairsNum) {
                    break;
                }
                std::pair<int, int> imgPair = imagePairs[pairId];

                featureMatchMatrix[imgPair.first][imgPair.second] = structureFromMotion::featureMatch(BF, imagesDesc[imgPair.first], imagesDesc[imgPair.second]);

				std::cout<<"Thread "<<threadId<<": Image pair -> "<<imgPair.first<<", "<<imgPair.second<<"\n";
            }
        }));
    }

	for (auto& t : threads) {
    	t.join();
    }

}

void structureFromMotion::convert_to_float(cv::Mat &descriptors) {
	// Convert the descriptors to floating point
	if(descriptors.type()!=CV_32F) {
		descriptors.convertTo(descriptors, CV_32F);
	}
}

// change name to getIntrinsics
void structureFromMotion::getCameraMatrix() {
	std::cout<<"Getting camera matrix...\n";
	std::cout<<"----------------------------------------\n";
	cv::Mat intrinsics;
	cv::Mat distCoeffs;

	// Read .xml file
	cv::FileStorage fs("C:/Programs and Stuff/vr3dmodels/calibration/cameraMatrix2.xml", cv::FileStorage::READ);

	// Get data from .xml file with Camera_Matrix tag
	fs["Camera_Matrix"] >> intrinsics;
	fs["Distortion_Coefficients"] >> distCoeffs;

    double fx = intrinsics.at<double>(0,0);
    double fy = intrinsics.at<double>(1,1);
    double cx = intrinsics.at<double>(0,2);
    double cy = intrinsics.at<double>(1,2);

	// cv::Mat_<double> cam_matrix = (cv::Mat_<double>(3, 3)<< fx, 0, cx,
    // 														0, fy, cy,
    //                                         				0,  0,  1);
	
	// cv::Mat_<double> cam_matrix = (cv::Mat_<double>(3, 3)<< 2759.48, 0, 1520.69,
    // 														0, 2764.16, 1006.81,
    //                                         				0,  0,  1);

	cv::Mat_<double> cam_matrix = (cv::Mat_<double>(3, 3)<< 1379.74, 0, 760.34,
    														0, 1382.08, 503.40,
                                            				0,  0,  1);

	camMatrix.K = cam_matrix;

	double k1 = distCoeffs.at<double>(0,0);
    double k2 = distCoeffs.at<double>(0,1);
    double k3 = distCoeffs.at<double>(0,2);
    double p1 = distCoeffs.at<double>(0,3);
    double p2 = distCoeffs.at<double>(0,4);

    // cv::Mat_<double> dist_coeffs = (cv::Mat_<double>(1, 5) << k1, k2, k3, p1, p2);
	cv::Mat_<double> dist_coeffs = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);

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
	alignedQueryPts.clear();
	alignedTrainPts.clear();

	for (size_t i=0; i<matches.size(); i++) {
		alignedQueryPts.push_back(kp_query[matches[i].queryIdx].pt);
		alignedTrainPts.push_back(kp_train[matches[i].trainIdx].pt);
	}
}

void structureFromMotion::backReferences(std::vector<int> &idLeftOrigin, std::vector<int> &idRightOrigin, std::vector<cv::DMatch> matches) {
	idLeftOrigin.clear();
	idRightOrigin.clear();

	for(size_t i=0; i<matches.size(); i++) {
        idLeftOrigin.push_back(matches[i].queryIdx);
        idRightOrigin.push_back(matches[i].trainIdx);
      }
}

int structureFromMotion::findHomographyInliers(std::vector<cv::Point2d> alignedL, std::vector<cv::Point2d> alignedR, std::vector<cv::DMatch> matches) {
	cv::Mat inliersMask;
	cv::Mat homography;

	if (matches.size() >= 4) {
		homography = cv::findHomography(alignedL, alignedR, cv::RANSAC, 3.0, inliersMask);
	}

	if (matches.size() < 4 || homography.empty()) {
		return 0;
	}

	return cv::countNonZero(inliersMask);
}

std::map<float, std::pair<int, int>> structureFromMotion::findBestPair() {
	std::cout<<"\n----------------------------------------\n";
	std::cout<<"Finding the best pair of images...\n";
	std::cout<<"----------------------------------------\n";

	std::map<float,std::pair<int,int>> numInliersMap;

	const size_t numOfImgs = images.size();

	for (int i=0; i<numOfImgs-1; i++) {
		for (int j=i+1; j<numOfImgs; j++) {

			if (featureMatchMatrix[i][j].size() < MIN_POINT_CT) {
				std::cout<<"Not enough matches for pair: "<<i<<", "<<j<<"\n";
				numInliersMap[1.0] = std::make_pair(i, j);
				continue;
			}

			std::vector<cv::Point2d> alignedLeft;
  			std::vector<cv::Point2d> alignedRight;

			alignPointsFromMatches(imagesKps[i], imagesKps[j], featureMatchMatrix[i][j], alignedLeft, alignedRight);

			// cv::Mat cam_matrix = cv::Mat(camMatrix.K);
			// cv::Mat mask;
			// cv::Mat essential_mat = cv::findEssentialMat(alignedLeft, alignedRight, cam_matrix, cv::RANSAC, 0.999, 1.0, mask);
			// std::vector<cv::DMatch> prunedMatches;

        	// for (unsigned i=0; i<mask.rows; i++) {
        	// 	if (mask.at<uchar>(i)) {
            //     	prunedMatches.push_back(good_matches[i]);
            // 	}
        	// }
			// float inliersRatio = (float)prunedMatches.size() / (float)good_matches.size();

			const int numInliers = findHomographyInliers(alignedLeft, alignedRight, featureMatchMatrix[i][j]);

			float inliersRatio = (float)numInliers / (float)featureMatchMatrix[i][j].size();

			numInliersMap[inliersRatio] = std::make_pair(i, j);

			std::cout<<"Pair: "<<i<<", "<<j<<" -> " <<inliersRatio<<" inliers ratio\n";
		}
	}
	return numInliersMap;

}

void structureFromMotion::pruneWithE(std::vector<cv::DMatch> matches, std::vector<cv::DMatch> &prunedMatches, cv::Mat mask) {
	prunedMatches.clear();

	for (int i=0; i<mask.rows; i++) {
		if (mask.at<uchar>(i)) {
			prunedMatches.push_back(matches[i]);
		}
	}
}

void structureFromMotion::findCameraMatrices (intrinsics intrinsics, const int queryImageIdx, const int trainImageIdx, std::vector<cv::DMatch> matches, std::vector<cv::KeyPoint> kpLeft, std::vector<cv::KeyPoint> kpRight, cv::Matx34d& Pleft, cv::Matx34d& Pright, std::vector<cv::DMatch> &prunedMatches) {
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

	Pleft = cv::Matx34d::eye();
	Pright = cv::Matx34d(R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),T.at<double>(0),
                  		 R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),T.at<double>(1),
                  		 R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),T.at<double>(2));

	pruneWithE(matches, prunedMatches, mask);

}

void structureFromMotion::triangulateViews(std::vector<cv::KeyPoint> kpQuery, std::vector<cv::KeyPoint> kpTrain, std::vector<cv::DMatch> matches, cv::Matx34d P1, cv::Matx34d P2, intrinsics cameraMatrix, std::pair<int, int> imgIdxPair, std::vector<Point3D> &pointcloud) {
	std::cout<<"Triangulating views...\n";

	pointcloud.clear();

	std::vector<cv::Point2d> alignedLeft;
  	std::vector<cv::Point2d> alignedRight;

	alignPointsFromMatches(kpQuery, kpTrain, matches, alignedLeft, alignedRight);

	std::vector<int> leftBackReference;
	std::vector<int> rightBackReference;

	backReferences(leftBackReference, rightBackReference, matches);

	cv::Mat normalizedLeftPts, normalizedRightPts;
  	cv::undistortPoints(alignedLeft, normalizedLeftPts, cameraMatrix.K, cameraMatrix.distCoef);
	cv::undistortPoints(alignedRight, normalizedRightPts, cameraMatrix.K, cameraMatrix.distCoef);

	cv::Mat pts3dHomogeneous;
	cv::triangulatePoints(P1, P2, normalizedLeftPts, normalizedRightPts, pts3dHomogeneous);

	cv::Mat pts3d;
	cv::convertPointsFromHomogeneous(pts3dHomogeneous.t(), pts3d);

	// std::cout<<pts3d;

	// export_to_json("fountainORBBFhighres", pts3d);

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
	std::map<float, std::pair<int, int>> bestViews = findBestPair();

	if (bestViews.size() <= 0) {
		std::cerr << "Could not obtain a good pair for the base reconstruction\n";
	}

	for (std::pair<const float, std::pair<int, int>>& bestPair : bestViews) {
		int queryImageIdx = bestPair.second.first;
    	int trainImageIdx = bestPair.second.second;
		std::cout<<"Base reconstruction with pair: "<<queryImageIdx<<", "<<trainImageIdx<<"\n";

		cv::Matx34d Pleft  = cv::Matx34d::eye();
    	cv::Matx34d Pright = cv::Matx34d::eye();

		std::vector<cv::DMatch> pruned_matches;

		findCameraMatrices(camMatrix, queryImageIdx, trainImageIdx, featureMatchMatrix[queryImageIdx][trainImageIdx], imagesKps[queryImageIdx], imagesKps[trainImageIdx], Pleft, Pright, pruned_matches);

		double inliersRatio = (double)pruned_matches.size() / (double)featureMatchMatrix[queryImageIdx][trainImageIdx].size();
		// std::cout<<"Inliers ratio: "<<inliersRatio<<"Pruned Matches: "<<pruned_matches.size()<<" Good Matches: "<<good_matches.size()<<"\n";
		if (inliersRatio < POSE_INLIERS_MINIMAL_RATIO) {
			std::cout<<"Not enough matches after pruning. Should try another pair...\n";
			continue;
		}

		showMatches(queryImageIdx, trainImageIdx, pruned_matches);

		featureMatchMatrix[queryImageIdx][trainImageIdx] = pruned_matches;

		std::vector<Point3D> pointCloud;
		triangulateViews(imagesKps[queryImageIdx], imagesKps[trainImageIdx], pruned_matches, Pleft, Pright, camMatrix, std::make_pair(queryImageIdx, trainImageIdx), pointCloud);

		reconstructionCloud = pointCloud;
		
		cameraPoses[queryImageIdx] = Pleft;
        cameraPoses[trainImageIdx] = Pright;

        doneViews.insert(queryImageIdx);
        doneViews.insert(trainImageIdx);

        goodViews.insert(queryImageIdx);
        goodViews.insert(trainImageIdx);

		break;
	}
}

void structureFromMotion::find2d3dmatches(int newView, std::vector<cv::Point3d> &points3D, std::vector<cv::Point2d> &points2D, std::vector<cv::DMatch> &bestMatches, int &doneView) {
	points3D.clear(); 
	points2D.clear();
    int queryImageIdx;
	int trainImageIdx;
    size_t bestMatchesNum = 0;
    std::vector<cv::DMatch> bestMatchs;

	for (int view:doneViews) {
        if (newView < view) {
            queryImageIdx = newView;
            trainImageIdx = view;
        }
		else {
            queryImageIdx = view;
            trainImageIdx = newView;
        }

		size_t matchesNum = featureMatchMatrix[queryImageIdx][trainImageIdx].size();
		if (matchesNum > bestMatchesNum) {
			bestMatchesNum = matchesNum;
			bestMatches = featureMatchMatrix[queryImageIdx][trainImageIdx];
			doneView = view;
		}

		for (Point3D cloudPoint:reconstructionCloud) {
			bool found2DPoint = false;

			for (std::pair<const int, int> origViewAndPoint:cloudPoint.idxImage) {
				int origViewIdx = origViewAndPoint.first;
            	int origViewFeatureIdx = origViewAndPoint.second;
			
				if (origViewIdx != doneView) continue;

				for (cv::DMatch match:bestMatches) {
					int matched2DPointInNewView = -1;

					if (origViewIdx < newView) {

                    	if (match.queryIdx == origViewFeatureIdx) {
                        	matched2DPointInNewView = match.trainIdx;
                    	}
                 	}
				 	else {

                    	if (match.trainIdx == origViewFeatureIdx) {
                        	matched2DPointInNewView = match.queryIdx;
                    	}
                 	}

					if (matched2DPointInNewView >= 0) {
						std::vector<cv::Point2d> newViewFeatures = imagesPts2D[newView];
						points2D.push_back(newViewFeatures.at(matched2DPointInNewView));
						points3D.push_back(cloudPoint.pt);
						found2DPoint = true;
						break;
                	}
				}
				if(found2DPoint) break;
			}
		}
	}
}

void structureFromMotion::findCameraPosePNP(intrinsics cameraMatrix, std::vector<cv::Point3d> pts3D, std::vector<cv::Point2d> pts2D, cv::Matx34d &P) {
	if (pts3D.size() <= 7 || pts2D.size() <= 7 || pts3D.size() != pts2D.size()) {
		std::cerr << "Could not find enough corresponding cloud points... (only " << pts3D.size() << ")\n";
	}

	cv::Mat rvec, T;
	std::vector<int> inliers;
	double minVal, maxVal;
	cv::minMaxIdx(pts2D, &minVal, &maxVal);
	cv::solvePnPRansac(pts3D, pts2D, cameraMatrix.K, cameraMatrix.distCoef, rvec, T, true, 1000, 0.006*maxVal, 0.99, inliers, 1);

	cv::Mat R;
  	cv::Rodrigues(rvec, R);

  	if (!CheckCoherentRotation(R)) std::cerr<<"Invalid rotation\n";

	P = cv::Matx34d(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), T.at<double>(0),
                  	R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), T.at<double>(1),
                  	R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), T.at<double>(2));

}

void structureFromMotion::addPoints(std::vector<Point3D> newPtCloud) {
	std::cout<<"Adding new points...\n";

	for (Point3D newPt:newPtCloud) {
		bool foundMatching3DPoint = false;
		for (Point3D& existingPt:reconstructionCloud) {
			if (cv::norm(existingPt.pt - newPt.pt) < 0.01) {
				foundMatching3DPoint = true;
				break;
			}
		}
		if (!foundMatching3DPoint) reconstructionCloud.push_back(newPt);
	}
}

void structureFromMotion::addViews() {
	std::vector<cv::Point3d> points3d;
    std::vector<cv::Point2d> points2d;

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

		for (int frame:newFrames) {
			if (doneViews.count(frame) == 1) continue;

			std::vector<cv::DMatch> bestMatches;
			int doneView;

			find2d3dmatches(frame, points3d, points2d, bestMatches, doneView);

			doneViews.insert(frame);

			cv::Matx34d newP = cv::Matx34d::eye();

			findCameraPosePNP(camMatrix, points3d, points2d, newP);

			cameraPoses[frame] = newP;

			std::vector<Point3D> newPointCloud;

			for (int goodView:goodViews) {
				int queryImageIdx;
				int trainImageIdx;

				if (frame < goodView) {
					queryImageIdx = frame;
					trainImageIdx = goodView;
				}
				else {
					queryImageIdx = goodView;
					trainImageIdx = frame;
				}

				triangulateViews(imagesKps[queryImageIdx], imagesKps[trainImageIdx], featureMatchMatrix[queryImageIdx][trainImageIdx], cameraPoses[queryImageIdx], cameraPoses[trainImageIdx], camMatrix, std::make_pair(queryImageIdx, trainImageIdx), newPointCloud);
			
				addPoints(newPointCloud);
			
			}

			goodViews.insert(frame);
			// adjustBundle();

		}
	}
}

void structureFromMotion::export_to_json(std:: string filename, cv::Mat matrix) {
    cv::FileStorage fs("C:/Programs and Stuff/vr3dmodels/jsonOutput/" + filename + ".json", cv::FileStorage::WRITE);
    fs<<"Matrix"<<matrix;
    fs.release();
}

void structureFromMotion::setLogging() {
	cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_SILENT);
}