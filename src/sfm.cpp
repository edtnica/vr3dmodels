#include "../include/sfm.h"

structureFromMotion::structureFromMotion(std::string reconstructionName) {
	this->reconstructionName = reconstructionName;
}

void structureFromMotion::loadImages() {

	std::cout<<"--------------------------------------------------\n";
	std::cout<<"               Loading images...\n";
	std::cout<<"--------------------------------------------------\n";
	std::vector<cv::String> imgPaths;
    cv::glob("../images/fountain/*.jpg", imgPaths);

	std::sort(imgPaths.begin(), imgPaths.end());

	for (auto const &imgPath : imgPaths) {
		// std::cout<<imgPath<<"\n";
		cv::Mat image = cv::imread(imgPath);

		if (image.empty()) {
			std::cerr << "Could not load image " << imgPath << "\n";
			continue;
		}

		images.push_back(image);
		imagesPaths.push_back(imgPath);
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
	std::cout<<"--------------------------------------------------\n";
	std::cout<<"           Getting image features...\n";
	std::cout<<"--------------------------------------------------\n";
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
	const float ratio_thresh = 0.7;
	for (size_t i = 0; i < knnMatches.size(); i++) {
		if (knnMatches[i][0].distance <= ratio_thresh * knnMatches[i][1].distance) {
			good_matches.push_back(knnMatches[i][0]);
		}
	}
	return good_matches;
}

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
	std::cout<<"--------------------------------------------------\n";
	std::cout<<"           Create the match matrix...\n";
	std::cout<<"--------------------------------------------------\n\n";

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

	std::cout<<"Launch "<<threadsNum<<" threads with "<<pairsPerThread<<" pairs per thread\n";

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
void structureFromMotion::getIntrinsics() {
	std::cout<<"--------------------------------------------------\n";
	std::cout<<"            Getting camera matrix...\n";
	std::cout<<"--------------------------------------------------\n";
	cv::Mat intrinsics;
	cv::Mat distCoeffs;

	// Read .xml file
	cv::FileStorage fs("../calibration/cameraMatrix2.xml", cv::FileStorage::READ);

	// Get data from .xml file with Camera_Matrix tag
	fs["Camera_Matrix"] >> intrinsics;
	fs["Distortion_Coefficients"] >> distCoeffs;

    double fx = intrinsics.at<double>(0,0);
    double fy = intrinsics.at<double>(1,1);
    double cx = intrinsics.at<double>(0,2);
    double cy = intrinsics.at<double>(1,2);

	// cv::Mat_<double> cam_matrix = (cv::Mat_<double>(3, 3)<< fx, 0, cx,
    														// 0, fy, cy,
                                            				// 0,  0,  1);
	
	cv::Mat_<double> cam_matrix = (cv::Mat_<double>(3, 3)<< 2759.48, 0, 1520.69,
    														0, 2764.16, 1006.81,
                                            				0,  0,  1);

	// cv::Mat_<double> cam_matrix = (cv::Mat_<double>(3, 3)<< 1379.74, 0, 760.34,
    // 														0, 1382.08, 503.40,
    //                                         				0,  0,  1);

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
	std::cout<<"Finding the best pair of images...\n";

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

	// export_to_json("fountain_ORB_BF", pts3d);

	cv::Mat rvecLeft;
    cv::Rodrigues(P1.get_minor<3, 3>(0, 0), rvecLeft);
    cv::Mat tvecLeft(P1.get_minor<3, 1>(0, 3).t());

    std::vector<cv::Point2d> projectedOnLeft(alignedLeft.size());
    cv::projectPoints(pts3d, rvecLeft, tvecLeft, cameraMatrix.K, cv::Mat(), projectedOnLeft);

    cv::Mat rvecRight;
    cv::Rodrigues(P2.get_minor<3, 3>(0, 0), rvecRight);
    cv::Mat tvecRight(P2.get_minor<3, 1>(0, 3).t());

    std::vector<cv::Point2d> projectedOnRight(alignedRight.size());
    cv::projectPoints(pts3d, rvecRight, tvecRight, cameraMatrix.K, cv::Mat(), projectedOnRight);


	for (int i=0; i<pts3d.rows; i++) {
		if (cv::norm(projectedOnLeft[i]  - alignedLeft[i])  > 5 && cv::norm(projectedOnRight[i] - alignedRight[i]) > 5) {
            continue;
        }


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
	std::cout << "\n--------------------------------------------------\n";
	std::cout << "          Starting base reconstruction...\n";
	std::cout << "--------------------------------------------------\n\n";
	
	std::map<float, std::pair<int, int>> bestViews = findBestPair();

	if (bestViews.size() <= 0) {
		std::cerr << "Could not obtain a good pair for the base reconstruction\n";
	}

	for (std::pair<const float, std::pair<int, int>>& bestPair : bestViews) {
		int queryImageIdx = bestPair.second.first;
    	int trainImageIdx = bestPair.second.second;
		std::cout << "Base reconstruction with pair: " << queryImageIdx << ", " << trainImageIdx << "\n";

		cv::Matx34d Pleft  = cv::Matx34d::eye();
    	cv::Matx34d Pright = cv::Matx34d::eye();

		std::vector<cv::DMatch> pruned_matches;

		findCameraMatrices(camMatrix, queryImageIdx, trainImageIdx, featureMatchMatrix[queryImageIdx][trainImageIdx], imagesKps[queryImageIdx], imagesKps[trainImageIdx], Pleft, Pright, pruned_matches);

		double inliersRatio = (double)pruned_matches.size() / (double)featureMatchMatrix[queryImageIdx][trainImageIdx].size();
		// std::cout<<"Inliers ratio: "<<inliersRatio<<"Pruned Matches: "<<pruned_matches.size()<<" Good Matches: "<<good_matches.size()<<"\n";
		if (inliersRatio < POSE_INLIERS_MINIMAL_RATIO) {
			std::cout << "Not enough matches after pruning. Should try another pair...\n";
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
	sfmBundleAdjustment::adjustBundle(reconstructionCloud, cameraPoses, camMatrix, imagesPts2D);
	pointcloud_to_ply("../reconstructions/" + reconstructionName + "/base/baseReconstruction");
}

structureFromMotion::image2D3DMatches structureFromMotion::find2d3dmatches() {
	image2D3DMatches matches;
	
	for (size_t viewIdx = 0; viewIdx < images.size(); viewIdx++){
		if (doneViews.find(viewIdx) != doneViews.end()) continue;
        
		image2D3DPair pair2D3D;

		for (Point3D cloudPoint:reconstructionCloud) {
			bool found2DPoint = false;

			for (std::pair<const int, int> origViewAndPoint : cloudPoint.idxImage) {
				int origViewIdx = origViewAndPoint.first;
				int origViewFeatureIdx = origViewAndPoint.second;
			
				int leftViewIdx = (origViewIdx < viewIdx) ? origViewIdx : viewIdx;
                int rightViewIdx = (origViewIdx < viewIdx) ? viewIdx : origViewIdx;

				for (const cv::DMatch &match : featureMatchMatrix[leftViewIdx][rightViewIdx]) {
					int matched2DPointInNewView = -1;

					if (origViewIdx < viewIdx) {

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
						std::vector<cv::Point2d> newViewFeatures = imagesPts2D[viewIdx];
						pair2D3D.pts2D.push_back(newViewFeatures.at(matched2DPointInNewView));
						pair2D3D.pts3D.push_back(cloudPoint.pt);
						found2DPoint = true;
						break;
					}
				}
				if (found2DPoint) break;
			}
		}
		matches[viewIdx] = pair2D3D;
	}

	return matches;
}

bool structureFromMotion::findCameraPosePNP(intrinsics cameraMatrix, std::vector<cv::Point3d> pts3D, std::vector<cv::Point2d> pts2D, cv::Matx34d &P) {
	cv::Mat rvec, T;
	cv::Mat mask;

	if (pts2D.size() < 4 || pts3D.size() < 4) {
        std::cout << "Not enough matches for PnPRansac..." << std::endl;
        return false;
    }

	cv::solvePnPRansac(pts3D, pts2D, cameraMatrix.K, cameraMatrix.distCoef, rvec, T, false, 100, 10.0f, 0.99, mask);

	if (((float)cv::countNonZero(mask) / (float)pts2D.size()) < 0.5) {
        std::cout << "Inliers ratio too small: " << (float)cv::countNonZero(mask) / (float)pts2D.size() << "\n";
        return false;
    }

	cv::Mat R;
  	cv::Rodrigues(rvec, R);

	P = cv::Matx34d(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), T.at<double>(0),
                  	R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), T.at<double>(1),
                  	R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), T.at<double>(2));

	return true;
}

// void structureFromMotion::addPoints(std::vector<Point3D> newPtCloud) {
// 	std::cout<<"Adding new points...\n";

// 	for (Point3D newPt:newPtCloud) {
// 		bool foundMatching3DPoint = false;
// 		for (Point3D& existingPt:reconstructionCloud) {
// 			if (cv::norm(existingPt.pt - newPt.pt) < 0.01) {
// 				foundMatching3DPoint = true;
// 				break;
// 			}
// 		}
// 		if (!foundMatching3DPoint) reconstructionCloud.push_back(newPt);
// 	}
// }

void structureFromMotion::addPoints(std::vector<Point3D> newPtCloud) {
	std::cout<<"Adding new points...\n";

	// const size_t imagesNum = images.size();
    std::vector<std::vector<std::vector<cv::DMatch>>> mergeMatchMatrix;
    mergeMatchMatrix.resize(images.size(), std::vector<std::vector<cv::DMatch>>(images.size()));

    size_t newPoints = 0;
    size_t mergedPoints = 0;

    for (const Point3D &p : newPtCloud) {
        // const cv::Point3f newPoint = p.pt;
		const cv::Point3d newPoint = p.pt;

        bool foundAnyMatchingExistingViews = false;
        bool foundMatching3DPoint = false;
        for (Point3D &existingPoint : reconstructionCloud) {
            if (cv::norm(existingPoint.pt - newPoint) < 0.01) {
                foundMatching3DPoint = true;

                for (const auto& newKv : p.idxImage) {
                    for (const auto& existingKv : existingPoint.idxImage) {

                        bool foundMatchingFeature = false;

						const bool newIsLeft = newKv.first < existingKv.first;
						const int leftViewIdx         = (newIsLeft) ? newKv.first  : existingKv.first;
                        const int leftViewFeatureIdx  = (newIsLeft) ? newKv.second : existingKv.second;
                        const int rightViewIdx        = (newIsLeft) ? existingKv.first  : newKv.first;
                        const int rightViewFeatureIdx = (newIsLeft) ? existingKv.second : newKv.second;

                        const std::vector<cv::DMatch> &matching = featureMatchMatrix[leftViewIdx][rightViewIdx];
                        for (const cv::DMatch &match : matching) {
                            if (match.queryIdx == leftViewFeatureIdx && match.trainIdx == rightViewFeatureIdx && match.distance < 20.0) {
                            	mergeMatchMatrix[leftViewIdx][rightViewIdx].push_back(match);
                                foundMatchingFeature = true;
                                break;
                            }
                        }

                        if (foundMatchingFeature) {
                            existingPoint.idxImage[newKv.first] = newKv.second;
                            foundAnyMatchingExistingViews = true;

                        }
                    }
                }
            }
            if (foundAnyMatchingExistingViews) {
                mergedPoints++;
                break;
            }
        }

        if (!foundAnyMatchingExistingViews && !foundMatching3DPoint) {
            reconstructionCloud.push_back(p);
            newPoints++;
        }
    }
}

void structureFromMotion::addViews() {
	std::cout << "\n--------------------------------------------------\n";
	std::cout << "                 Adding more views...\n";
	std::cout << "--------------------------------------------------\n\n";

	while (doneViews.size() != images.size()) {
		image2D3DMatches matches2D3D = find2d3dmatches();

		size_t bestView;
		size_t bestMatchesNum = 0;
		std::cout << "Matches num: ";
		for (const auto &match : matches2D3D) {
			const size_t matchesNum = match.second.pts2D.size();
			if (matchesNum > bestMatchesNum) {
				bestView = match.first;
				bestMatchesNum = matchesNum;
			}
			std::cout << matchesNum << " ";
		}
		std::cout << "\n";
		
		std::cout << "Next best view to add: " << bestView << " with " << bestMatchesNum << " matches\n";

		doneViews.insert(bestView);

		cv::Matx34d newP = cv::Matx34d::eye();
		bool pose_estimation_succes = findCameraPosePNP(camMatrix, matches2D3D[bestView].pts3D, matches2D3D[bestView].pts2D, newP);
		
		if (!pose_estimation_succes) {
			std::cout << "Pose estimation failed. Skipping view...\n";
			continue;
		}

		cameraPoses[bestView] = newP;

		for (const int goodView : goodViews) {
			size_t queryImageIdx = (goodView < bestView) ? goodView : bestView;
			size_t trainImageIdx = (goodView < bestView) ? bestView : goodView;

			std::vector<cv::DMatch> pruned_matches;
			cv::Matx34d leftP  = cv::Matx34d::eye();
			cv::Matx34d rightP = cv::Matx34d::eye();

			findCameraMatrices(camMatrix, queryImageIdx, trainImageIdx, featureMatchMatrix[queryImageIdx][trainImageIdx], imagesKps[queryImageIdx], imagesKps[trainImageIdx], leftP, rightP, pruned_matches);
			featureMatchMatrix[queryImageIdx][trainImageIdx] = pruned_matches;

			showMatches(queryImageIdx, trainImageIdx, featureMatchMatrix[queryImageIdx][trainImageIdx]);

			std::vector<Point3D> newPointCloud;
			triangulateViews(imagesKps[queryImageIdx], imagesKps[trainImageIdx], featureMatchMatrix[queryImageIdx][trainImageIdx], cameraPoses[queryImageIdx], cameraPoses[trainImageIdx], camMatrix, std::make_pair(queryImageIdx, trainImageIdx), newPointCloud);
			std::cout << "Merging triangulation between " << queryImageIdx
                      << " and " << trainImageIdx << "-> matching points: "
                      << featureMatchMatrix[queryImageIdx][trainImageIdx].size() << "\n";

			addPoints(newPointCloud);

		}			
		sfmBundleAdjustment::adjustBundle(reconstructionCloud, cameraPoses, camMatrix, imagesPts2D);
		goodViews.insert(bestView);
		// pointcloud_to_ply("fountain_" + std::to_string(bestView));
	}
}

void structureFromMotion::run_reconstruction() {
	std::filesystem::create_directories("../reconstructions/" + reconstructionName);
	std::filesystem::create_directories("../reconstructions/" + reconstructionName + "/base");
    std::filesystem::create_directories("../reconstructions/" + reconstructionName + "/sparse");
    std::filesystem::create_directories("../reconstructions/" + reconstructionName + "/dense");
    std::filesystem::create_directories("../reconstructions/" + reconstructionName + "/mesh");
	
	
	setLogging();

	loadImages();
	
	getIntrinsics();

	getFeatures();

	createFeatureMatchMatrix();
	
	baseReconstruction();

	addViews();

}

void structureFromMotion::pointcloud_to_ply(const std::string &filename) {
	std::cout<<"Saving the pointcloud to file: "<<filename<<".ply\n";

	std::ofstream output(filename + ".ply");

	output<<"ply\n"<<
            "format ascii 1.0\n"<<
            "element vertex " << reconstructionCloud.size() <<"\n"<<
            "property float x\n"<<
            "property float y\n"<<
            "property float z\n"<<
            "property uchar red\n"<<
            "property uchar green\n"<<
            "property uchar blue\n"<<
            "end_header\n";

	for (Point3D &point : reconstructionCloud) {
		auto originView = point.idxImage.begin();
		const int viewIdx = originView->first;
		cv::Point2d point2d = imagesPts2D[viewIdx][originView->second];
		cv::Vec3b pointColor = images[viewIdx].at<cv::Vec3b>(point2d);

		output << point.pt.x << " " <<
        	   	  point.pt.y << " " <<
			      point.pt.z << " " <<
			   	  (int)pointColor(2) << " " <<
			   	  (int)pointColor(1) << " " <<
			   	  (int)pointColor(0) << "\n";
	}

	output.close();

}

void structureFromMotion::PMVS2(){

  /*FOLDERS FOR PMVS2*/
  std::cout << "Creating folders for PMVS2..." << std::endl;
  int dont_care;
  dont_care = std::system("mkdir -p denseCloud/visualize");
  dont_care = std::system("mkdir -p denseCloud/txt");
  dont_care = std::system("mkdir -p denseCloud/models");
  std::cout << "Created: \nfolder:visualize" << "\n" << "folder:txt" << "\n" << "folder:models\n";

  /*OPTIONS CONFIGURATION FILE FOR PMVS2*/
  std::cout << "Creating options file for PMVS2...\n";
  std::ofstream option("denseCloud/options.txt");
  option << "minImageNum 5\n";
  option << "CPU 4\n";
  option << "timages  -1 " << 0 << " " << (images.size()-1) << "\n";
  option << "oimages 0\n";
  option << "level 1\n";
  option.close();
  std::cout << "Created: options.txt\n";

  /*CAMERA POSES AND IMAGES INPUT FOR PMVS2*/
  std::cout << "Saving camera poses for PMVS2...\n";
  std::cout << "Saving camera images for PMVS2...\n";
  for(int i=0; i<cameraPoses.size(); i++) {

      char str[256];
      boost::filesystem::directory_entry x(imagesPaths[i]);
      std::string extension = x.path().extension().string();
      boost::algorithm::to_lower(extension);

      std::sprintf(str, "cp -f %s denseCloud/visualize/%04d.jpg", imagesPaths[i].c_str(), (int)i);
      dont_care = std::system(str);
      cv::imwrite(str, images[i]);

      std::sprintf(str, "denseCloud/txt/%04d.txt", (int)i);
      std::ofstream ofs(str);
      cv::Matx34d pose = cameraPoses[i];

      //K*P
      pose = (cv::Matx33d)camMatrix.K*pose;

      ofs << "CONTOUR" << std::endl;
      ofs << pose(0,0) << " " << pose(0,1) << " " << pose(0,2) << " " << pose(0,3) << "\n"
          << pose(1,0) << " " << pose(1,1) << " " << pose(1,2) << " " << pose(1,3) << "\n"
          << pose(2,0) << " " << pose(2,1) << " " << pose(2,2) << " " << pose(2,3) << std::endl;

      ofs << std::endl;
      ofs.close();
  } 
  std::cout << "Camera poses saved." << "\n" << "Camera images saved." << std::endl; 
}

void structureFromMotion::export_to_json(std:: string filename, cv::Mat matrix) {
    cv::FileStorage fs("/home/csimage/GitRepos/3rdYear/vr3dmodels/" + filename + ".json", cv::FileStorage::WRITE);
    fs<<"Matrix"<<matrix;
    fs.release();
}

void structureFromMotion::setLogging() {
	cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_SILENT);
}
