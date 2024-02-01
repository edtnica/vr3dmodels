#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "cameraCalibration.h"

void cameraCalibration::export_to_xml(std:: string filename, cv::Mat K, cv::Mat distCoeff) {
    cv::FileStorage fs("C:/Programs and Stuff/vr3dmodels/calibration/" + filename + ".xml", cv::FileStorage::WRITE);
    fs<<"Camera_Matrix"<<K;
    fs<<"Distortion_Coefficients"<<distCoeff;
    fs.release();
}

void cameraCalibration::calibCamera(cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Mat &R, cv::Mat &T) {
    int CHECKERBOARD[2]{6,8};
    // int fieldSize = 25;

    std::vector<std::vector<cv::Point3f>> objpoints;
    std::vector<std::vector<cv::Point2f>> imgpoints;

    std::vector<cv::Point3f> objp;

    // Defining the world coordinates for 3D points
    for (int i=0; i<CHECKERBOARD[1]; i++) {
        for (int j=0; j<CHECKERBOARD[0]; j++) {
            objp.push_back(cv::Point3f(j,i,0)); // maybe j*fieldsize and i*fieldsize
        }
    }

    std::vector<cv::String> images;
    
    cv::glob("C:/Programs and Stuff/vr3dmodels/calibration/checkerboardImgs4k/*.jpg", images);

    cv::Mat frame, gray;
    std::vector<cv::Point2f> corner_points;
    bool patternFound;

    // Looping over all the images in the directory
    for (auto const &image : images) {
        frame = cv::imread(image);
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        patternFound = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_points, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (patternFound) {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
            cv::cornerSubPix(gray, corner_points, cv::Size(11,11), cv::Size(-1,-1), criteria);
            objpoints.push_back(objp);
            imgpoints.push_back(corner_points);
        }

        // Display
        cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_points, patternFound);
        cv::imshow("Checkerboard", frame);
        cv::waitKey(0);
    }

    cv::destroyAllWindows();

    cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cameraMatrix, distCoeffs, R, T);

    export_to_xml("cameraMatrix2", cameraMatrix, distCoeffs);
}