#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <opencv2/core.hpp>

#include "kinect.h"

namespace calibration {

const float calibrationSquareDimension = 0.02500f; // in meters
const float arucoSquareDimension = 0.0565f;        // in meters

void createChessboardBoarder(const cv::Size& boardSize, float squareEdgeLength,
    std::vector<cv::Point3f>& corners);

void findChessboardCorners(std::vector<cv::Mat>& images,
    std::vector<std::vector<cv::Point2f>>& allFoundCorners,
    bool showResults = false);

void calibrate(std::vector<cv::Mat> calibrationImages,
    const cv::Size& boardSize, float squareEdgeLength, cv::Mat& cameraMatrix,
    cv::Mat& distanceCoefficients);

bool exportCalibration(const std::string& name, cv::Mat cameraMatrix,
    cv::Mat distanceCoefficients);

bool importCalibration(const std::string& name, cv::Mat& cameraMatrix,
    cv::Mat& distanceCoefficients);

void createArucoMarkers();

int findArucoMarkers(
    const cv::Mat& cameraMatrix, const cv::Mat& distanceCoefficients);

void startChessBoardCalibration(
    cv::Mat& cameraMatrix, cv::Mat distanceCoefficients);

void kinect2CV(std::shared_ptr<Kinect>& sptr_kinect);
}
#endif /* CALIBRATION_H */
