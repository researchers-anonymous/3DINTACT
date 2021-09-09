#ifndef SURFACE_H
#define SURFACE_H

#include <opencv2/opencv.hpp>
#include <string>

#include "kinect.h"
#include "i3d.h"

namespace surface {

cv::Mat getSurfaceCapture(std::shared_ptr<i3d>& sptr_i3d);

void project(const std::string& window, const int& w, const int& h,
    cv::Mat& img, const cv::Mat& R, const cv::Mat& t);

cv::Mat color(const bool& contrast, const int& w, const int& h);

void contrast(std::shared_ptr<i3d>& sptr_i3d,
              const std::string& window, const int& w, const int& h,
              std::vector<cv::Mat>& scene);

void undistort(cv::Mat& frame);

void saturate(const cv::Mat& src, cv::Mat& dst);

cv::Rect reg(const cv::Mat& src1, const cv::Mat& src2);

}
#endif // SURFACE_H
