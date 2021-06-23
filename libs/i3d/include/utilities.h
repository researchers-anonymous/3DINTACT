#ifndef INTACT_UTILS_H
#define INTACT_UTILS_H

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <torch/script.h>

#include "i3d.h"
#include "io.h"

namespace utils {

void configTorch(
    std::vector<std::string>& classNames, torch::jit::script::Module& module);

void adapt(const int& index, Point& point,
    const std::vector<int16_t>& pCloudFrame,
    const std::vector<uint8_t>& imgFrame);

bool invalid(const int& index, const k4a_float2_t* ptr_xyTable,
    const uint16_t* ptr_depth);

bool invalid(const int& index, const int16_t* ptr_pCloudData,
    const uint8_t* ptr_imgData);

bool null(const int& index, std::vector<int16_t>& pCloudFrame,
    std::vector<uint8_t>& imgFrame);

void addXYZ(const int& index, std::vector<int16_t>& pCloudFrame);

void addXYZ(const int& index, std::vector<int16_t>& pCloudFrame,
    const int16_t* ptr_pCloudData);

void addPixel_RGBA(const int& index, std::vector<uint8_t>& imgFrame_GL);

void addPixel_RGBA(const int& index, std::vector<uint8_t>& imgFrame_GL,
    const uint8_t* ptr_imgData);

void addPixel_BGRA(const int& index, std::vector<uint8_t>& imgFrame_CV,
    const uint8_t* ptr_imgData);

void addPixel_BGRA(const int& index, std::vector<uint8_t>& imgFrame_CV);

bool inSegment(const int& index, const std::vector<int16_t>& pCloudFrame,
    const Point& minPoint, const Point& maxPoint);

void stitch(const int& index, Point& point, int16_t* ptr_pCloud,
    uint8_t* ptr_img_GL, uint8_t* ptr_img_CV);

std::pair<Point, Point> queryBoundary(std::vector<Point>& points);

void stitch(const int& index, Point& point, uint8_t* ptr_img_CV);

void cvDisplay(cv::Mat img, std::shared_ptr<I3d>& sptr_i3d, clock_t start);

void add(std::vector<uint8_t*>& colors);

void stitch(const int& index, Point& point, std::vector<int16_t>& pCloud,
    std::vector<uint8_t> image_GL);

int randNum(const int& max);
}
#endif /*INTACT_UTILS_H*/
