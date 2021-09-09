#ifndef INTACT_UTILS_H
#define INTACT_UTILS_H

#include <opencv2/core/mat.hpp>
#include <string>
#include <thread>

#include "i3d.h"
#include "io.h"

namespace i3dutils {

void adapt(const int& index, Point& point, const std::vector<int16_t>& pCloud,
    const std::vector<uint8_t>& image);

bool invalid(const int& index, const k4a_float2_t* ptr_xyTable,
    const uint16_t* depthData);

bool invalid(const int& index, const int16_t* xyzData, const uint8_t* rgbaData);

bool null(const int& index, std::vector<int16_t>& pCloud,
    std::vector<uint8_t>& image);

void addXYZ(const int& index, std::vector<int16_t>& pCloud);

void addXYZ(
    const int& index, std::vector<int16_t>& pCloud, const int16_t* xyzData);

void addRGBA(const int& index, std::vector<uint8_t>& rgbaImage);

void addRGBA(const int& index, std::vector<uint8_t>& rgbaImage,
    const uint8_t* ptr_imgData);

void addBGRA(const int& index, std::vector<uint8_t>& bgraImage,
    const uint8_t* ptr_imgData);

void addBGRA(const int& index, std::vector<uint8_t>& bgraImage);

bool inSegment(const int& index, const std::vector<int16_t>& pCloud,
    const Point& minPoint, const Point& maxPoint);

void stitch(const int& index, Point& point, int16_t* xyzData, uint8_t* rgbaData,
    uint8_t* bgraData);

std::pair<Point, Point> queryBoundary(std::vector<Point>& points);

void stitch(const int& index, Point& point, uint8_t* bgra);

// void show(cv::Mat img, std::shared_ptr<i3d>& sptr_i3d, clock_t start);

// void show(uint8_t* bgraData, std::shared_ptr<i3d>& sptr_i3d, clock_t start);

void add(std::vector<uint8_t*>& colors);

void stitch(const int& index, Point& point, std::vector<int16_t>& pCloud,
    std::vector<uint8_t> rgba);

int randNum(const int& max);

void show(const int& h, const int& w, uint8_t* bgraData,
    std::shared_ptr<i3d>& sptr_i3d);
}
#endif /*INTACT_UTILS_H*/
