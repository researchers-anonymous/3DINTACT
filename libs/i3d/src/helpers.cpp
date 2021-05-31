#include <cmath>
#include <k4a/k4a.hpp>
#include <opencv2/core.hpp>
#include <torch/script.h>

#include "helpers.h"
#include "macros.hpp"
#include "point.h"

void utils::configTorch(
    std::vector<std::string>& classNames, torch::jit::script::Module& module)
{
    const std::string scriptName = io::pwd() + "/resources/torchscript.pt";
    const std::string cocoNames = io::pwd() + "/resources/coco.names";
    module = torch::jit::load(scriptName);
    std::ifstream f(cocoNames);
    std::string name;
    while (std::getline(f, name)) {
        classNames.push_back(name);
    }
}

void utils::adapt(const int& index, Point& point,
    const std::vector<int16_t>& pCloudFrame,
    const std::vector<uint8_t>& imgFrame)
{
    int16_t x = pCloudFrame[3 * index + 0];
    int16_t y = pCloudFrame[3 * index + 1];
    int16_t z = pCloudFrame[3 * index + 2];

    uint8_t b = imgFrame[4 * index + 0];
    uint8_t g = imgFrame[4 * index + 1];
    uint8_t r = imgFrame[4 * index + 2];
    uint8_t a = imgFrame[4 * index + 3];

    int16_t xyz[3] = { x, y, z };
    uint8_t rgba[4] = { r, g, b, a };
    uint8_t bgra[4] = { b, g, r, a };

    point.m_id = index;
    point.setPoint(xyz);
    point.setPixel_GL(rgba);
    point.setPixel_CV(bgra);
}

bool utils::null(const int& index, std::vector<int16_t>& pCloudFrame,
    std::vector<uint8_t>& imgFrame)
{
    if (pCloudFrame[3 * index + 2] == 0 && imgFrame[4 * index + 3] == 0) {
        return true;
    }
    return false;
}

bool utils::invalid(const int& index, const k4a_float2_t* ptr_xyTable,
    const uint16_t* ptr_depth)
{
    if (ptr_depth[index] == 0 && std::isnan(ptr_xyTable[index].xy.x)
        && std::isnan(ptr_xyTable[index].xy.y)) {
        return true;
    }

    int16_t x = ptr_xyTable[index].xy.x * (float)ptr_depth[index];
    int16_t y = ptr_xyTable[index].xy.y * (float)ptr_depth[index];
    int16_t z = (float)ptr_depth[index];
    if (x == 0 && y == 0 && z == 0) {
        return true;
    }

    return false;
}

bool utils::invalid(
    const int& index, const int16_t* ptr_pCloudData, const uint8_t* ptr_imgData)
{
    if (ptr_pCloudData[3 * index + 2] == 0
        || std::isnan(ptr_pCloudData[3 * index + 2])) {
        return true;
    }
    if (std::isnan(ptr_imgData[index])) {
        return true;
    }
    return false;
}

void utils::addXYZ(const int& index, std::vector<int16_t>& pCloudFrame,
    const int16_t* ptr_pCloudData)
{
    pCloudFrame[3 * index + 0] = ptr_pCloudData[3 * index + 0];
    pCloudFrame[3 * index + 1] = ptr_pCloudData[3 * index + 1];
    pCloudFrame[3 * index + 2] = ptr_pCloudData[3 * index + 2];
}

void utils::addXYZ(const int& index, std::vector<int16_t>& pCloudFrame)
{
    pCloudFrame[3 * index + 0] = 0;
    pCloudFrame[3 * index + 1] = 0;
    pCloudFrame[3 * index + 2] = 0;
}

void utils::addPixel_CV(const int& index, std::vector<uint8_t>& imgFrame_CV,
    const uint8_t* ptr_imgData)
{
    imgFrame_CV[4 * index + 0] = ptr_imgData[4 * index + 0]; // blue
    imgFrame_CV[4 * index + 1] = ptr_imgData[4 * index + 1]; // green
    imgFrame_CV[4 * index + 2] = ptr_imgData[4 * index + 2]; // red
    imgFrame_CV[4 * index + 3] = ptr_imgData[4 * index + 3]; // alpha
}

void utils::addPixel_GL(const int& index, std::vector<uint8_t>& imgFrame_GL,
    const uint8_t* ptr_imgData)
{
    imgFrame_GL[4 * index + 2] = ptr_imgData[4 * index + 0]; // blue
    imgFrame_GL[4 * index + 1] = ptr_imgData[4 * index + 1]; // green
    imgFrame_GL[4 * index + 0] = ptr_imgData[4 * index + 2]; // red
    imgFrame_GL[4 * index + 3] = ptr_imgData[4 * index + 3]; // alpha
}

void utils::addPixel_GL(const int& index, std::vector<uint8_t>& imgFrame_GL)
{
    imgFrame_GL[4 * index + 0] = 0; // red
    imgFrame_GL[4 * index + 1] = 0; // green
    imgFrame_GL[4 * index + 2] = 0; // blue
    imgFrame_GL[4 * index + 3] = 0; // alpha
}

void utils::addPixel_CV(const int& index, std::vector<uint8_t>& imgFrame_CV)
{
    imgFrame_CV[4 * index + 0] = 0; // red
    imgFrame_CV[4 * index + 1] = 0; // green
    imgFrame_CV[4 * index + 2] = 0; // blue
    imgFrame_CV[4 * index + 3] = 0; // alpha
}

bool utils::inSegment(const int& index, const std::vector<int16_t>& pCloudFrame,
    const Point& minPoint, const Point& maxPoint)
{
    if (pCloudFrame[3 * index + 2] == 0) {
        return false;
    }

    if ((int16_t)pCloudFrame[3 * index + 0] > maxPoint.m_xyz[0]
        || (int16_t)pCloudFrame[3 * index + 0] < minPoint.m_xyz[0]
        || (int16_t)pCloudFrame[3 * index + 1] > maxPoint.m_xyz[1]
        || (int16_t)pCloudFrame[3 * index + 1] < minPoint.m_xyz[1]
        || (int16_t)pCloudFrame[3 * index + 2] > maxPoint.m_xyz[2]
        || (int16_t)pCloudFrame[3 * index + 2] < minPoint.m_xyz[2]) {
        return false;
    }
    return true;
}

void utils::stitch(const int& index, Point& point, int16_t* ptr_pcl,
    uint8_t* ptr_img_GL, uint8_t* ptr_img_CV)
{
    ptr_pcl[3 * index + 0] = point.m_xyz[0]; // x
    ptr_pcl[3 * index + 1] = point.m_xyz[1]; // y
    ptr_pcl[3 * index + 2] = point.m_xyz[2]; // z

    ptr_img_CV[4 * index + 0] = point.m_bgra[0]; // blue
    ptr_img_CV[4 * index + 1] = point.m_bgra[1]; // green
    ptr_img_CV[4 * index + 2] = point.m_bgra[2]; // red
    ptr_img_CV[4 * index + 3] = point.m_bgra[3]; // alpha

    ptr_img_GL[4 * index + 0] = point.m_rgba[0]; // red
    ptr_img_GL[4 * index + 1] = point.m_rgba[1]; // green
    ptr_img_GL[4 * index + 2] = point.m_rgba[2]; // blue
    ptr_img_GL[4 * index + 3] = point.m_rgba[3]; // alpha
}

void utils::stitch(const int& index, Point& point, uint8_t* ptr_img_CV)
{
    ptr_img_CV[4 * index + 0] = point.m_bgra[0]; // blue
    ptr_img_CV[4 * index + 1] = point.m_bgra[1]; // green
    ptr_img_CV[4 * index + 2] = point.m_bgra[2]; // red
    ptr_img_CV[4 * index + 3] = point.m_bgra[3]; // alpha
}

std::pair<Point, Point> utils::queryBoundary(std::vector<Point>& points)
{
    std::vector<int16_t> X(points.size());
    std::vector<int16_t> Y(points.size());
    std::vector<int16_t> Z(points.size());
    for (int i = 0; i < points.size(); i++) {
        X[i] = points[i].m_xyz[0];
        Y[i] = points[i].m_xyz[1];
        Z[i] = points[i].m_xyz[2];
    }

    int16_t xMax = (int16_t)*std::max_element(X.begin(), X.end());
    int16_t xMin = (int16_t)*std::min_element(X.begin(), X.end());
    int16_t yMax = (int16_t)*std::max_element(Y.begin(), Y.end());
    int16_t yMin = (int16_t)*std::min_element(Y.begin(), Y.end());
    int16_t zMax = (int16_t)*std::max_element(Z.begin(), Z.end());
    int16_t zMin = (int16_t)*std::min_element(Z.begin(), Z.end());

    Point min(xMin, yMin, zMin);
    Point max(xMax, yMax, zMax);
    return { min, max };
}

// todo: move to viewer
void utils::cvDisplay(
    cv::Mat img, std::shared_ptr<I3d>& sptr_i3d, clock_t start)
{
    cv::putText(img,
        "FPS: " + std::to_string(int(1e7 / (double)(clock() - start))),
        cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0),
        2);

    cv::imshow("", img);
    if (cv::waitKey(1) == 27) {
        STOP
    }
}
