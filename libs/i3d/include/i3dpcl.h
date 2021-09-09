#ifndef PLY_H
#define PLY_H

#include "point.h"
#include <k4a/k4a.h>

/**
 * io::write
 *   Output point cloud as *.write file
 *
 * @param raw
 *   Unorganized 3D point cloud.
 *
 * @param context
 *   Interaction context 3D point cloud.
 */

namespace i3dpcl {
void write(const Point& lowerBound, const Point& upperBound,
    const k4a_image_t& pclImage, const k4a_image_t& rgbImage,
    const std::string& file);

void write(std::vector<Point>& pCloud, std::vector<Point>& pCloudSeg);

void write1(std::vector<Point>& points);

void write(k4a_image_t const& pclImage, k4a_image_t const& rgbImage,
    const std::string& FILE);

void write(std::vector<Point>& points, const std::string& FILE);

std::vector<Point> build(
    const int& w, const int& h, const int16_t* pCloudData, const uint8_t* bgra);

void write(const int& w, const int& h, const int16_t* pCloudData,
    const uint8_t* rgbData, const std::string& FILE);
}
#endif /* PLY_H */
