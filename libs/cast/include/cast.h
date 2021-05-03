#ifndef CAST_H
#define CAST_H

#include <string>
#include <vector>

#include "kinect.h"
#include "point.h"

// A raw point cloud is defined by two components:
//  (1) std::vector<float>    | geometry
//  (2) std::vector<uint8_t>  | color
// For processing, we use the Point type definition (@see point.h)
// this libs help cast between the data definitions
//
namespace cast {

/**
 * toPoint
 *   Casts point cloud to std::vector<Point>.
 *
 * @param pcl
 *   Point cloud.
 *
 * @param color
 *   Point cloud color.
 *
 * @param numPoints
 *   Number of point cloud points.
 *
 * @retval
 *    Points.
 */
std::vector<Point> toPoint(const std::vector<float>& pcl,
    const std::vector<uint8_t>& color, const int& numPoints);

/**
 * toPcl
 *   Casts std::vector<Point> to point cloud.
 *
 * @param sptr_kinect
 *   Kinect device.
 *
 * @retval
 *    Point cloud and point-cloud color.
 */
std::pair<std::vector<float>, std::vector<uint8_t>> toPcl(
    std::vector<Point>& points);

std::pair<std::vector<float>, std::vector<uint8_t>> toClusteredPcl(
    std::vector<Point>& points);
}
#endif /* CAST_H */
