#ifndef KNN_H
#define KNN_H

#include <string>
#include <vector>

#include "point.h"

namespace knn {

/** compute
 *    Computes the K nearest neighbours of a queryPoint.
 *
 *  @param points
 *    Set of 3D points (x, y, z).
 *
 *  @param k
 *    K value.
 *
 *  @param queryPoint
 *    query point
 *
 *  @retval
 *    The indexes of the nearest neighbors from closest
 *    neighbor to furthest neighbour
 * */
std::vector<int> compute(
    std::vector<Point>& points, const Point& queryPoint, const int& k);

/** compute
 *    Computes the K nearest neighbours of a set of queryPoints.
 *
 *  @param points
 *    Set of 3D points (x, y, z).
 *
 *  @param k
 *    K value.
 *
 *  @param queryPoints
 *    Set of 3D query points (x, y, z).
 *
 *  @retval
 *    The squared distance to the Kth nearest neighbor
 *    for each query point.
 * */
std::vector<float> compute(std::vector<Point>& points,
    const std::vector<Point>& queryPoints, const int& k);
}
#endif /* KNN_H */
