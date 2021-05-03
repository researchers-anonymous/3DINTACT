#ifndef KNN_H
#define KNN_H

#include <string>
#include <vector>

#include "point.h"

namespace knn {

/** compute
 *    Wraps nanoflann's knn implementation into a tiny interface
 *
 *  @param points
 *    Set of 3D points (x, y, z).
 *
 *  @param k
 *    Value of K, i.e., number of neighbours
 *
 *  @param indexOfQueryPoint
 *    Index of the query point, i.e., index corresponding to first
 *    parameter
 *
 *  @retval
 *     A list of point-distance pairs corresponding the nearest
 *     neighbours. The points and respective distances from a
 *     query point are sorted in ascending order.
 * */

std::vector<std::pair<Point, float>> compute(
    std::vector<Point>& points, const int& k, const int& indexOfQueryPoint);
}
#endif /* KNN_H */
