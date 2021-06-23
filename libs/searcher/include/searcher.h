#ifndef SEARCHER_H
#define SEARCHER_H

#include "point.h"
#include <vector>

namespace searcher {

/** pointFound
 *    Computes the K nearest neighbours of a queryPoint.
 *
 *  @param points
 *    Set of 3D points (x, y, z).
 *
 *  @param queryPoint
 *    query point
 *
 *  @retval
 *    true: if point found
 *    false: if point not found
 * */
bool pointFound(std::vector<Point>& points, const Point& queryPoint);
}
#endif /* SEARCHER_H */
