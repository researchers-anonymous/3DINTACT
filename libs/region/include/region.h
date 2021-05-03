#ifndef REGION_H
#define REGION_H

#include <vector>

#include "point.h"

namespace region {

/**
 * segment
 *   Segments context.
 *
 * @param points
 *   Points to segment.
 *
 * @retval
 *   Segmented points.
 */
std::vector<Point> segment(std::vector<Point>& points);

/**
 * queryBoundary
 *   Queries the min max point boundaries of segmented context.
 *
 * @param points
 *   Segmented interaction context.
 *
 * @retval
 *    { Point_min, Point_max }
 */
std::pair<Point, Point> queryBoundary(std::vector<Point>& points);
}
#endif /* REGION_H */
