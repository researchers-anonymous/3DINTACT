#ifndef REGION_H
#define REGION_H

#include "point.h"
#include <vector>

namespace region {

/** segment
 *   Segments a 3D point cloud and extracts an
 *   orthogonal planar surface.
 *
 * @param points
 *   The given set of 3D points
 *
 * @retval
 *   Set of 3D points corresponding to the
 *   orthogonal planar surface
 */
std::vector<Point> segment(std::vector<Point>& points);

}
#endif /* REGION_H */
