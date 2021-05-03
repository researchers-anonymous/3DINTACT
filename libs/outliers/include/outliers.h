#ifndef LDA_H
#define LDA_H

#include "point.h"
#include <vector>

namespace outliers {
/**
 * remove
 *   Finds and removes outliers from a vector of points.
 *
 * @param points
 *   Vector of 3D points.
 *
 * @retval
 *    Vector of 3D points without outliers.
 */
std::vector<Point> filter(std::vector<Point>& points);

/**
 * remove
 *   Finds and removes outliers from a vector of floats.
 *
 * @param
 *   Vector of float values.
 *
 * @retval
 *    Vector of floats without outliers.
 */
std::vector<float> remove(std::vector<float>& points);
}
#endif /* LDA_H  */
