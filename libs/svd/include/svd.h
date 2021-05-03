#ifndef SVD_H
#define SVD_H

#include <Eigen/Dense>
#include <utility>
#include <vector>

#include "point.h"

namespace svd {
std::pair<Eigen::JacobiSVD<Eigen::MatrixXf>, Eigen::MatrixXf> compute(
    std::vector<Point>& points);
};
#endif /* SVD_H */
