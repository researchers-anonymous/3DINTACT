#ifndef PROPOSAL_H
#define PROPOSAL_H

#include "point.h"
#include <Eigen/Dense>
#include <vector>

namespace proposal {

/**
 * grow
 *   Grows proposal of tabletop interaction context.
 *
 * @param computation
 *   Computed SVD (singular value decomposition).
 *
 * @retval
 *    Coarse segment/proposal region of tabletop interaction context.
 */
std::vector<Point> grow(
    const std::pair<Eigen::JacobiSVD<Eigen::MatrixXf>, Eigen::MatrixXf>&
        computation,
    std::vector<Point>& points);
}

#endif /* PROPOSAL_H */
