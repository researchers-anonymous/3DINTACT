#ifndef PROPOSAL_H
#define PROPOSAL_H

#include <Eigen/Dense>
#include <vector>

#include "point.h"
#include "svd.h"

namespace proposal {

/** grow
 *    Grows proposal of tabletop interaction context.
 *
 * @param computation
 *    Computed SVD (singular value decomposition).
 *
 * @retval
 *    Coarse segment/proposal region of tabletop interaction context.
 */
std::vector<Point> grow(SVD& svd, std::vector<Point>& points);
}

#endif /* PROPOSAL_H */
