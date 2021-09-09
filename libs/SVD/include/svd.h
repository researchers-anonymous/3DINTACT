#ifndef SVD_H
#define SVD_H

#include <Eigen/Dense>
#include <vector>

#include "point.h"

class SVD {

private:
    const int C0 = 0; // column 0
    const int C1 = 1; // column 1
    const int C2 = 2; // column 2
    std::vector<Point> m_points;

public:
    Point m_centroid;                        // centroid of given set of points
    Eigen::MatrixXf m_vectors;               // points translated from centroid
    Eigen::JacobiSVD<Eigen::MatrixXf> m_usv; // computed USV solution

    /** svd
     *    Constructs and a singular value decomposition
     *    USV solution, i.e., the
     *    1. [ U ] the unitary matrix,
     *    2. [ S ] rectangular diagonal matrix, and
     *    3. [ V ] the complex unitary matrix.
     *
     * @param points
     *   Give set of points.
     */
    SVD(const std::vector<Point>& points, const int& flag);

    /** getV3Normal
     *
     * @retval
     *   Normal vector from the complex unitary matrix.
     */
    Eigen::Vector3d getV3Normal();

    /** getUNormal
     *
     * @retval
     *   Normal vectors from the unitary matrix.
     */
    std::vector<Eigen::Vector3d> getUNormals();
};
#endif /* SVD_H */
