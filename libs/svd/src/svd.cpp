#include "svd.h"

std::pair<Eigen::JacobiSVD<Eigen::MatrixXf>, Eigen::MatrixXf> svd::compute(
    std::vector<Point>& points)
{
    /** compute centroid */
    Point centroid = Point::centroid(points);

    /** represent points as matrix */
    Eigen::MatrixXf vectors(points.size(), R);
    int row = 0;
    for (auto point : points) {
        vectors(row, xCol) = point.m_xyz[0];
        vectors(row, yCol) = point.m_xyz[1];
        vectors(row, zCol) = point.m_xyz[2];
        row++;
    }
    /**   translate points to direction vectors */
    vectors.col(xCol).array() -= centroid.m_xyz[0];
    vectors.col(yCol).array() -= centroid.m_xyz[1];
    vectors.col(zCol).array() -= centroid.m_xyz[2];

    /** compute svd */
    int setting = Eigen::ComputeThinU | Eigen::ComputeThinV;
    Eigen::JacobiSVD<Eigen::MatrixXf> svd = vectors.jacobiSvd(setting);

    /** return computed svd and translated points matrix */
    return { svd, vectors };
}
