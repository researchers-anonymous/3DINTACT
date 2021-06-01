#include "svd.h"

SVD::SVD(const std::vector<Point>& points, const int& flag)
    : m_points(points)
{
    // compute centroid
    m_centroid = Point::centroid(m_points);

    // cast points to eigen matrix
    const int R = points.size();
    const int C = 3;
    m_vectors = Eigen::MatrixXf(R, C);
    int row = 0;
    for (auto point : points) {
        m_vectors(row, C0) = (float)point.m_xyz[0];
        m_vectors(row, C1) = (float)point.m_xyz[1];
        m_vectors(row, C2) = (float)point.m_xyz[2];
        row++;
    }

    // translate points to directional vectors from centroid
    m_vectors.col(C0).array() -= (float)m_centroid.m_xyz[0];
    m_vectors.col(C1).array() -= (float)m_centroid.m_xyz[1];
    m_vectors.col(C2).array() -= (float)m_centroid.m_xyz[2];

    // compute singular value decomposition
    m_usv = m_vectors.jacobiSvd(flag);
}

Eigen::Vector3d SVD::getV3Normal()
{
    Eigen::MatrixXf normal = m_usv.matrixV().col(2);
    Eigen::Vector3d m_v3Norm
        = Eigen::Vector3d(normal(0, C0), normal(1, C0), normal(2, C0));
    return m_v3Norm;
}

std::vector<Eigen::Vector3d> SVD::getUNormals()
{
    // get normals computed for each of the X, Y, and Z vectors
    Eigen::MatrixXf X = m_usv.matrixU().col(0);
    Eigen::MatrixXf Y = m_usv.matrixU().col(1);
    Eigen::MatrixXf Z = m_usv.matrixU().col(2);

    // assert # of computed U normals tallies with # of points
    assert(m_points.size() == m_usv.matrixU().rows());

    // collect U normal vectors for each point
    std::vector<Eigen::Vector3d> normals(m_points.size());
    for (int i = 0; i < m_points.size(); i++) {
        Eigen::Vector3d normal(X(i, C0), Y(i, C1), Z(i, C2));
        normals[i] = normal;
    }
    return normals;
}
