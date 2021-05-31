#include <cmath>

#include "edge.h"
#include "io.h"
#include "proposal.h"

#define V3_ARGMIN 350
#define U_ARGMIN 3
const int C0 = 0; // column 0
const int C1 = 1; // column 1
const int C2 = 2; // column 2

std::vector<Point> proposal::grow(SVD& svd, std::vector<Point>& points)
{
    // get unitary matrix normals
    Eigen::MatrixXf x = svd.m_usv.matrixU().col(0);
    Eigen::MatrixXf y = svd.m_usv.matrixU().col(1);
    Eigen::MatrixXf z = svd.m_usv.matrixU().col(2);

    // determine region of interest
    std::vector<Point> roi(points.size());
    Eigen::Vector3d vec;
    Eigen::Vector3d svdUNormal;
    Eigen::Vector3d svdV3Normal = svd.getV3Normal();

    int index = 0;
    for (int i = 0; i < points.size(); i++) {
        vec = Eigen::Vector3d(
            svd.m_vectors(i, C0), svd.m_vectors(i, C1), svd.m_vectors(i, C2));

        if (std::abs(svdV3Normal.dot(vec)) < V3_ARGMIN) {

            svdUNormal = Eigen::Vector3d(x(i, C0), y(i, C0), z(i, C0));

            if (svdUNormal.dot(vec) < U_ARGMIN) {
                if (edge::detect(z(i, C0))) {
                    roi[index] = points[i];
                    index++;
                }
            }
        }
    }
    std::vector<Point> optimizedRoi(roi.begin(), roi.begin() + index);

    // clean coarse segment
    const int16_t CLEAN = 100;
    Point centroid = Point::centroid(optimizedRoi);

    std::vector<Point> segment(optimizedRoi.size());

    index = 0;
    for (int i = 0; i < optimizedRoi.size(); i++) { /*NOLINT*/
        if (optimizedRoi[i].m_xyz[2] < centroid.m_xyz[2] + CLEAN) {
            segment[index] = optimizedRoi[i];
            index++;
        }
    }
    std::vector<Point> optimizedSegment(
        segment.begin(), segment.begin() + index);

    return optimizedSegment;
}
