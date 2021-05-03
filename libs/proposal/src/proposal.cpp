#include <algorithm>
#include <cmath>

#include "edge.h"
#include "io.h"
#include "proposal.h"

#define HEIGHT 350 // <- height of context in mm
#define ARGMIN 3   // <- argmin for growing region

const int SINGLE_COLUMN = 0;

std::vector<Point> proposal::grow(
    const std::pair<Eigen::JacobiSVD<Eigen::MatrixXf>, Eigen::MatrixXf>&
        computation,
    std::vector<Point>& points)
{
    /** growing region */
    std::vector<Point> region;

    Eigen::JacobiSVD<Eigen::MatrixXf> svd = computation.first;
    Eigen::MatrixXf vectors = computation.second;

    /** get normal */
    Eigen::MatrixXf x = svd.matrixU().col(0);
    Eigen::MatrixXf y = svd.matrixU().col(1);
    Eigen::MatrixXf z = svd.matrixU().col(2);
    Eigen::MatrixXf v3 = svd.matrixV().col(2);
    Eigen::Vector3d v3Norm(
        v3(0, SINGLE_COLUMN), v3(1, SINGLE_COLUMN), v3(2, SINGLE_COLUMN));

    /** grow region */
    std::vector<float> norms;

    std::vector<float> depthMeasures;
    int index = 0;
    for (const auto& point : points) {
        Eigen::Vector3d vec(
            vectors(index, xCol), vectors(index, yCol), vectors(index, zCol));
        Eigen::Vector3d norm(x(index, SINGLE_COLUMN), y(index, SINGLE_COLUMN),
            z(index, SINGLE_COLUMN));

        float uScalar = norm.dot(vec);
        float v3Scalar = v3Norm.dot(vec);

        /** stretch to -H and  account for surface
         *   unevenness and imperfect/uneven kinect mounting */
        if (std::abs(v3Scalar) < HEIGHT) {

            /** In practice, norm dot vec != 0.
             *  grow region of interest using argmin */
            if (uScalar < ARGMIN) {

                /** detect edges using z-component norm */
                if (edge::detect(z(index, SINGLE_COLUMN))) {
                    depthMeasures.push_back(point.m_xyz[2]);
                    region.push_back(point);
                }
            }
        }
        index++;
    }
    // todo: automate finding depth measure clip-off
    // std::sort(depthMeasures.begin(), depthMeasures.end(), std::greater<>());
    // const std::string file = io::pwd() + "/output/norms.csv";
    // io::write(depthMeasures, file);
    const float CLEAN = 100;
    Point centroid = Point::centroid(region);
    std::vector<Point> segment;
    for (auto& point : region) {
        if (point.m_xyz[2] < centroid.m_xyz[2] + CLEAN) {
            segment.push_back(point);
        }
    }
    return segment;
}
