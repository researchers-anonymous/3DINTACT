
#include <cmath>
#include <iostream>
#include <nanoflann.hpp>

#include "knn.h"
#include "utils.h"

/** change to 1 to print knn solution to terminal */
#define PRINT_KNN 0

#if PRINT_KNN == 1
#define KNN_RESULTS                                                            \
    std::cout << "num of points -> " << points.size() << std::endl;            \
    std::cout << "  query point -> (" << queryPoint[0] << ", "                 \
              << queryPoint[1] << ", " << queryPoint[2] << ")" << std::endl;   \
    std::cout << "searching for -> " << k << " nearest neighbours"             \
              << std::endl;                                                    \
    for (size_t i = 0; i < resultSet.size(); ++i) {                            \
        std::cout << "#" << i << ",\t"                                         \
                  << "index: " << ret_index[i] << ",\t"                        \
                  << "dist: " << out_dist_sqr[i] << ",\t"                      \
                  << "point: (" << cloud.pts[ret_index[i]].x << ", "           \
                  << cloud.pts[ret_index[i]].y << ", "                         \
                  << cloud.pts[ret_index[i]].z << ")" << std::endl;            \
    }                                                                          \
    std::cout << std::endl
#else
#define SHOW_RESULTS
#endif

template <typename T>
void castToNanoflannPoint(
    PointCloud<T>& point, const std::vector<Point>& points)
{
    const size_t N = points.size();
    point.pts.resize(N);
    for (size_t i = 0; i < N; i++) {
        point.pts[i].x = points[i].m_xyz[0];
        point.pts[i].y = points[i].m_xyz[1];
        point.pts[i].z = points[i].m_xyz[2];
    }
}

template <typename num_t>
std::vector<std::pair<Point, float>> nanoflannKnn(
    const std::vector<Point>& points, const size_t& indexOfQueryPoint,
    const size_t& k)
{
    const size_t N = points.size();
    PointCloud<num_t> cloud;

    /** construct a kd-tree index: */
    typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
        nanoflann::L2_Simple_Adaptor<num_t, PointCloud<num_t>>,
        PointCloud<num_t>, 3 /* dim */
        >
        my_kd_tree_t;

    my_kd_tree_t index(3 /*dim*/, cloud,
        nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));

    /** adapt points to nanoflann::PointCloud<T> */
    castToNanoflannPoint(cloud, points);

    /** parse query point */
    num_t queryPoint[3] = { points[indexOfQueryPoint].m_xyz[0],
        points[indexOfQueryPoint].m_xyz[1],
        points[indexOfQueryPoint].m_xyz[2] };

    /** knn search */
    size_t chunk_size = 100;
    for (size_t i = 0; i < N; i = i + chunk_size) {
        size_t end = std::min(size_t(i + chunk_size), N - 1);

        /** Inserts all points from [i, end] */
        index.addPoints(i, end);
    }
    size_t removePointIndex = N - 1;
    index.removePoint(removePointIndex);
    size_t ret_index[k];
    num_t out_dist_sqr[k];
    nanoflann::KNNResultSet<num_t> resultSet(k);
    resultSet.init(ret_index, out_dist_sqr);
    index.findNeighbors(resultSet, queryPoint, nanoflann::SearchParams(10));

    /** [ optional ]
     *  to print results to terminal set SHOW 1 */
    SHOW_RESULTS;

    /** collect results and return solution */
    std::vector<std::pair<Point, float>> nnHeap;
    for (size_t i = 0; i < resultSet.size(); ++i) {
        auto x = (float)cloud.pts[ret_index[i]].x;
        auto y = (float)cloud.pts[ret_index[i]].y;
        auto z = (float)cloud.pts[ret_index[i]].z;
        Point point(x, y, z);
        nnHeap.push_back({ point, out_dist_sqr[i] });
    }
    return nnHeap;
}

std::vector<std::pair<Point, float>> knn::compute(
    std::vector<Point>& points, const int& k, const int& indexOfQueryPoint)
{
    return nanoflannKnn<float>(points, indexOfQueryPoint, k);
}
