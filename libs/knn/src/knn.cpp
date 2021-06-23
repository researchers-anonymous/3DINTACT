#include <cmath>
#include <iostream>
#include <nanoflann.hpp>

#include "knn.h"

template <typename T> struct PointCloud {
    struct Point {
        T x, y, z;
    };

    std::vector<Point> pts;
    [[nodiscard]] inline size_t kdtree_get_point_count() const
    {
        return pts.size();
    }
    [[nodiscard]] inline T kdtree_get_pt(
        const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return pts[idx].x;
        else if (dim == 1)
            return pts[idx].y;
        else
            return pts[idx].z;
    }

    template <class BBOX> bool kdtree_get_bbox(BBOX& /* bb */) const
    {
        return false;
    }
};

void print(const std::vector<Point>& points, const PointCloud<float>& cloud,
    const float* point, const int& k,
    const nanoflann::KNNResultSet<float>& resultSet, const size_t* kIndex,
    const float* distsSquared)
{
#define SHOW_KNN_RESULTS 0
#if SHOW_KNN_RESULTS == 1
    std::cout << "search space = " << points.size() << " points \t"
              << " searching for the " << k << " nearest neighbors "
              << std::endl;
    for (size_t i = 0; i < resultSet.size(); ++i) {
        std::cout << "query point >> (" << point[0] << ", " << point[1] << ", "
                  << point[2] << ") \t"
                  << "nearest neighbour #" << i << ",\t"
                  << "point >> (" << cloud.pts[kIndex[i]].x << ", "
                  << cloud.pts[kIndex[i]].y << ", " << cloud.pts[kIndex[i]].z
                  << ")"
                  << "\t"
                  << "index: " << kIndex[i] << ",\t"
                  << "dist: " << distsSquared[i] << " (squared L2 distance)"
                  << std::endl;
    }
#endif
}

template <typename T>
void toNanoflannPoint(PointCloud<T>& point, const std::vector<Point>& points)
{
    const size_t N = points.size();
    point.pts.resize(N);
    for (size_t i = 0; i < N; i++) {
        point.pts[i].x = points[i].m_xyz[0];
        point.pts[i].y = points[i].m_xyz[1];
        point.pts[i].z = points[i].m_xyz[2];
    }
}

/** alias kd-tree index */
typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloud<float>>, PointCloud<float>,
    3>
    kdTree;

std::vector<int> nanoflannKnn(
    const std::vector<Point>& points, const Point& queryPoint, const int& k)
{
    const size_t N = points.size();
    PointCloud<float> cloud;

    /** build kd-tree */
    kdTree index(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));

    toNanoflannPoint(cloud, points);

    int chunk_size = 100;

    for (size_t i = 0; i < N; i = i + chunk_size) {
        size_t end = std::min(size_t(i + chunk_size), N - 1);
        index.addPoints(i, end);
    }

    size_t removePointIndex = N - 1;
    index.removePoint(removePointIndex);
    size_t kIndex[k];
    float distsSquared[k];
    nanoflann::KNNResultSet<float> resultSet(k);
    resultSet.init(kIndex, distsSquared);

    float point[3] = { (float)queryPoint.m_xyz[0], (float)queryPoint.m_xyz[1],
        (float)queryPoint.m_xyz[2] };

    /**  search tree for point */
    index.findNeighbors(resultSet, point, nanoflann::SearchParams(10));

    print(points, cloud, point, k, resultSet, kIndex, distsSquared);

    std::vector<int> heap(kIndex, kIndex + sizeof kIndex / sizeof kIndex[0]);
    return heap;
}

std::vector<float> nanoflannKnn(const std::vector<Point>& points,
    const std::vector<Point>& queryPoints, const int& k)
{
    const size_t N = points.size();
    PointCloud<float> cloud;

    /** build kd-tree */
    kdTree index(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));

    toNanoflannPoint(cloud, points);

    int chunk_size = 100;

    for (size_t i = 0; i < N; i = i + chunk_size) {
        size_t end = std::min(size_t(i + chunk_size), N - 1);
        index.addPoints(i, end);
    }

    /**  for heaping distances to the Kth nearest neighbour */
    std::vector<float> heap;
    for (const auto& queryPoint : queryPoints) {

        size_t removePointIndex = N - 1;
        index.removePoint(removePointIndex);
        size_t kIndex[k];
        float distsSquared[k];
        nanoflann::KNNResultSet<float> resultSet(k);
        resultSet.init(kIndex, distsSquared);

        float point[3] = { (float)queryPoint.m_xyz[0],
            (float)queryPoint.m_xyz[1], (float)queryPoint.m_xyz[2] };

        /**  search tree for point */
        index.findNeighbors(resultSet, point, nanoflann::SearchParams(10));

        print(points, cloud, point, k, resultSet, kIndex, distsSquared);
        heap.emplace_back(std::sqrt(distsSquared[3]));
    }
    return heap;
}

std::vector<int> knn::compute(
    std::vector<Point>& points, const Point& queryPoint, const int& k)
{
    return nanoflannKnn(points, queryPoint, k);
}

std::vector<float> knn::compute(std::vector<Point>& points,
    const std::vector<Point>& queryPoints, const int& k)
{
    return nanoflannKnn(points, queryPoints, k);
}
