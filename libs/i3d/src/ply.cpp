#include <fstream>
#include <iostream>
#include <nanoflann.hpp>
#include <set>
#include <sstream>
#include <vector>

#include "io.h"
#include "ply.h"
#include "point.h"
#include "searcher.h"

#define PLY_HEADER                                                             \
    std::ofstream ofs(FILE);                                                   \
    ofs << "ply" << std::endl;                                                 \
    ofs << "format ascii 1.0" << std::endl;                                    \
    ofs << "element vertex"                                                    \
        << " " << points.size() << std::endl;                                  \
    ofs << "property float x" << std::endl;                                    \
    ofs << "property float y" << std::endl;                                    \
    ofs << "property float z" << std::endl;                                    \
    ofs << "property uchar red" << std::endl;                                  \
    ofs << "property uchar green" << std::endl;                                \
    ofs << "property uchar blue" << std::endl;                                 \
    ofs << "end_header" << std::endl;                                          \
    ofs.close()

struct t_rgbPoint {
    int16_t xyz[3];
    uint8_t rgb[3];
};

void ply::write(const k4a_image_t& pclImage, const k4a_image_t& rgbImage,
    const std::string& FILE)
{
    std::vector<t_rgbPoint> points;
    int width = k4a_image_get_width_pixels(pclImage);
    int height = k4a_image_get_height_pixels(rgbImage);

    auto* point_cloud_image_data
        = (int16_t*)(void*)k4a_image_get_buffer(pclImage);
    uint8_t* color_image_data = k4a_image_get_buffer(rgbImage);

    for (int i = 0; i < width * height; i++) {
        t_rgbPoint point {};
        point.xyz[0] = point_cloud_image_data[3 * i + 0];
        point.xyz[1] = point_cloud_image_data[3 * i + 1];
        point.xyz[2] = point_cloud_image_data[3 * i + 2];
        if (point.xyz[2] == 0) {
            continue;
        }
        point.rgb[0] = color_image_data[4 * i + 0];
        point.rgb[1] = color_image_data[4 * i + 1];
        point.rgb[2] = color_image_data[4 * i + 2];
        uint8_t alpha = color_image_data[4 * i + 3];

        if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0
            && alpha == 0) {
            continue;
        }
        points.push_back(point);
    }
    /** write to file */
    PLY_HEADER;
    std::stringstream ss;
    for (auto& point : points) {
        // image data is BGR
        ss << (float)point.xyz[0] << " " << (float)point.xyz[1] << " "
           << (float)point.xyz[2];
        ss << " " << (float)point.rgb[2] << " " << (float)point.rgb[1] << " "
           << (float)point.rgb[0];
        ss << std::endl;
    }
    std::ofstream ofs_text(FILE, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

/////////////////////////////////////////////////////////////
//                   colorize segment
/////////////////////////////////////////////////////////////

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

typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloud<float>>, PointCloud<float>,
    3>
    kdTree;

std::vector<Point> colourPCloudSeg(
    std::vector<Point>& pCloud, std::vector<Point>& pCloudSeg)
{

    const int k = 1;
    const size_t N = pCloudSeg.size();
    PointCloud<float> cloud;

    /** build kd-tree */
    kdTree index(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    toNanoflannPoint(cloud, pCloudSeg);
    int chunk_size = 100;
    for (size_t i = 0; i < N; i = i + chunk_size) {
        size_t end = std::min(size_t(i + chunk_size), N - 1);
        index.addPoints(i, end);
    }

    for (auto& queryPoint : pCloud) {
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

        for (size_t i = 0; i < resultSet.size(); ++i) {
            auto x = (int16_t)cloud.pts[kIndex[i]].x;
            auto y = (int16_t)cloud.pts[kIndex[i]].y;
            auto z = (int16_t)cloud.pts[kIndex[i]].z;
            Point nN(x, y, z);
            if (queryPoint == nN) {
                queryPoint.m_crgb = " 174 1 126";
            }
        }
    }
    return pCloud;
}

/////////////////////////////////////////////////////////////
//                   colorize segment
/////////////////////////////////////////////////////////////

void ply::write(std::vector<Point>& pCloud, std::vector<Point>& pCloudSeg)
{
    std::vector<Point> points = colourPCloudSeg(pCloud, pCloudSeg);
    const std::string FILE = io::pwd() + "/output/context.ply";

    /** write to file */
    PLY_HEADER;
    std::stringstream ss;
    for (const auto& point : points) {
        std::string x = std::to_string(point.m_xyz[0]);
        std::string y = std::to_string(point.m_xyz[1]);
        std::string z = std::to_string(point.m_xyz[2]);
        ss << x << " " << y << " " << z << point.m_crgb << std::endl;
    }
    std::ofstream ofs_text(FILE, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

void ply::write(std::vector<Point>& points)
{
    const std::string FILE = io::pwd() + "/output/context.ply";

    /** write to file */
    PLY_HEADER;
    std::stringstream ss;
    for (const auto& point : points) {
        std::string x = std::to_string(point.m_xyz[0]);
        std::string y = std::to_string(point.m_xyz[1]);
        std::string z = std::to_string(point.m_xyz[2]);
        std::string r = std::to_string(point.m_rgba[0]);
        std::string g = std::to_string(point.m_rgba[1]);
        std::string b = std::to_string(point.m_rgba[2]);

        ss << x << " " << y << " " << z << " " << r << " " << g << " " << b
           << std::endl;
    }
    std::ofstream ofs_text(FILE, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}
