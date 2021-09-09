#include <fstream>
#include <iostream>
#include <nanoflann.hpp>
#include <set>
#include <sstream>
#include <vector>

#include "i3dpcl.h"
#include "io.h"
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

void i3dpcl::write(const k4a_image_t& pclImage, const k4a_image_t& rgbImage,
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

void i3dpcl::write(std::vector<Point>& pCloud, std::vector<Point>& pCloudSeg)
{
    std::vector<Point> points = colourPCloudSeg(pCloud, pCloudSeg);
    const std::string FILE = io::pwd() + "/output/context.pcl";

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

void i3dpcl::write(std::vector<Point>& points, const std::string& FILE)
{
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

void i3dpcl::write1(std::vector<Point>& points)
{
    const std::string FILE = io::pwd() + "/output/context1.pcl";

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

std::vector<Point> i3dpcl::build(
    const int& w, const int& h, const int16_t* pCloudData, const uint8_t* bgra)
{
    std::vector<Point> pCloud;
    for (int i = 0; i < w * h; i++) {
        Point point {};
        point.m_xyz[0] = pCloudData[3 * i + 0];
        point.m_xyz[1] = pCloudData[3 * i + 1];
        point.m_xyz[2] = pCloudData[3 * i + 2];
        if (point.m_xyz[2] == 0) {
            continue;
        }
        uint8_t r = bgra[4 * i + 2];
        uint8_t g = bgra[4 * i + 1];
        uint8_t b = bgra[4 * i + 0];
        uint8_t a = bgra[4 * i + 3];
        uint8_t rgba[4] = { r, g, b, a };
        point.setRGBA(rgba);

        if (point.m_rgba[0] == 0 && point.m_rgba[1] == 0 && point.m_rgba[2] == 0
            && point.m_rgba[3] == 0) {
            continue;
        }
        pCloud.push_back(point);
    }
    return pCloud;
}

void i3dpcl::write(const int& w, const int& h, const int16_t* pCloudData,
    const uint8_t* rgbData, const std::string& FILE)
{
    std::vector<Point> points = build(w, h, pCloudData, rgbData);

    PLY_HEADER;
    std::stringstream ss;
    for (auto& point : points) {
        int16_t x = point.m_xyz[0];
        int16_t y = point.m_xyz[1];
        int16_t z = point.m_xyz[2];

        // k4a color image is in fact BGR (not RGB)
        auto r = (float)point.m_rgba[2];
        auto g = (float)point.m_rgba[1];
        auto b = (float)point.m_rgba[0];

        ss << x << " " << y << " " << z << " ";
        ss << r << " " << g << " " << b << std::endl;
    }
    std::ofstream ofs_text(FILE, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}
