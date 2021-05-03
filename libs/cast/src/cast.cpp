#include "cast.h"

std::vector<Point> cast::toPoint(const std::vector<float>& pcl,
    const std::vector<uint8_t>& color, const int& numPoints)
{
    std::vector<Point> points;
    for (int i = 0; i < numPoints; i++) {
        float x = pcl[3 * i + 0];
        float y = pcl[3 * i + 1];
        float z = pcl[3 * i + 2];

        if (x == 0 || y == 0 || z == 0) {
            continue;
        }
        std::vector<float> rgb(3);
        rgb[0] = color[3 * i + 0];
        rgb[1] = color[3 * i + 1];
        rgb[2] = color[3 * i + 2];
        Point point(x, y, z);
        point.setColor(rgb);
        points.push_back(point);
    }
    return points;
}

std::pair<std::vector<float>, std::vector<uint8_t>> cast::toPcl(
    std::vector<Point>& points)
{
    std::vector<float> pcl;
    std::vector<uint8_t> color;
    std::string delimiter = " ";
    for (auto& point : points) {
        pcl.push_back(point.m_xyz[0]);
        pcl.push_back(point.m_xyz[1]);
        pcl.push_back(point.m_xyz[2]);

        color.push_back(point.m_rgb[0]);
        color.push_back(point.m_rgb[1]);
        color.push_back(point.m_rgb[2]);
    }
    return { pcl, color };
}
std::pair<std::vector<float>, std::vector<uint8_t>> cast::toClusteredPcl(
    std::vector<Point>& points)
{
    std::vector<float> pcl;
    std::vector<uint8_t> color;
    std::string delimiter = " ";
    for (auto& point : points) {
        pcl.push_back(point.m_xyz[0]);
        pcl.push_back(point.m_xyz[1]);
        pcl.push_back(point.m_xyz[2]);

        /** cast Point color to pcl-point color */
        std::string colorStr = point.m_clusterColor;
        colorStr.erase(0, 1);
        size_t last = 0;
        size_t next = 0;
        int rgb[3];
        int index = 0;
        while ((next = colorStr.find(delimiter, last)) != std::string::npos) {
            int colorVal = std::stoi(colorStr.substr(last, next - last));
            rgb[index] = colorVal;
            last = next + 1;
            index++;
        }
        int colorVal = std::stoi(colorStr.substr(last));
        rgb[index] = colorVal;

        color.push_back(rgb[0]);
        color.push_back(rgb[1]);
        color.push_back(rgb[2]);
    }
    return { pcl, color };
}
