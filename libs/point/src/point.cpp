#include <algorithm>
#include <cmath>

#include "point.h"

extern const int UNLABELED = -1;

bool compare(const Point& point, const Point& other)
{
    return point.m_distance.second < other.m_distance.second;
}

void Point::sort(std::vector<Point>& points)
{
    std::sort(points.begin(), points.end(), compare);
}

Point::Point()
    : m_xyz({ (int16_t)0.0, (int16_t)0.0, (int16_t)0.0 })
    , m_cluster(UNLABELED)
    , m_distance(nullptr, __DBL_MAX__)
{
    m_crgb = " 0 0 0";
    // m_rgb = { 0, 0, 0 };
}

Point::Point(int16_t x, int16_t y, int16_t z)
    : m_xyz({ x, y, z })

    , m_cluster(UNLABELED)
    , m_distance(nullptr, __DBL_MAX__)
{
    m_crgb = " 0 0 0";
    // m_rgb = { 0, 0, 0 };
}

void Point::setPoint(const int16_t xyz[3])
{
    m_xyz[0] = xyz[0];
    m_xyz[1] = xyz[1];
    m_xyz[2] = xyz[2];
}

void Point::setPixel_GL(const uint8_t rgba[4])
{
    m_rgba[0] = rgba[0];
    m_rgba[1] = rgba[1];
    m_rgba[2] = rgba[2];
    m_rgba[3] = rgba[3];
}

void Point::setPixel_CV(const uint8_t bgra[4])
{
    m_bgra[0] = bgra[0];
    m_bgra[1] = bgra[1];
    m_bgra[2] = bgra[2];
    m_bgra[3] = bgra[3];
}

int16_t Point::distance(const Point& other) const
{
    int16_t x = m_xyz[0] - other.m_xyz[0];
    int16_t y = m_xyz[1] - other.m_xyz[1];
    int16_t z = m_xyz[2] - other.m_xyz[2];
    return (int16_t)std::sqrt((x * x) + (y * y) + (z * z));
}

// TODO: quick test
Point Point::centroid(std::vector<Point>& points)
{
    float xSum = 0;
    float ySum = 0;
    float zSum = 0;
    for (const auto& point : points) {
        xSum = xSum + (float)point.m_xyz[0];
        ySum = ySum + (float)point.m_xyz[1];
        zSum = zSum + (float)point.m_xyz[2];
    }
    return Point { (int16_t)(xSum / points.size()),
        (int16_t)(ySum / points.size()), (int16_t)(zSum / points.size()) };
}

bool Point::operator==(const Point& rhs) const
{
    return (m_xyz[0] == rhs.m_xyz[0] && m_xyz[1] == rhs.m_xyz[1]
        && m_xyz[2] == rhs.m_xyz[2]);
}

bool Point::operator!=(const Point& rhs) const
{
    return (m_xyz[0] != rhs.m_xyz[0] || m_xyz[1] != rhs.m_xyz[1]
        || m_xyz[2] != rhs.m_xyz[2]);
}

bool Point::operator<(const Point& rhs) const
{
    return (this->m_distance.second < rhs.m_distance.second);
}

std::ostream& operator<<(std::ostream& stream, const Point& point)
{
    stream << point.m_xyz[0] << " " << point.m_xyz[1] << " " << point.m_xyz[2];
    return stream;
}

std::istream& operator>>(std::istream& stream, Point& point)
{
    stream >> point.m_xyz[0] >> point.m_xyz[1] >> point.m_xyz[2];
    return stream;
}
