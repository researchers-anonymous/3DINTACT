#include "outliers.h"
#include "point.h"
#include <vector>

const float IQR = 1.5; // <- Inter quartile range (in theory 1.5)

std::vector<Point> outliers::filter(std::vector<Point>& points)
{
    /** find within-point variance using the centroid */
    Point centroid = Point::centroid(points);
    for (auto& point : points) {
        float distance = centroid.distance(point);
        point.m_distance.second = distance;
    }
    Point::sort(points);

    /** remove outliers */
    float sum = 0;
    float varsum = 0;

    for (auto& point : points) {
        sum += point.m_distance.second;
    }
    float mean = sum / (float)points.size();

    for (auto& point : points) {
        varsum += (float)pow((point.m_distance.second - mean), 2);
    }

    float variance = varsum / (float)points.size();
    float sd = std::sqrt(variance);

    auto upperBound = mean + (IQR * sd);
    int reject = 0;
    for (auto& point : points) {
        if (point.m_distance.second > upperBound) {
            reject++;
        }
    }
    points.resize(points.size() - reject);
    return points;
}
