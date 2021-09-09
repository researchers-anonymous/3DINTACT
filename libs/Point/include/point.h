#ifndef POINT_H
#define POINT_H

#include <array>
#include <iostream>
#include <vector>

extern const int UNLABELED;

class Point {

public:
    int m_id {};
    int m_cluster;                       // <- cluster label
    std::string m_crgb;                  // <- cluster color
    std::array<uint8_t, 4> m_rgba {};    // <- image (openGL format)
    std::array<uint8_t, 4> m_bgra {};    // <- image (cv Frame format)
    std::array<int16_t, 3> m_xyz {};     // <- coordinates (openGL format)
    std::pair<Point*, float> m_distance; // <- Euclidean distance to a Point*

    /** 3D point constructors */
    Point();
    Point(int16_t x, int16_t y, int16_t z);

    /**
     * sort
     *   Sorts a given set of points using m_distance as a criteria.
     *
     *  @param points
     *    Set of points with distance measures to a common/shared point.
     */
    static void sort(std::vector<Point>& points);

    /**
     * rgb
     *   Set the color corresponding color for each xyz point.
     *
     *  @param rgb
     *    Color.
     */

    /**
     * centroid
     *   Computes the centroid for a given set of points.
     *
     *  @param points
     *    Points of interest.
     *
     *  @retval
     *    Centroid.
     */
    static Point centroid(std::vector<Point>& points);

    /**
     * distance
     *   Computes distance of *this Point to another point.
     *
     *  @param other
     *    Other Point.
     *
     *  @retval
     *    Euclidean distance to other point.
     */
    [[nodiscard]] int16_t distance(const Point& other) const;

    /** operator overrides */
    bool operator<(const Point& other) const;
    bool operator!=(const Point& other) const;
    bool operator==(const Point& other) const;
    friend std::ostream& operator<<(std::ostream& t_stream, const Point& point);
    friend std::istream& operator>>(std::istream& t_stream, Point& point);

    void setPoint(const int16_t xyz[3]);
    void setRGBA(const uint8_t* rgba);
    void setBGRA(const uint8_t* bgra);
};
#endif /* POINT_H */
