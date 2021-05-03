#ifndef OBJECT_H
#define OBJECT_H

#include <vector>
#include <string>
#include "point.h"
#include "region.h"

class Object {
public:
    bool m_movable = false;
    bool m_projectable = false;
    bool m_interactable = false;

    std::vector<Point> m_points;
    std::pair<Point, Point> m_boundary;

    explicit Object(std::vector<Point>& points):m_points(points){
        m_boundary = region::queryBoundary(m_points);
    }
};

#endif /* OBJECT_H */