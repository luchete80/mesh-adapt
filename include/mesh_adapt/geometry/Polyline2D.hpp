#pragma once
#include <vector>
#include <cmath>
#include "Contour2D.hpp"


namespace mesh_adapt {

class Polyline2D : public Contour2D {
public:
    std::vector<Vec2> pts;

    Polyline2D() = default;

    Polyline2D(const std::vector<Vec2>& points)
        : pts(points) {}

    double distance(const Vec2& p) const override;
    Vec2 project(const Vec2& p) const override;
    std::vector<Vec2> sample(int n) const override;
};

}
