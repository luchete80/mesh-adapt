#pragma once
#include <vector>
#include "mesh_adapt/geometry/Vec2.hpp"
#include "mesh_adapt/core/Node2D.hpp"

namespace mesh_adapt {

class Contour2D {
public:
    virtual ~Contour2D() = default;

    // Signed distance function
    virtual double distance(const Vec2& p) const = 0;

    // Closest point projection
    virtual Vec2 project(const Vec2& p) const = 0;

    // Discretization of boundary
    virtual std::vector<Vec2> sample(int n) const = 0;
};

struct ContourPoint {
    Vec2 p;
    double s;          // parámetro sobre el contour
    NodeFlag flag;
    int from_ring_gid; // -1 si constrained
};

}

