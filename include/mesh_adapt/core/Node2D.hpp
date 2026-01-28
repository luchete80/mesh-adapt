#pragma once
#include "mesh_adapt/geometry/Vec2.hpp"

namespace mesh_adapt {

struct Node2D {
    Vec2 x;          // posición
    bool boundary = false;
    bool constrained = false;

    Node2D() = default;
    Node2D(double X, double Y) : x(X,Y) {}
};

}

