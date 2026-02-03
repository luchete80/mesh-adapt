#pragma once
#include "mesh_adapt/geometry/Vec2.hpp"

namespace mesh_adapt {
  
// Flags de nodos (del código Python)
enum NodeFlag {
    NODE_PROJECTED = 0,
    NODE_CRITICAL = 1,   // nodos del contorno original
    NODE_INTERIOR = 2,
    NODE_AXIS = 3,
    NODE_EXTERNAL = 4,
    NODE_SUBDIVIDED = 5,
    NODE_RING = 6,
    NODE_CENTER = 7 //In the center of a quad
};


struct Node2D {
    Vec2 x;
    NodeFlag flag = NODE_INTERIOR;
    
    bool boundary = false;
    bool constrained = false;

    Node2D() = default;
    Node2D(double X, double Y) : x(X,Y) {}
};

}

