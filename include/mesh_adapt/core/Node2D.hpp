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
    NODE_CENTER, = 7 //In the center of a quad
    NODE_NONE = 8
};


struct Node2D {
    Vec2 x;
    NodeFlag flag = NODE_NONE;
    
    bool boundary = false;
    bool constrained = false;

    Node2D() = default;
    Node2D(double X, double Y, NodeFlag f = NODE_NONE) : x(X,Y),flag(f) {}

    // Suma de nodos
    Node2D operator+(const Node2D& other) const {
        Node2D result;
        result.x = x + other.x;
        result.flag = flag; // opcional, podés decidir qué flag conservar
        result.boundary = boundary;
        result.constrained = constrained;
        return result;
    }

    // Multiplicación por escalar Node * double
    Node2D operator*(double s) const {
        Node2D result;
        result.x = x * s;
        result.flag = flag;
        result.boundary = boundary;
        result.constrained = constrained;
        return result;
    }

    // Multiplicación por escalar double * Node (friend)
    friend Node2D operator*(double s, const Node2D& n) {
        return n * s;  // reutiliza Node * double
    }
    
};

}

