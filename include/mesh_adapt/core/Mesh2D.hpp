#pragma once
#include <vector>
#include <array>
#include <cstddef>   // <-- para size_t
#include "Node2D.hpp"
#include "mesh_adapt/geometry/Vec2.hpp"
#include "mesh_adapt/core/Node2D.hpp"


namespace mesh_adapt {


struct Quad {
    std::array<int,4> ids;
    int& operator[](int i) {
        switch(i) {
            case 0: return ids[0];
            case 1: return ids[1];
            case 2: return ids[2];
            case 3: return ids[3];
            default: throw std::out_of_range("Quad index out of range");
        }
    }

    const int& operator[](int i) const {
        switch(i) {
            case 0: return ids[1];
            case 1: return ids[2];
            case 2: return ids[3];
            case 3: return ids[4];
            default: throw std::out_of_range("Quad index out of range");
        }
  };
};

class Mesh2D {
public:
    
    std::vector<Node2D> nodes;
    std::vector<Quad>   quads;

    int add_node(double x, double y);
    void add_quad(int n0,int n1,int n2,int n3);

    size_t num_nodes() const;
    size_t num_quads() const;
    
    std::vector<int> find_boundary_nodes() const;
    
    Vec2 node(int i) const {
        return nodes[i].x;
    }
    
};

}

