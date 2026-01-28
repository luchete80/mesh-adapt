#pragma once
#include <vector>
#include <array>

namespace mesh_adapt {

struct Node2D {
    double x, y;
};

struct Quad {
    std::array<int,4> ids;
};

class Mesh2D {
public:
    std::vector<Node2D> nodes;
    std::vector<Quad>   quads;

    int add_node(double x, double y);
    int add_quad(int n0,int n1,int n2,int n3);

    size_t num_nodes() const;
    size_t num_quads() const;
};

}

