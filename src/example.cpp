#include <iostream>
#include "mesh_adapt/core/Mesh2D.hpp"

int main() {

    mesh_adapt::Mesh2D mesh;

    mesh.add_node(0.0, 0.0);
    mesh.add_node(1.0, 0.0);
    mesh.add_node(1.0, 1.0);
    mesh.add_node(0.0, 1.0);

    mesh.add_quad(0,1,2,3);

    std::cout << "Mesh nodes: " << mesh.num_nodes() << "\n";
    std::cout << "Mesh quads: " << mesh.num_quads() << "\n";

    return 0;
}
