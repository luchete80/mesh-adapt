#include "mesh_adapt/core/Mesh2D.hpp"

namespace mesh_adapt {

int Mesh2D::add_node(double x, double y)
{
    nodes.push_back({x, y});
    return (int)nodes.size() - 1;
}

void Mesh2D::add_quad(int a, int b, int c, int d)
{
    quads.push_back({a, b, c, d});
}

size_t Mesh2D::num_nodes() const
{
    return (int)nodes.size();
}

size_t Mesh2D::num_quads() const
{
    return (int)quads.size();
}

} // namespace mesh_adapt
