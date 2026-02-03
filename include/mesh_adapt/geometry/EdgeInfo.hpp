#pragma once
#include <vector>
#include "Edge.hpp"
#include "mesh_adapt/geometry/Vec2.hpp"
#include <utility> // std::pair

namespace mesh_adapt {

struct EdgeInfo
{
    Vec2 p0, p1;                   // puntos extremos
    std::vector<std::pair<int,int>> quad_refs; // (quad_id, local_edge_id)
    bool subdivide = false;         // marcado por refined points

    EdgeInfo() = default;
    EdgeInfo(const Vec2& a, const Vec2& b) : p0(a), p1(b) {}
};

}

