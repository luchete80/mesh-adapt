#pragma once
#include <vector>
#include "Edge.hpp"
#include "mesh_adapt/geometry/Vec2.hpp"
#include <utility> // std::pair

namespace mesh_adapt {

enum class RefinementCause : uint8_t {
    NONE        = 0,
    FALLBACK    = 1 << 0,
    INITIAL     = 1 << 1,
    PLASTIC     = 1 << 2,
    ERROR_EST   = 1 << 3
};

struct EdgeInfo
{
    Vec2 p0, p1;                   // puntos extremos
    std::vector<std::pair<int,int>> quad_refs; // (quad_id, local_edge_id)
    bool subdivide = false;         // marcado por refined points
    uint8_t cause_mask = 0;

    EdgeInfo() = default;
    EdgeInfo(const Vec2& a, const Vec2& b) : p0(a), p1(b) {}
};

inline void add_cause(EdgeInfo& e, RefinementCause c)
{
    e.subdivide = true;
    e.cause_mask |= static_cast<uint8_t>(c);
}

inline bool has_cause(const EdgeInfo& e, RefinementCause c)
{
    return e.cause_mask & static_cast<uint8_t>(c);
}

}

