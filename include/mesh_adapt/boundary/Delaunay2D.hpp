#pragma once

#include <vector>
#include <array>
#include "mesh_adapt/geometry/Vec2.hpp"
#include "mesh_adapt/geometry/Polyline2D.hpp"

namespace mesh_adapt {

struct Edge2D {
    int a, b;
    Edge2D(int A,int B):a(A),b(B){}
};

class Delaunay2D {
public:
    Delaunay2D() = default;

    // Build domain ONLY from two closed polylines
    void build_from_two_polylines(
        const Polyline2D& outer,
        const Polyline2D& inner
    );

    void triangulate();

    const std::vector<std::array<int,3>>& get_triangles() const {
        return triangles;
    }

    const std::vector<Vec2>& get_points() const {
        return points;
    }

private:
    std::vector<Vec2> points;
    std::vector<Edge2D> constraints;
    std::vector<std::array<int,3>> triangles;

private:
    bool circumcircle_contains(
        const std::array<int,3>& tri,
        const Vec2& p
    ) const;

    bool edges_equal(const Edge2D& e1,
                     const Edge2D& e2) const;

    void add_constraints_from_loop(
        const std::vector<int>& loop_ids
    );
};

} // namespace mesh_adapt
