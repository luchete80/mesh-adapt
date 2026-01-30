#pragma once
#include <vector>
#include <array>
#include "mesh_adapt/geometry/Vec2.hpp"
#include "mesh_adapt/geometry/Polyline2D.hpp"

namespace mesh_adapt {

// Simple 2D edge
struct Edge2D {
    int a, b;
    Edge2D(int i, int j) : a(i), b(j) {}
};

// Delaunay triangulation class (supports constrained edges from polylines)
class Delaunay2D {
public:
    Delaunay2D(const std::vector<Vec2>& pts) : points(pts) {}

    // Insert constraints from a Polyline2D
    void insert_constraints(const Polyline2D& polyline);

    // Perform the triangulation
    void triangulate();

    // Get triangles
    const std::vector<std::array<int,3>>& get_triangles() const { return triangles; }

private:
    std::vector<Vec2> points;
    std::vector<std::array<int,3>> triangles;
    std::vector<Edge2D> constraints;

    // Helper functions
    bool circumcircle_contains(const std::array<int,3>& tri, const Vec2& p) const;
    bool edges_equal(const Edge2D& e1, const Edge2D& e2) const;
};

}
