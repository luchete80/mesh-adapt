#include <iostream>
#include "mesh_adapt/core/Mesh2D.hpp"
#include "mesh_adapt/geometry/Polyline2D.hpp"


int main() {

    mesh_adapt::Mesh2D mesh;

    mesh.add_node(0.0, 0.0);
    mesh.add_node(1.0, 0.0);
    mesh.add_node(1.0, 1.0);
    mesh.add_node(0.0, 1.0);

    mesh.add_quad(0,1,2,3);

    std::cout << "Mesh nodes: " << mesh.num_nodes() << "\n";
    std::cout << "Mesh quads: " << mesh.num_quads() << "\n";

    // ----------------------------
    // 2. Create geometric contour
    //    (quare for example)
    // ----------------------------
    std::vector<mesh_adapt::Vec2> contour_pts = {
        {0.0, 0.0},
        {2.0, 0.0},
        {2.0, 2.0},
        {0.0, 2.0},
        {0.0, 0.0} // cerrar polyline
    };

    mesh_adapt::Polyline2D contour(contour_pts);

    // ----------------------------
    // 3. Test: distance to an external point
    // ----------------------------
    mesh_adapt::Vec2 p = {1.0, 1.0};

    double d = contour.distance(p);

    std::cout << "Distance from (1,1) to contour = " << d << "\n";


    // ----------------------------
    // 4. Test: Boundary projection
    // ----------------------------
    mesh_adapt::Vec2 q = contour.project(p);

    std::cout << "Projection of (1,1) onto contour = ("
              << q.x << ", " << q.y << ")\n";
              
    return 0;

    // --- Find boundary nodes ---
    auto boundary = mesh.find_boundary_nodes();

    std::cout << "Boundary nodes:\n";
    for(int id : boundary) {
        auto p = mesh.node(id);

        auto proj = contour.project(p);

        std::cout << "  node " << id
                  << " (" << p.x << "," << p.y << ")"
                  << " -> projected (" << proj.x << "," << proj.y << ")\n";
    }


}
