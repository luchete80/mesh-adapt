#pragma once

#include "Mesh2D.hpp"
#include "mesh_adapt/geometry/Polyline2D.hpp"
#include "mesh_adapt/geometry/PolygonUtils.hpp"

namespace mesh_adapt {

Mesh2D generate_structured_grid(double x_min, double x_max,
                                double y_min, double y_max,
                                double SL) {
    Mesh2D mesh;
    int nx = static_cast<int>((x_max - x_min) / SL) + 1;
    int ny = static_cast<int>((y_max - y_min) / SL) + 1;

    for(int j = 0; j < ny; ++j) {
        for(int i = 0; i < nx; ++i) {
            double x = x_min + i * SL;
            double y = y_min + j * SL;
            mesh.add_node(x, y);
        }
    }

    // Crear quads
    for(int j = 0; j < ny-1; ++j) {
        for(int i = 0; i < nx-1; ++i) {
            int n0 = j*nx + i;
            int n1 = n0 + 1;
            int n2 = n0 + nx + 1;
            int n3 = n0 + nx;
            mesh.add_quad(n0, n1, n2, n3);
        }
    }

    return mesh;
}

std::vector<int> filter_near_boundary_nodes(
        const Mesh2D& mesh,
        const Polyline2D& contour,
        double rmax) 
{
    std::vector<int> near_nodes;
    const auto& nodes = mesh.get_nodes();
    for(size_t i = 0; i < nodes.size(); ++i) {
        if(contour.distance(nodes[i].x) <= rmax) {
            near_nodes.push_back(i);
        }
    }
    return near_nodes;
}

Mesh2D filter_nodes_inside_contour(
    const Mesh2D& mesh,
    const Polyline2D& contour
)
{
    Mesh2D new_mesh;

    const auto& pts = contour.get_points();

    // map old → new
    std::vector<int> old_to_new(mesh.num_nodes(), -1);

    // 1. Copiar solo nodos interiores
    for(size_t i=0; i<mesh.num_nodes(); ++i)
    {
        Vec2 p = mesh.node(i);

        if(point_in_polygon(p, pts)) {
            int new_id = new_mesh.add_node(p.x, p.y);
            old_to_new[i] = new_id;
        }
    }

    // 2. Copiar solo quads válidos
    for(const auto& q : mesh.get_quads())
    {
        int a = old_to_new[q[0]];
        int b = old_to_new[q[1]];
        int c = old_to_new[q[2]];
        int d = old_to_new[q[3]];

        if(a!=-1 && b!=-1 && c!=-1 && d!=-1)
            new_mesh.add_quad(a,b,c,d);
    }

    return new_mesh;
}
Mesh2D remove_nodes_near_contour(
    const Mesh2D& mesh,
    const Contour2D& contour,
    double SL
)
{
    Mesh2D new_mesh;
    std::vector<int> old_to_new(mesh.num_nodes(), -1);

    // ------------------------------------------------------------
    // 1. Copy nodes that are far enough from the contour
    // ------------------------------------------------------------
    for(size_t i = 0; i < mesh.num_nodes(); ++i)
    {
        Vec2 p = mesh.node(i);

        if(contour.distance(p) > SL * 0.5)
        {
            int new_id = new_mesh.add_node(p.x, p.y);
            old_to_new[i] = new_id;
        }
    }

    // ------------------------------------------------------------
    // 2. Rebuild quads: keep only fully valid elements
    // ------------------------------------------------------------
    int kept = 0;
    int removed = 0;

    for(const auto& quad : mesh.get_quads())
    {
        int a_old = quad[0];
        int b_old = quad[1];
        int c_old = quad[2];
        int d_old = quad[3];

        int a = old_to_new[a_old];
        int b = old_to_new[b_old];
        int c = old_to_new[c_old];
        int d = old_to_new[d_old];

        // Only keep quads whose 4 nodes survived
        if(a != -1 && b != -1 && c != -1 && d != -1)
        {
            new_mesh.add_quad(a, b, c, d);
            kept++;
        }
        else
        {
            removed++;
        }
    }

    std::cout << "[remove_nodes_near_contour]\n";
    std::cout << "   quads kept    = " << kept << "\n";
    std::cout << "   quads removed = " << removed << "\n";

    return new_mesh;
}



}
