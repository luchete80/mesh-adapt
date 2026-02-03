#pragma once
#include <fstream>
#include <string>
#include "mesh_adapt/core/Mesh2D.hpp"

namespace mesh_adapt {

inline void export_mesh_to_vtk(const Mesh2D& mesh,
                              const std::string& filename)
{
    std::ofstream out(filename);

    if(!out.is_open()) {
        throw std::runtime_error("Cannot open VTK file: " + filename);
    }

    out << "# vtk DataFile Version 3.0\n";
    out << "Mesh2D export\n";
    out << "ASCII\n";
    out << "DATASET UNSTRUCTURED_GRID\n";

    // -------------------------
    // POINTS
    // -------------------------
    out << "POINTS " << mesh.num_nodes() << " float\n";

    for(const auto& node : mesh.get_nodes()) {
        out << node.x.x << " " << node.x.y << " 0.0\n";
    }

    // -------------------------
    // CELLS (quads)
    // -------------------------
    int ncells = mesh.num_quads();
    int total_size = ncells * 5; // each quad: "4 i j k l"

    out << "CELLS " << ncells << " " << total_size << "\n";

    for(const auto& q : mesh.get_quads()) {
        out << "4 "
            << q[0] << " "
            << q[1] << " "
            << q[2] << " "
            << q[3] << "\n";
    }

    // -------------------------
    // CELL TYPES
    // -------------------------
    out << "CELL_TYPES " << ncells << "\n";
    for(int i=0; i<ncells; ++i) {
        out << "9\n"; // VTK_QUAD = 9
    }

    // -------------------------
    // POINT DATA (flags)
    // -------------------------
    out << "POINT_DATA " << mesh.num_nodes() << "\n";
    out << "SCALARS boundary_flag int 1\n";
    out << "LOOKUP_TABLE default\n";

    for(const auto& node : mesh.get_nodes()) {
        out << (node.boundary ? 1 : 0) << "\n";
    }

    out.close();
}


inline void export_points_vtk(const std::vector<Vec2>& pts,
                              const std::string& filename)
{
    std::ofstream out(filename);

    out << "# vtk DataFile Version 3.0\n";
    out << "Point cloud\n";
    out << "ASCII\n";
    out << "DATASET UNSTRUCTURED_GRID\n";

    out << "POINTS " << pts.size() << " float\n";
    for(auto& p : pts)
        out << p.x << " " << p.y << " 0.0\n";

    // vertices
    out << "CELLS " << pts.size()
        << " " << pts.size()*2 << "\n";

    for(size_t i=0; i<pts.size(); ++i)
        out << "1 " << i << "\n";

    out << "CELL_TYPES " << pts.size() << "\n";
    for(size_t i=0; i<pts.size(); ++i)
        out << "1\n"; // VTK_VERTEX
}



//==============================================================
// Export a Polyline2D as VTK_POLY_LINE
//==============================================================
inline void export_polyline_to_vtk(
    const Polyline2D& poly,
    const std::string& filename
){
    std::ofstream out(filename);

    if(!out.is_open()) {
        throw std::runtime_error(
            "Cannot open VTK file: " + filename
        );
    }

    const auto& pts = poly.get_points();

    out << "# vtk DataFile Version 3.0\n";
    out << "Polyline2D export\n";
    out << "ASCII\n";
    out << "DATASET UNSTRUCTURED_GRID\n";

    // ----------------------------------------------------------
    // POINTS
    // ----------------------------------------------------------
    out << "POINTS " << pts.size() << " float\n";

    for(const auto& p : pts) {
        out << p.x << " " << p.y << " 0.0\n";
    }

    // ----------------------------------------------------------
    // CELLS (one polyline cell)
    //
    // VTK format:
    // CELLS ncells total_size
    //
    // For one polyline with N points:
    // total_size = N+1
    //
    // Example:
    // 5 0 1 2 3 4
    // ----------------------------------------------------------
    out << "CELLS 1 " << (pts.size() + 1) << "\n";

    out << pts.size();
    for(size_t i = 0; i < pts.size(); ++i) {
        out << " " << i;
    }
    out << "\n";

    // ----------------------------------------------------------
    // CELL TYPES
    //
    // VTK_POLY_LINE = 4
    // ----------------------------------------------------------
    out << "CELL_TYPES 1\n";
    out << "4\n";

    out.close();
}


/// Export a triangle mesh (points + triangles) into a VTK file
inline void export_triangles_to_vtk(
    const std::vector<Vec2>& points,
    const std::vector<std::array<int,3>>& triangles,
    const std::string& filename
)
{
    std::ofstream out(filename);

    if(!out.is_open()) {
        throw std::runtime_error("Cannot open VTK file: " + filename);
    }

    out << "# vtk DataFile Version 3.0\n";
    out << "Triangle mesh export\n";
    out << "ASCII\n";
    out << "DATASET UNSTRUCTURED_GRID\n";

    // ------------------------------------------------------------
    // POINTS
    // ------------------------------------------------------------
    out << "POINTS " << points.size() << " float\n";

    for(const auto& p : points) {
        out << p.x << " " << p.y << " 0.0\n";
    }

    // ------------------------------------------------------------
    // CELLS (triangles)
    // Each triangle: "3 i j k"
    // ------------------------------------------------------------
    int ncells = (int)triangles.size();
    int total_size = ncells * 4;

    out << "CELLS " << ncells << " " << total_size << "\n";

    for(const auto& t : triangles) {
        out << "3 "
            << t[0] << " "
            << t[1] << " "
            << t[2] << "\n";
    }

    // ------------------------------------------------------------
    // CELL TYPES
    // VTK_TRIANGLE = 5
    // ------------------------------------------------------------
    out << "CELL_TYPES " << ncells << "\n";

    for(int i = 0; i < ncells; ++i) {
        out << "5\n";
    }

    out.close();

    std::cout << "[VTK] Exported triangles to " << filename
              << " (" << ncells << " tris)\n";
}

void export_quads_to_vtk(
    const std::vector<Vec2>& points,
    const std::vector<std::array<int,4>>& quads,
    const std::string& filename
)
{
    std::ofstream file(filename);
    file << "# vtk DataFile Version 3.0\n";
    file << "quads\n";
    file << "ASCII\n";
    file << "DATASET UNSTRUCTURED_GRID\n";

    file << "POINTS " << points.size() << " float\n";
    for(const auto& p : points)
        file << p.x << " " << p.y << " 0\n";

    size_t num_cells = quads.size();
    file << "CELLS " << num_cells << " " << num_cells*5 << "\n";
    for(const auto& q : quads)
        file << "4 " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << "\n";

    file << "CELL_TYPES " << num_cells << "\n";
    for(size_t i = 0; i < num_cells; ++i)
        file << "9\n"; // VTK_QUAD
}


} // namespace mesh_adapt
