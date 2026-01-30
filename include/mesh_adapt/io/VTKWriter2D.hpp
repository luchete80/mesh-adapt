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

} // namespace mesh_adapt
