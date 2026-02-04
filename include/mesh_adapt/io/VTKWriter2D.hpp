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
    out << "Mesh2D with Detailed Node Flags\n";
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
    // CELLS
    // -------------------------
    int ncells = mesh.num_quads();
    int total_size = ncells * 5;

    out << "CELLS " << ncells << " " << total_size << "\n";

    for(const auto& q : mesh.get_quads()) {
        out << "4 "
            << q[0] << " "
            << q[1] << " "
            << q[2] << " "
            << q[3] << "\n";
    }

    out << "CELL_TYPES " << ncells << "\n";
    for(int i=0; i<ncells; ++i) {
        out << "9\n";
    }

    // -------------------------
    // POINT DATA - DETALLADO
    // -------------------------
    out << "POINT_DATA " << mesh.num_nodes() << "\n";

    // Contadores para estadísticas
    std::map<NodeFlag, int> flag_counts;
    int boundary_count = 0;
    int constrained_count = 0;

    // 1. Flag numérico
    out << "SCALARS flag_int int 1\n";
    out << "LOOKUP_TABLE default\n";
    for(const auto& node : mesh.get_nodes()) {
        int flag_val = static_cast<int>(node.flag);
        out << flag_val << "\n";
        flag_counts[node.flag]++;
        if(node.boundary) boundary_count++;
        if(node.constrained) constrained_count++;
    }

    // 2. Flag categorizado (grupos)
    out << "SCALARS flag_category int 1\n";
    out << "LOOKUP_TABLE default\n";
    for(const auto& node : mesh.get_nodes()) {
        int category = 0;
        switch(node.flag) {
            case NODE_RING:       category = 1; break;    // Grupo 1: Ring
            case NODE_PROJECTED:  category = 2; break;    // Grupo 2: Projected
            case NODE_CRITICAL:   category = 3; break;    // Grupo 3: Critical
            case NODE_SUBDIVIDED: 
            case NODE_CENTER:     category = 4; break;    // Grupo 4: Subdivision
            case NODE_AXIS:       category = 5; break;    // Grupo 5: Axis
            case NODE_EXTERNAL:   category = 6; break;    // Grupo 6: External
            case NODE_INTERIOR:   category = 7; break;    // Grupo 7: Interior
            default:              category = 0; break;
        }
        out << category << "\n";
    }

    // 3. Boundary (sí/no)
    out << "SCALARS is_boundary int 1\n";
    out << "LOOKUP_TABLE default\n";
    for(const auto& node : mesh.get_nodes()) {
        out << (node.boundary ? 1 : 0) << "\n";
    }

    // 4. Constrained (sí/no)
    out << "SCALARS is_constrained int 1\n";
    out << "LOOKUP_TABLE default\n";
    for(const auto& node : mesh.get_nodes()) {
        out << (node.constrained ? 1 : 0) << "\n";
    }

    // 5. Combinación de flags (bitmask)
    out << "SCALARS flag_bitmask int 1\n";
    out << "LOOKUP_TABLE default\n";
    for(const auto& node : mesh.get_nodes()) {
        int bitmask = 0;
        bitmask |= static_cast<int>(node.flag) & 0x07;      // bits 0-2: flag
        if(node.boundary) bitmask |= 0x08;                  // bit 3: boundary
        if(node.constrained) bitmask |= 0x10;               // bit 4: constrained
        out << bitmask << "\n";
    }

    out.close();

    // Mostrar estadísticas
    std::cout << "\n[VTK] Mesh exported to " << filename << "\n";
    std::cout << "  - Nodes: " << mesh.num_nodes() << "\n";
    std::cout << "  - Quads: " << mesh.num_quads() << "\n";
    std::cout << "  - Node flag distribution:\n";
    for(const auto& [flag, count] : flag_counts) {
        std::string flag_name;
        switch(flag) {
            case NODE_PROJECTED:  flag_name = "PROJECTED"; break;
            case NODE_CRITICAL:   flag_name = "CRITICAL"; break;
            case NODE_INTERIOR:   flag_name = "INTERIOR"; break;
            case NODE_AXIS:       flag_name = "AXIS"; break;
            case NODE_EXTERNAL:   flag_name = "EXTERNAL"; break;
            case NODE_SUBDIVIDED: flag_name = "SUBDIVIDED"; break;
            case NODE_RING:       flag_name = "RING"; break;
            case NODE_CENTER:     flag_name = "CENTER"; break;
            default:              flag_name = "UNKNOWN"; break;
        }
        std::cout << "    * " << flag_name << ": " << count 
                  << " (" << (100.0 * count / mesh.num_nodes()) << "%)\n";
    }
    std::cout << "  - Boundary nodes: " << boundary_count << "\n";
    std::cout << "  - Constrained nodes: " << constrained_count << "\n";
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


// Export quads from a set of nodes and quads (SubdivisionResult compatible)
inline void export_nodes_and_quads_to_vtk(
    const std::vector<Node2D>& nodes,
    const std::vector<Quad>& quads,
    const std::string& filename
)
{
    std::ofstream file(filename);
    if(!file.is_open())
        throw std::runtime_error("Cannot open file: " + filename);

    file << "# vtk DataFile Version 3.0\n";
    file << "SubdivisionResult quads\n";
    file << "ASCII\n";
    file << "DATASET UNSTRUCTURED_GRID\n";

    // -------------------------
    // POINTS
    // -------------------------
    file << "POINTS " << nodes.size() << " float\n";
    for(const auto& n : nodes)
        file << n.x.x << " " << n.x.y << " 0.0\n";

    // -------------------------
    // CELLS
    // -------------------------
    size_t num_cells = quads.size();
    file << "CELLS " << num_cells << " " << num_cells*5 << "\n"; // 4 nodes + 1 per quad
    for(const auto& q : quads)
        file << "4 " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << "\n";

    // -------------------------
    // CELL TYPES
    // -------------------------
    file << "CELL_TYPES " << num_cells << "\n";
    for(size_t i=0; i<num_cells; ++i)
        file << "9\n"; // VTK_QUAD

    // -------------------------
    // POINT DATA (flags opcional)
    // -------------------------
    file << "POINT_DATA " << nodes.size() << "\n";
    file << "SCALARS node_flags int 1\n";
    file << "LOOKUP_TABLE default\n";
    for(const auto& n : nodes)
        file << static_cast<int>(n.flag) << "\n";

    file.close();
}


} // namespace mesh_adapt
