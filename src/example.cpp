#include <iostream>
#include <iomanip>
#include "mesh_adapt/core/Mesh2D.hpp"
#include "mesh_adapt/geometry/Polyline2D.hpp"
#include "mesh_adapt/boundary/Boundary2D.hpp"

int main() {
    std::cout << "========================================\n";
    std::cout << "  Mesh Adapt - Boundary2D Test\n";
    std::cout << "========================================\n\n";
    
    // ============================================
    // 1. Create mesh (single quad)
    // ============================================
    std::cout << "1. Creating mesh...\n";
    mesh_adapt::Mesh2D mesh;  // ✅ tipo claro
    mesh.add_node(0.0, 0.0);
    mesh.add_node(1.0, 0.0);
    mesh.add_node(1.0, 1.0);
    mesh.add_node(0.0, 1.0);
    mesh.add_quad(0, 1, 2, 3);
    
    std::cout << "   Mesh nodes: " << mesh.num_nodes() << "\n";
    std::cout << "   Mesh quads: " << mesh.num_quads() << "\n";
    
    // Mostrar nodos
    std::cout << "\n   Node positions:\n";
    std::vector<mesh_adapt::Vec2> node_positions = mesh.get_node_positions();  // ✅ explícito
    
    for(size_t i = 0; i < node_positions.size(); i++) {  // ✅ size_t explícito
        std::cout << "     node[" << i << "] = " << node_positions[i] << "\n";
    }
    
    // ============================================
    // 2. Create geometric contour
    // ============================================
    std::cout << "\n2. Creating contour...\n";
    std::vector<mesh_adapt::Vec2> contour_pts = {
        {0.0, 0.0},
        {2.0, 0.0},
        {2.0, 2.0},
        {0.0, 2.0},
        {0.0, 0.0}
    };
    mesh_adapt::Polyline2D contour(contour_pts);  // ✅ tipo claro
    
    std::cout << "   Contour points: " << contour_pts.size() << "\n";
    for(size_t i = 0; i < contour_pts.size(); i++) {
        std::cout << "     contour[" << i << "] = " << contour_pts[i] << "\n";
    }
    
    // ============================================
    // 3. Test geometric operations
    // ============================================
    std::cout << "\n3. Testing geometric operations...\n";
    mesh_adapt::Vec2 test_point = {1.0, 1.0};  // ✅ tipo claro
    double dist = contour.distance(test_point);  // ✅ double explícito
    mesh_adapt::Vec2 proj = contour.project(test_point);  // ✅ tipo claro
    
    std::cout << "   Test point: " << test_point << "\n";
    std::cout << "   Distance to contour: " << dist << "\n";
    std::cout << "   Projection onto contour: " << proj << "\n";
    
    // ============================================
    // 4. BOUNDARY LAYER
    // ============================================
    std::cout << "\n4. Analyzing boundary layer...\n";
    mesh_adapt::Boundary2D boundary(contour);  // ✅ tipo claro
    
    boundary.find_boundary_nodes_from_quads(node_positions, mesh.get_quads());
    
    // ✅ BUEN uso de auto (referencia const, tipo largo)
    const auto& boundary_nodes = boundary.get_boundary_nodes();
    std::cout << "   Boundary nodes found: " << boundary_nodes.size() << "\n";
    
    if(boundary_nodes.empty()) {
        std::cout << "   WARNING: No boundary nodes found!\n";
    } else {
        std::cout << "\n   Boundary node details:\n";
        
        // ✅ BUEN uso de auto en range-for
        for(const auto& bn : boundary_nodes) {
            mesh_adapt::Vec2 pos = node_positions[bn.mesh_idx];  // ✅ explícito
            mesh_adapt::Vec2 projected = contour.project(pos);   // ✅ explícito
            double dist_to_contour = contour.distance(pos);      // ✅ explícito
            
            std::cout << "     mesh_idx=" << bn.mesh_idx
                      << " pos=" << pos
                      << " -> proj=" << projected
                      << " (dist=" << std::fixed << std::setprecision(4) 
                      << dist_to_contour << ")\n";
        }
    }
    
    // ============================================
    // 5. Generate projected nodes
    // ============================================
    std::cout << "\n5. Generating projected nodes (Martins)...\n";
    mesh_adapt::ProjectionParams params(0.25);  // ✅ tipo es documentación
    
    std::cout << "   Projection parameters:\n";
    std::cout << "     SL = " << params.SL << "\n";
    std::cout << "     rmin = " << params.rmin << "\n";
    std::cout << "     rmax = " << params.rmax << "\n";
    
    std::vector<mesh_adapt::Vec2> proj_nodes = 
        boundary.generate_projected_nodes(node_positions, params);  // ✅ explícito
    
    std::cout << "\n   Projected nodes: " << proj_nodes.size() << "\n";
    
    if(proj_nodes.empty()) {
        std::cout << "   INFO: No projected nodes generated\n";
    } else {
        std::cout << "\n   Projected node coordinates:\n";
        for(size_t i = 0; i < proj_nodes.size(); i++) {
            std::cout << "     proj_node[" << i << "] = " << proj_nodes[i] << "\n";
        }
    }
    
    // ============================================
    // 6. Build transition points
    // ============================================
    std::cout << "\n6. Building transition point set...\n";
    std::vector<mesh_adapt::Vec2> trans_points;      // ✅ explícito
    std::vector<size_t> trans_to_global;             // ✅ size_t explícito
    
    boundary.build_transition_points(
        node_positions,
        proj_nodes,
        trans_points,
        trans_to_global
    );
    
    std::cout << "   Transition points: " << trans_points.size() << "\n";
    std::cout << "   Composition:\n";
    std::cout << "     - Boundary nodes: " << boundary_nodes.size() << "\n";
    std::cout << "     - Projected nodes: " << proj_nodes.size() << "\n";
    std::cout << "     - Total: " << trans_points.size() << "\n";
    
    if(!trans_points.empty()) {
        std::cout << "\n   Transition point mapping:\n";
        for(size_t i = 0; i < trans_points.size(); i++) {
            std::cout << "     trans[" << i << "] -> global[" 
                      << trans_to_global[i] << "] = " 
                      << trans_points[i] << "\n";
        }
    }
    
    // ============================================
    // Summary
    // ============================================
    std::cout << "\n========================================\n";
    std::cout << "  Summary\n";
    std::cout << "========================================\n";
    std::cout << "  Mesh nodes:        " << mesh.num_nodes() << "\n";
    std::cout << "  Mesh quads:        " << mesh.num_quads() << "\n";
    std::cout << "  Boundary nodes:    " << boundary_nodes.size() << "\n";
    std::cout << "  Projected nodes:   " << proj_nodes.size() << "\n";
    std::cout << "  Transition points: " << trans_points.size() << "\n";
    std::cout << "========================================\n";
    
    return 0;
}
