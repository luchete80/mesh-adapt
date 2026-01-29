#include <iostream>
#include <iomanip>
#include "mesh_adapt/core/Mesh2D.hpp"
#include "mesh_adapt/core/Mesh2DUtils.hpp"
#include "mesh_adapt/geometry/Polyline2D.hpp"
#include "mesh_adapt/boundary/Boundary2D.hpp"
#include "mesh_adapt/geometry/PolylineUtils.hpp"

#include "mesh_adapt/geometry/ContourUtils.hpp"
#include "mesh_adapt/geometry/PolygonUtils.hpp"

using namespace mesh_adapt;

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
    std::vector<mesh_adapt::Vec2> node_positions = mesh.get_node_positions();  
    
    for(size_t i = 0; i < node_positions.size(); i++) {  
        std::cout << "     node[" << i << "] = " << node_positions[i] << "\n";
    }
    
    // ============================================
    // 2. Create geometric contour
    // ============================================
    std::cout << "\n2. Creating contour...\n";
    
    //// EXPLICIT BOUNDARY
    
    mesh_adapt::Mesh2D mesh_cont;
    mesh_cont.add_node(0.0, 0.0);
    mesh_cont.add_node(1.0, 0.0);
    mesh_cont.add_node(1.0, 1.0);
    mesh_cont.add_node(0.0, 1.0);
    mesh_cont.add_quad(0, 1, 2, 3);
    
    mesh_adapt::Polyline2D boundary_poly = mesh_adapt::build_polyline_from_mesh(mesh_cont);

    std::cout << "Boundary polyline points:\n";
    for(const auto& p : boundary_poly.get_points()) {
        std::cout << p << "\n";
    }
    //std::vector<Vec2> 
    auto contour_pts = boundary_poly.get_points();

    

    // /// KNOWN BOUNDARY /////
    // std::vector<mesh_adapt::Vec2> contour_pts = {
        // {0.0, 0.0},
        // {2.0, 0.0},
        // {2.0, 2.0},
        // {0.0, 2.0},
        // {0.0, 0.0}
    // };
    // mesh_adapt::Polyline2D contour(contour_pts);  
    
    
    // std::cout << "   Contour points: " << contour_pts.size() << "\n";
    // for(size_t i = 0; i < contour_pts.size(); i++) {
        // std::cout << "     contour[" << i << "] = " << contour_pts[i] << "\n";
    // }
    
    //////////////////////////////////////////
    mesh_adapt::Polyline2D contour(contour_pts);
    
    // ============================================
    // 3. Test geometric operations
    // ============================================
    std::cout << "\n3. Testing geometric operations...\n";
    mesh_adapt::Vec2 test_point = {1.0, 1.0};  
    double dist = contour.distance(test_point);  
    mesh_adapt::Vec2 proj = contour.project(test_point);  
    
    std::cout << "   Test point: " << test_point << "\n";
    std::cout << "   Distance to contour: " << dist << "\n";
    std::cout << "   Projection onto contour: " << proj << "\n";
    
    // ============================================
    // 4. BOUNDARY LAYER
    // ============================================
    std::cout << "\n4. Analyzing boundary layer...\n";
    mesh_adapt::Boundary2D boundary(contour);  
    
    boundary.find_boundary_nodes_from_quads(node_positions, mesh.get_quads());
    
    // ✅ BUEN uso de auto (referencia const, tipo largo)
    const auto& boundary_nodes = boundary.get_boundary_nodes();
    std::cout << "   Boundary nodes found: " << boundary_nodes.size() << "\n";
    
    if(boundary_nodes.empty()) {
        std::cout << "   WARNING: No boundary nodes found!\n";
    } else {
        std::cout << "\n   Boundary node details:\n";

        for(const auto& bn : boundary_nodes) {
            mesh_adapt::Vec2 pos = node_positions[bn.mesh_idx];  
            mesh_adapt::Vec2 projected = contour.project(pos);   
            double dist_to_contour = contour.distance(pos);      
            
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
    mesh_adapt::ProjectionParams params(0.25); 
    
    std::cout << "   Projection parameters:\n";
    std::cout << "     SL = " << params.SL << "\n";
    std::cout << "     rmin = " << params.rmin << "\n";
    std::cout << "     rmax = " << params.rmax << "\n";
    
    std::vector<mesh_adapt::Vec2> proj_nodes = 
        boundary.generate_projected_nodes(node_positions, params); 
    
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
    std::vector<mesh_adapt::Vec2> trans_points;     
    std::vector<size_t> trans_to_global;             
    
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
    
    
    
    /////////////////////////////////////////////////////////////////////
    /////////////////////// CONE EXAMPLE ////////////////////////////////
    std::cout << "\n========================================\n";
    std::cout << "   CONE EXAMPLE \n";
    std::cout << "========================================\n";    
    
    double SL = 0.25;
    
    std::vector<mesh_adapt::Vec2> cone_pts = {
    {0.0, 0.0},   // axis bottom
    {2.0, 0.0},   // base radius
    {1.0, 4.0},   // cone side tip
    {0.0, 4.0},   // axis top
    {0.0, 0.0}    // close loop
    };
    mesh_adapt::Polyline2D cone_contour(cone_pts);

    std::cout << "\n[1] Cone contour points:\n";
    for(size_t i = 0; i < cone_pts.size(); ++i) {
        std::cout << "   contour[" << i << "] = " << cone_pts[i] << "\n";
    }

    // ------------------------------------------------------------
    // 2. Compute bounding box
    // ------------------------------------------------------------
    auto bbox = compute_bbox(cone_pts);

    std::cout << "\n[2] Bounding box:\n";
    std::cout << "   xmin = " << bbox.xmin << "\n";
    std::cout << "   xmax = " << bbox.xmax << "\n";
    std::cout << "   ymin = " << bbox.ymin << "\n";
    std::cout << "   ymax = " << bbox.ymax << "\n";

    // padding extra
    double pad = 0.5;

    // ------------------------------------------------------------
    // 3. Generate structured background mesh
    // ------------------------------------------------------------
    std::cout << "\n[3] Generating structured background grid...\n";

    Mesh2D mesh_bg = generate_structured_grid(
        bbox.xmin - pad,
        bbox.xmax + pad,
        bbox.ymin - pad,
        bbox.ymax + pad,
        SL
    );    

    std::cout << "   Background mesh created:\n";
    std::cout << "     Nodes = " << mesh_bg.num_nodes() << "\n";
    std::cout << "     Quads = " << mesh_bg.num_quads() << "\n";

    // Print a few sample nodes
    std::cout << "\n   Sample background nodes:\n";
    for(size_t i = 0; i < std::min<size_t>(10, mesh_bg.num_nodes()); ++i) {
        std::cout << "     node[" << i << "] = " << mesh_bg.node(i) << "\n";
    }


    // ------------------------------------------------------------
    // 4. Filter nodes inside contour
    // ------------------------------------------------------------
    std::cout << "\n[4] Filtering nodes inside cone contour...\n";

    // 4. Keep only inside nodes
    Mesh2D inside_mesh = filter_nodes_inside_contour(mesh_bg, cone_contour);

    std::cout << "   Inside mesh result:\n";
    std::cout << "     Nodes kept = " << inside_mesh.num_nodes() << "\n";
    std::cout << "     Quads kept = " << inside_mesh.num_quads() << "\n";

    std::cout << "   Removed nodes = "
              << mesh_bg.num_nodes() - inside_mesh.num_nodes()
              << "\n";

    // Print a few inside nodes
    std::cout << "\n   Sample inside nodes:\n";
    for(size_t i = 0; i < std::min<size_t>(10, inside_mesh.num_nodes()); ++i) {
        std::cout << "     inside_node[" << i << "] = " << inside_mesh.node(i) << "\n";
    }

    // ------------------------------------------------------------
    // 5. Remove nodes too close to contour (boundary band)
    // ------------------------------------------------------------
    std::cout << "\n[5] Removing nodes near contour (distance < SL)...\n";
    std::cout << "   Threshold SL = " << SL << "\n";
              
    // 5. Remove nodes too close to contour (future)
    Mesh2D band_mesh = remove_nodes_near_contour(inside_mesh, cone_contour, SL);


    std::cout << "   Band mesh result:\n";
    std::cout << "     Nodes kept = " << band_mesh.num_nodes() << "\n";
    std::cout << "     Quads kept = " << band_mesh.num_quads() << "\n";

    std::cout << "   Removed near-boundary nodes = "
              << inside_mesh.num_nodes() - band_mesh.num_nodes()
              << "\n";

    // ------------------------------------------------------------
    // 6. Debug: print some removed nodes (optional)
    // ------------------------------------------------------------
    std::cout << "\n[6] Debug check: nodes close to boundary\n";

    int printed = 0;
    for(size_t i = 0; i < inside_mesh.num_nodes(); ++i)
    {
        mesh_adapt::Vec2 p = inside_mesh.node(i);
        double d = cone_contour.distance(p);

        if(d < SL) {
            std::cout << "   removed candidate: p=" << p
                      << " dist=" << d << "\n";
            printed++;
        }

        if(printed > 10) break;
    }

    std::cout << "\n========================================\n";
    std::cout << "   CONE EXAMPLE FINISHED\n";
    std::cout << "========================================\n";    
    
    return 0;
}
