#include <iostream>
#include <iomanip>
#include "mesh_adapt/core/Mesh2D.hpp"
#include "mesh_adapt/core/Mesh2DUtils.hpp"
#include "mesh_adapt/geometry/Polyline2D.hpp"
#include "mesh_adapt/boundary/Boundary2D.hpp"
#include "mesh_adapt/boundary/TransitionPatch2D.hpp"
#include "mesh_adapt/boundary/Delaunay2D.hpp"
#include "mesh_adapt/geometry/PolylineUtils.hpp"

#include "mesh_adapt/geometry/ContourUtils.hpp"
#include "mesh_adapt/geometry/PolygonUtils.hpp"
#include "mesh_adapt/io/VTKWriter2D.hpp"


using namespace mesh_adapt;

int main() {

    
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

    //~ auto band_boundary = ///std::vector<int> 
        //~ find_boundary_nodes(band_mesh);
        

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


    export_mesh_to_vtk(mesh_bg, "background.vtk");
    export_mesh_to_vtk(inside_mesh,     "inside.vtk");
    export_mesh_to_vtk(band_mesh,       "band.vtk");
    
            
    // ------------------------------------------------------------
    // 7. Generate projected nodes (Martins projection)
    //    PROYECTAR SOLO EL BORDE DEL BAND MESH
    // ------------------------------------------------------------
    std::cout << "\n[7] Generating projected nodes from band boundary...\n";

    // 1. Boundary ring del band mesh
    std::vector<int> ring_nodes =
        band_mesh.find_ordered_boundary_nodes();

    std::cout << "   Ring boundary nodes = "
              << ring_nodes.size() << "\n";

    // 2. Proyectarlos al contour geométrico
    std::vector<Vec2> proj_nodes;
    proj_nodes.reserve(ring_nodes.size());

    double rmin = 0.5 * SL;

    for(int gid : ring_nodes)
    {
        std::cout << "Ring node "<<gid<<std::endl;
        Vec2 p = band_mesh.node(gid);

        // proyección geométrica
        Vec2 q = cone_contour.project(p);

        // --------------------------------------------------------
        // Filtrar projected nodes demasiado cercanos
        // --------------------------------------------------------
        bool too_close = false;
        for(const auto& pk : proj_nodes)
        {
            if((pk - q).norm() < rmin)
            {
                too_close = true;
                break;
            }
        }

        if(too_close)
            continue;

        proj_nodes.push_back(q);
    }

    std::cout << "   Projected nodes kept = "
              << proj_nodes.size() << "\n";

    // 3. Exportar para debug
    export_points_vtk(proj_nodes, "proj_nodes.vtk");
    export_points_vtk(cone_pts,   "cone_contour.vtk");


    std::cout << "\n========================================\n";
    std::cout << "   CONE EXAMPLE FINISHED\n";
    std::cout << "========================================\n";    


    TransitionPatch2D patch =
        build_transition_patch_from_band(
            band_mesh,
            cone_contour,
            SL
        );
    
    Polyline2D ring_polyline = make_polyline(patch,patch.ring_loop); 
    Polyline2D proj_polyline = make_polyline(patch,patch.proj_loop); 
    
    // ------------------------------------------------------------
    // 8. Debug: print proj_from_ring_gid mapping
    // ------------------------------------------------------------
    std::cout << "\n[8] Debug: Projected mapping (proj_from_ring_gid)\n";
    std::cout << "----------------------------------------\n";

    std::cout << "   ring_polyline size = "
              << ring_polyline.get_points().size() << "\n";

    std::cout << "   proj_polyline size = "
              << proj_polyline.get_points().size() << "\n";

    // std::cout << "   proj_from_ring_gid size = "
              // << patch.proj_from_ring_gid.size() << "\n\n";

    // // Print first N correspondences
    size_t Nprint = std::min<size_t>(20, patch.proj_from_ring_gid.size());

    for(size_t i = 0; i < Nprint; ++i)
    {
        int gid = patch.proj_from_ring_gid[i];

        Vec2 ring_p = ring_polyline.get_points()[i];
        Vec2 proj_q = proj_polyline.get_points()[i];

        std::cout << "   proj[" << i << "] = " << proj_q
                  << "  comes from ring_gid = " << gid
                  << "  ring_point = " << ring_p
                  << "\n";
    }


      export_polyline_to_vtk(cone_contour, "poly_contour.vtk");

      export_polyline_to_vtk(ring_polyline, "poly_ring.vtk");

      export_polyline_to_vtk(proj_polyline, "poly_proj.vtk");

    std::cout << "----------------------------------------\n";

    Delaunay2D dt;
    dt.build_from_two_polylines(proj_polyline,ring_polyline); //outer, inner

    dt.triangulate();


    std::vector<std::array<int,3>> filtered;
    
    for(const auto& t : dt.get_triangles())
    {

        Vec2 c = (dt.get_points()[t[0]] + dt.get_points()[t[1]] + dt.get_points()[t[2]]) / 3.0;

        if( point_in_polygon(c, proj_polyline.pts) &&
           !point_in_polygon(c, ring_polyline.pts) )
        {
            filtered.push_back(t);
        }
    }

    patch.triangles = filtered;

    //patch.triangles = dt.get_triangles();
    
    //dt.get_points is the same that patch_points
    export_triangles_to_vtk(dt.get_points()/*patch.points*/, patch.triangles,
                            "patch_tris.vtk");
                            
                            
    debug_print_patch_nodes(patch);

    // Merge a quads
    merge_triangles_to_quads(patch);

    // Export
    export_triangles_to_vtk(dt.get_points(), patch.tris_left, "patch_tris_left.vtk");
    export_quads_to_vtk(dt.get_points(), patch.quads, "patch_quads.vtk");

    debug_print_patch_nodes(patch);
                            
    return 0;
}
