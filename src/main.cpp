#include <iostream>
#include <iomanip>
#include <string>
#include "mesh_adapt/core/Mesh2D.hpp"
#include "mesh_adapt/core/Mesh2DUtils.hpp"
#include "mesh_adapt/geometry/Polyline2D.hpp"
#include "mesh_adapt/boundary/Boundary2D.hpp"
#include "mesh_adapt/boundary/TransitionPatch2D.hpp"
#include "mesh_adapt/boundary/Delaunay2D.hpp"
#include "mesh_adapt/geometry/PolylineUtils.hpp"
#include "mesh_adapt/geometry/ContourUtils.hpp"
#include "mesh_adapt/geometry/PolygonUtils.hpp"
#include "mesh_adapt/io/VTKReader2D.hpp"
#include "mesh_adapt/io/VTKWriter2D.hpp"
#include "mesh_adapt/subdivision/MeshToSub.hpp" 
#include "mesh_adapt/subdivision/QuadRefiner.hpp"
#include "mesh_adapt/smoothing/LaplacianSmoother2D.hpp"

using namespace mesh_adapt;

void print_usage(const std::string& program_name) {
    std::cout << "\nUsage: " << program_name << " <input_vtk_file> [element_size] [output_prefix]\n\n";
    std::cout << "Arguments:\n";
    std::cout << "  input_vtk_file : VTK file with the mesh to process\n";
    std::cout << "  element_size   : Target element size (default: 0.25)\n";
    std::cout << "  output_prefix  : Prefix for output files (default: output)\n\n";
    std::cout << "Examples:\n";
    std::cout << "  " << program_name << " cylinder.vtk\n";
    std::cout << "  " << program_name << " geometry.vtk 0.1\n";
    std::cout << "  " << program_name << " mesh.vtk 0.15 results/my_mesh\n";
}

int main(int argc, char* argv[]) {
    
    // Verificar argumentos
    if (argc < 2) {
        std::cerr << "Error: Missing input file\n";
        print_usage(argv[0]);
        return 1;
    }
    
    std::string input_vtk = argv[1];
    double SL = (argc > 2) ? std::stod(argv[2]) : 5.0e-4;
    std::string output_prefix = (argc > 3) ? argv[3] : "output";
    
    /////////////////////////////////////////////////////////////////////
    /////////////////////// VTK FILE PROCESSING /////////////////////////
    std::cout << "\n========================================\n";
    std::cout << "   PROCESSING VTK FILE: " << input_vtk << "\n";
    std::cout << "========================================\n";    
    
    std::cout << "Parameters:\n";
    std::cout << "  Input file: " << input_vtk << "\n";
    std::cout << "  Element size (SL): " << SL << "\n";
    std::cout << "  Output prefix: " << output_prefix << "\n";

    try {
        // ------------------------------------------------------------
        // 1. Extract contour from VTK file
        // ------------------------------------------------------------
        std::cout << "\n[1] Extracting contour from VTK file...\n";
        
        mesh_adapt::Polyline2D contour = extract_contour_from_vtk(input_vtk);
        
        // Get the contour points
        const auto& contour_pts = contour.get_points();
        
        std::cout << "   Contour points extracted: " << contour_pts.size() << "\n";
        
        // Export the extracted contour for verification
        export_polyline_to_vtk(contour, output_prefix + "_extracted_contour.vtk");

        // ------------------------------------------------------------
        // 2. Compute bounding box
        // ------------------------------------------------------------
        auto bbox = compute_bbox(contour_pts);

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

        // ------------------------------------------------------------
        // 4. Filter nodes inside contour
        // ------------------------------------------------------------
        std::cout << "\n[4] Filtering nodes inside contour...\n";

        Mesh2D inside_mesh = filter_nodes_inside_contour(mesh_bg, contour);

        std::cout << "   Inside mesh result:\n";
        std::cout << "     Nodes kept = " << inside_mesh.num_nodes() << "\n";
        std::cout << "     Quads kept = " << inside_mesh.num_quads() << "\n";

        std::cout << "   Removed nodes = "
                  << mesh_bg.num_nodes() - inside_mesh.num_nodes()
                  << "\n";

        // ------------------------------------------------------------
        // 5. Remove nodes too close to contour (boundary band)
        // ------------------------------------------------------------
        std::cout << "\n[5] Removing nodes near contour (distance < SL)...\n";
        std::cout << "   Threshold SL = " << SL << "\n";
                  
        Mesh2D band_mesh = remove_nodes_near_contour(inside_mesh, contour, SL);
        
        std::cout << "   Band mesh result:\n";
        std::cout << "     Nodes kept = " << band_mesh.num_nodes() << "\n";
        std::cout << "     Quads kept = " << band_mesh.num_quads() << "\n";

        std::cout << "   Removed near-boundary nodes = "
                  << inside_mesh.num_nodes() - band_mesh.num_nodes()
                  << "\n";

        // ------------------------------------------------------------
        // 6. Export intermediate meshes
        // ------------------------------------------------------------
        export_mesh_to_vtk(mesh_bg, output_prefix + "_background.vtk");
        export_mesh_to_vtk(inside_mesh, output_prefix + "_inside.vtk");
        export_mesh_to_vtk(band_mesh, output_prefix + "_band.vtk");
        
        // ------------------------------------------------------------
        // 7. Generate projected nodes (Martins projection)
        // ------------------------------------------------------------
        std::cout << "\n[7] Generating projected nodes from band boundary...\n";

        std::vector<int> ring_nodes = band_mesh.find_ordered_boundary_nodes();
        std::cout << "   Ring boundary nodes = " << ring_nodes.size() << "\n";

        std::vector<Vec2> proj_nodes;
        proj_nodes.reserve(ring_nodes.size());
        double rmin = 0.5 * SL;

        for(int gid : ring_nodes) {
            Vec2 p = band_mesh.node(gid);
            Vec2 q = contour.project(p);

            // Filtrar projected nodes demasiado cercanos
            bool too_close = false;
            for(const auto& pk : proj_nodes) {
                if((pk - q).norm() < rmin) {
                    too_close = true;
                    break;
                }
            }

            if(!too_close) {
                proj_nodes.push_back(q);
            }
        }

        std::cout << "   Projected nodes kept = " << proj_nodes.size() << "\n";
        export_points_vtk(proj_nodes, output_prefix + "_proj_nodes.vtk");

        std::cout << "\n========================================\n";
        std::cout << "   BUILDING TRANSITION PATCH\n";
        std::cout << "========================================\n";    

        // ------------------------------------------------------------
        // 8. Build transition patch
        // ------------------------------------------------------------
        TransitionPatch2D patch = build_transition_patch_from_band(
            band_mesh,
            contour,
            SL
        );
        
        Polyline2D ring_polyline = make_polyline(patch, patch.ring_loop); 
        Polyline2D proj_polyline = make_polyline(patch, patch.proj_loop); 
        
        // Export polylines
        export_polyline_to_vtk(contour, output_prefix + "_original_contour.vtk");
        export_polyline_to_vtk(ring_polyline, output_prefix + "_ring_polyline.vtk");
        export_polyline_to_vtk(proj_polyline, output_prefix + "_proj_polyline.vtk");

        // ------------------------------------------------------------
        // 9. Delaunay triangulation
        // ------------------------------------------------------------
        Delaunay2D dt;
        dt.build_from_two_polylines(proj_polyline, ring_polyline);
        dt.triangulate();

        // Filtrar triángulos en la región entre las polylines
        std::vector<std::array<int,3>> filtered;
        
        for(const auto& t : dt.get_triangles()) {
            Vec2 c = (dt.get_points()[t[0]] + 
                      dt.get_points()[t[1]] + 
                      dt.get_points()[t[2]]) / 3.0;
            
            if(point_in_polygon(c, proj_polyline.pts) &&
              !point_in_polygon(c, ring_polyline.pts)) {
                filtered.push_back(t);
            }
        }

        patch.triangles = filtered;
        export_triangles_to_vtk(dt.get_points(), patch.triangles,
                                output_prefix + "_patch_tris.vtk");
        
        debug_print_patch_nodes(patch);

        // ------------------------------------------------------------
        // 10. Merge triangles to quads
        // ------------------------------------------------------------
        merge_triangles_to_quads(patch);
        
        export_triangles_to_vtk(dt.get_points(), patch.tris_left, 
                                output_prefix + "_patch_tris_left.vtk");
        export_quads_to_vtk(dt.get_points(), patch.quads, 
                            output_prefix + "_patch_quads.vtk");

        // ------------------------------------------------------------
        // 11. Subdivide triangles to quads
        // ------------------------------------------------------------
        subdivide_tris_to_quads(patch);
        
        std::vector<std::array<int,4>> all_patch_quads = patch.quads;
        all_patch_quads.insert(all_patch_quads.end(), 
                               patch.quads_fallback.begin(), 
                               patch.quads_fallback.end());

        export_quads_to_vtk(patch.points, all_patch_quads,
                            output_prefix + "_all_quads_patch.vtk");    

        // ------------------------------------------------------------
        // 12. Mesh to subdivision
        // ------------------------------------------------------------
        MeshToSub mesh2sub(band_mesh, patch);
        export_mesh_to_vtk(mesh2sub.mesh, output_prefix + "_mesh2sub.vtk");
        
        debug_mesh_to_sub(mesh2sub);
        
        std::map<Edge, EdgeInfo> edge_map = build_edge_map(mesh2sub);
        debug_edge_map(edge_map);
        debug_edges_vs_map(patch);
        
        // ------------------------------------------------------------
        // 13. Quad refinement
        // ------------------------------------------------------------
        QuadRefiner quad_refiner(mesh2sub.mesh.quads, edge_map, 
                                 mesh2sub.subdivided_edge_to_global);
        quad_refiner.refine_to_conform();

        SubdivisionResult result = quad_refiner.subdivide_quads_with_nodes(
            mesh2sub.mesh.get_nodes());
        
        // ------------------------------------------------------------
        // 14. Build final mesh
        // ------------------------------------------------------------
        Mesh2D resmesh = build_final_mesh(result);
        export_mesh_to_vtk(resmesh, output_prefix + "_remeshed.vtk");
        
        export_nodes_and_quads_to_vtk(
            result.nodes,
            result.new_quads,
            output_prefix + "_subdivision_results.vtk"
        );

        // ------------------------------------------------------------
        // 15. Laplacian smoothing
        // ------------------------------------------------------------
        std::vector<int> boundary = resmesh.find_ordered_boundary_nodes();
        std::cout << "\nBoundary nodes: " << boundary.size() << "\n";
        
        std::vector<char> fixed(resmesh.nodes.size(), 0);
        for(int i : boundary) {
            fixed[i] = 1;
        }

        LaplacianSmoother2D smoother(
            resmesh.nodes,
            resmesh.quads,
            fixed
        );
        smoother.smooth();

        export_nodes_and_quads_to_vtk(
            smoother.nodes_,
            smoother.quads_,
            output_prefix + "_smoothed.vtk"
        ); 
        
        std::cout << "\n========================================\n";
        std::cout << "   PROCESS COMPLETED SUCCESSFULLY!\n";
        std::cout << "========================================\n";
        
        std::cout << "\nOutput files created with prefix: " << output_prefix << "\n";
        std::cout << "  - " << output_prefix << "_extracted_contour.vtk\n";
        std::cout << "  - " << output_prefix << "_remeshed.vtk (final remeshed)\n";
        std::cout << "  - " << output_prefix << "_smoothed.vtk (final smoothed)\n";
        std::cout << "  - Various intermediate files for debugging\n";
        
    } catch (const std::exception& e) {
        std::cerr << "\nError: " << e.what() << "\n";
        return 1;
    }
    
    return 0;
}
