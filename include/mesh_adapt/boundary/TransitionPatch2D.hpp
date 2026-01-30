#pragma once
#include "mesh_adapt/core/Mesh2D.hpp"
#include "mesh_adapt/geometry/Polyline2D.hpp"

namespace mesh_adapt {

struct TransitionPatch2D
{
    // puntos locales (boundary + projected + críticos)
    std::vector<Vec2> points;

    // mapping: local index → global mesh node id
    // si es -1 → es nodo nuevo projected o crítico
    std::vector<int> local_to_global;

    // triangulación local
    std::vector<std::array<int,3>> triangles;

    // quads generados localmente
    std::vector<std::array<int,4>> quads;

    // flags por nodo
    std::vector<NodeFlag> flags;

    // NEW: mapping projected -> ring node
    std::vector<int> proj_from_ring_gid;

    // polylines locales
    Polyline2D ring_polyline;
    Polyline2D proj_polyline;
};

//~ TransitionPatch2D build_transition_patch_from_band(
    //~ const Mesh2D& band_mesh,
    //~ const std::vector<Vec2>& proj_nodes
//~ )
//~ {
    //~ TransitionPatch2D patch;

    //~ patch.points.clear();
    //~ patch.local_to_global.clear();
    //~ patch.flags.clear();
    //~ patch.ring_polyline.pts.clear();
    //~ patch.proj_polyline.pts.clear();

    //~ // ------------------------------------------------------------
    //~ // 1. Boundary ring del band mesh
    //~ // ------------------------------------------------------------
    //~ std::vector<int> boundary_ids =
        //~ band_mesh.find_boundary_nodes();

    //~ std::cout << "[TransitionPatch] Boundary ring nodes = "
              //~ << boundary_ids.size() << "\n";

    //~ for(int gid : boundary_ids)
    //~ {
        //~ Vec2 p = band_mesh.node(gid);
        //~ patch.points.push_back(p);
        //~ patch.local_to_global.push_back(gid);
        //~ patch.flags.push_back(NodeFlag::NODE_RING);
        //~ patch.ring_polyline.pts.push_back(p);  // directamente el punto
    //~ }

    //~ // ------------------------------------------------------------
    //~ // 2. Projected contour nodes (nuevos)
    //~ // ------------------------------------------------------------
    //~ std::cout << "[TransitionPatch] Projected nodes = "
              //~ << proj_nodes.size() << "\n";

    //~ for(const Vec2& q : proj_nodes)
    //~ {
        //~ patch.points.push_back(q);
        //~ patch.local_to_global.push_back(-1);  // nuevo nodo
        //~ patch.flags.push_back(NodeFlag::NODE_EXTERNAL);
        //~ patch.proj_polyline.pts.push_back(q);  // directamente el punto
    //~ }

    //~ return patch;
//~ }
TransitionPatch2D build_transition_patch_from_band(
    const Mesh2D& band_mesh,
    const Polyline2D& contour,
    double SL
)
{
    TransitionPatch2D patch;

    // ------------------------------------------------------------
    // 1. Ordered ring boundary nodes
    // ------------------------------------------------------------
    auto ring_ids = band_mesh.find_ordered_boundary_nodes();

    std::cout << "[TransitionPatch] Ring nodes = "
              << ring_ids.size() << "\n";

    std::vector<Vec2> ring_pts;
    std::vector<Vec2> proj_pts;

    ring_pts.reserve(ring_ids.size());
    proj_pts.reserve(ring_ids.size());

    patch.proj_from_ring_gid.clear();

    // minimum spacing filter
    double rmin = 0.5 * SL;

    // ------------------------------------------------------------
    // 2. Project ring nodes onto contour (ordered)
    // ------------------------------------------------------------
    for(int gid : ring_ids)
    {
        Vec2 p = band_mesh.node(gid);
        Vec2 q = contour.project(p);

        // ---- filter projected points too close ----
        bool too_close = false;

        for(const auto& prev : proj_pts)
        {
            if((prev - q).norm() < rmin)
            {
                too_close = true;
                break;
            }
        }

        if(too_close)
            continue;

        // ---- keep both ring + projected in sync ----
        ring_pts.push_back(p);
        proj_pts.push_back(q);

        patch.proj_from_ring_gid.push_back(gid);
    }

    std::cout << "[TransitionPatch] Projected kept = "
              << proj_pts.size() << "\n";

    // ------------------------------------------------------------
    // 3. Store as polylines
    // ------------------------------------------------------------
    patch.ring_polyline = Polyline2D(ring_pts);
    patch.proj_polyline = Polyline2D(proj_pts);

    return patch;
}



}
