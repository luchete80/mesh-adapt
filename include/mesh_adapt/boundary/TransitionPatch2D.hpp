#pragma once
#include "mesh_adapt/core/Mesh2D.hpp"
#include "mesh_adapt/geometry/Polyline2D.hpp"
#include "mesh_adapt/geometry/Edge.hpp" // to merge quads
#include <map> 

namespace mesh_adapt {
  
struct TransitionPatch2D
{
    // geometría local ÚNICA
    std::vector<Vec2> points;

    // mapeo local → global
    std::vector<int> local_to_global;

    std::vector<NodeFlag> flags;

    // conectividad
    std::vector<std::array<int,3>> triangles;
    std::vector<std::array<int,4>> quads;

    // loops (orden topológico)
    std::vector<int> ring_loop;   // local ids
    std::vector<int> proj_loop;   // local ids

    // relación projected ↔ ring
    std::vector<int> proj_from_ring_gid;
    // NUEVO: local projected lid -> index en proj_loop
    // size == points.size(), -1 si no es projected
    std::vector<int> proj_lid_to_index;
    
};

Polyline2D make_polyline(
    const TransitionPatch2D& patch,
    const std::vector<int>& loop
)
{
    Polyline2D pl;
    for(int lid : loop)
        pl.pts.push_back(patch.points[lid]);
    return pl;
}



// TransitionPatch2D build_transition_patch_from_band(
    // const Mesh2D& band_mesh,
    // const Polyline2D& contour,
    // double SL
// )
// {
    // TransitionPatch2D patch;

    // // ------------------------------------------------------------
    // // 1. Ordered ring boundary nodes
    // // ------------------------------------------------------------
    // auto ring_ids = band_mesh.find_ordered_boundary_nodes();

    // std::cout << "[TransitionPatch] Ring nodes = "
              // << ring_ids.size() << "\n";

    // std::vector<Vec2> ring_pts;
    // std::vector<Vec2> proj_pts;

    // ring_pts.reserve(ring_ids.size());
    // proj_pts.reserve(ring_ids.size());

    // patch.proj_from_ring_gid.clear();

    // // minimum spacing filter
    // double rmin = 0.5 * SL;

    // // ------------------------------------------------------------
    // // 2. Project ring nodes onto contour (ordered)
    // // ------------------------------------------------------------
    // for(int gid : ring_ids)
    // {
        // Vec2 p = band_mesh.node(gid);
        // Vec2 q = contour.project(p);

        // // ---- filter projected points too close ----
        // bool too_close = false;

        // for(const auto& prev : proj_pts)
        // {
            // if((prev - q).norm() < rmin)
            // {
                // too_close = true;
                // break;
            // }
        // }

        // if(too_close)
            // continue;

        // // ---- keep both ring + projected in sync ----
        // ring_pts.push_back(p);
        // proj_pts.push_back(q);

        // patch.proj_from_ring_gid.push_back(gid);
    // }

    // std::cout << "[TransitionPatch] Projected kept = "
              // << proj_pts.size() << "\n";

    // // ------------------------------------------------------------
    // // 3. Store as polylines
    // // ------------------------------------------------------------
    // patch.ring_polyline = Polyline2D(ring_pts);
    // patch.proj_polyline = Polyline2D(proj_pts);

    // return patch;
// }

TransitionPatch2D build_transition_patch_from_band(
    const Mesh2D& band_mesh,
    const Polyline2D& contour,
    double SL
)
{
    TransitionPatch2D patch;

    auto ring_gids = band_mesh.find_ordered_boundary_nodes();
    double rmin = 0.5 * SL;
    
    patch.proj_lid_to_index.clear();
    // -----------------------------
    // 1. Ring nodes
    // -----------------------------
    for(int gid : ring_gids)
    {
        Vec2 p = band_mesh.node(gid);

        int lid = patch.points.size();

        patch.points.push_back(p);
        patch.local_to_global.push_back(gid);
        patch.flags.push_back(NodeFlag::NODE_RING);

        patch.ring_loop.push_back(lid);
    }

    // -----------------------------
    // 2. Projected nodes
    // -----------------------------
    for(size_t i = 0; i < ring_gids.size(); ++i)
    {
        Vec2 p = band_mesh.node(ring_gids[i]);
        Vec2 q = contour.project(p);

        bool too_close = false;
        for(int lid : patch.proj_loop)
        {
            if((patch.points[lid] - q).norm() < rmin)
            {
                too_close = true;
                break;
            }
        }
        if(too_close) continue;

        int lid = patch.points.size();

        patch.points.push_back(q);
        patch.local_to_global.push_back(-1);
        patch.flags.push_back(NodeFlag::NODE_PROJECTED);

        patch.proj_loop.push_back(lid);
        patch.proj_from_ring_gid.push_back(ring_gids[i]);
        patch.proj_lid_to_index.push_back(-1); // no projected
    }

    return patch;
}


void debug_print_patch_nodes(const TransitionPatch2D& patch, size_t Nmax = 50)
{
    std::cout << "\n[Patch Debug] Local nodes\n";
    std::cout << "--------------------------------------------------\n";
    std::cout << " lid | flag | global |        x        y | from_ring_gid\n";
    std::cout << "--------------------------------------------------\n";

    size_t N = std::min(Nmax, patch.points.size());

    for(size_t lid = 0; lid < N; ++lid)
    {
        const Vec2& p = patch.points[lid];
        int gid = patch.local_to_global[lid];

        std::string flag_str;
        switch(patch.flags[lid])
        {
            case NodeFlag::NODE_RING:      flag_str = "RING"; break;
            case NodeFlag::NODE_PROJECTED: flag_str = "PROJ"; break;
            default:                       flag_str = "OTHR"; break;
        }

        std::cout << std::setw(4) << lid << " | "
                  << std::setw(4) << flag_str << " | "
                  << std::setw(6) << gid << " | "
                  << std::setw(8) << p.x << " "
                  << std::setw(8) << p.y;

        // projected → mostrar origen
        if(patch.flags[lid] == NodeFlag::NODE_PROJECTED)
        {
            int pid = patch.proj_lid_to_index[lid];
            if(pid >= 0)
            {
                std::cout << " | from ring_gid = "
                          << patch.proj_from_ring_gid[pid];
            }
        }


        std::cout << "\n";
    }

    std::cout << "--------------------------------------------------\n";
}


// void merge_triangles_to_quads(
    // TransitionPatch2D& patch,
    // std::vector<std::array<int,3>>& tris_left
// )
// {
    // const auto& tris = patch.triangles;

    // std::map<Edge, std::vector<int>> edge_to_tris;

    // // ----------------------------------------
    // // 1. Build edge → triangle map
    // // ----------------------------------------
    // for(size_t t = 0; t < tris.size(); ++t)
    // {
        // const auto& tri = tris[t];

        // edge_to_tris[Edge(tri[0], tri[1])].push_back(t);
        // edge_to_tris[Edge(tri[1], tri[2])].push_back(t);
        // edge_to_tris[Edge(tri[2], tri[0])].push_back(t);
    // }

    // std::vector<bool> used(tris.size(), false);

    // // ----------------------------------------
    // // 2. Merge pairs
    // // ----------------------------------------
    // for(const auto& it : edge_to_tris)
    // {
        // const auto& tri_ids = it.second;
        // if(tri_ids.size() != 2) continue;

        // int t0 = tri_ids[0];
        // int t1 = tri_ids[1];

        // if(used[t0] || used[t1]) continue;

        // const auto& A = tris[t0];
        // const auto& B = tris[t1];

        // // collect unique vertices
        // std::set<int> verts;
        // verts.insert(A.begin(), A.end());
        // verts.insert(B.begin(), B.end());

        // if(verts.size() != 4) continue;

        // std::array<int,4> quad;
        // int k = 0;
        // for(int v : verts) quad[k++] = v;

        // patch.quads.push_back(quad);

        // used[t0] = true;
        // used[t1] = true;
    // }

    // // ----------------------------------------
    // // 3. Remaining triangles
    // // ----------------------------------------
    // tris_left.clear();
    // for(size_t t = 0; t < tris.size(); ++t)
    // {
        // if(!used[t])
            // tris_left.push_back(tris[t]);
    // }
// }


}
