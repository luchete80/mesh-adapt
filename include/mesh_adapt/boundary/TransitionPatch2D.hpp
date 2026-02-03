#pragma once
#include "mesh_adapt/core/Mesh2D.hpp"
#include "mesh_adapt/geometry/Polyline2D.hpp"
#include "mesh_adapt/geometry/Edge.hpp" // to merge quads
#include <map> 
#include <set>

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


    std::vector<std::array<int,3>> tris_left;
    
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
        patch.proj_lid_to_index.push_back(-1); // no projected
        
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
        int pid = patch.proj_from_ring_gid.size();
        
        patch.points.push_back(q);
        patch.local_to_global.push_back(-1);
        patch.flags.push_back(NodeFlag::NODE_PROJECTED);

        patch.proj_loop.push_back(lid);
        patch.proj_from_ring_gid.push_back(ring_gids[i]);
        patch.proj_lid_to_index.push_back(pid); // 👈 ACÁ SÍ VA EL pid
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
          if(lid < patch.proj_lid_to_index.size())
          {
              int pid = patch.proj_lid_to_index[lid];
              if(pid >= 0 && pid < (int)patch.proj_from_ring_gid.size())
              {
                  std::cout << " | from ring_gid = "
                            << patch.proj_from_ring_gid[pid];
              }
              else
              {
                  std::cout << " | from ring_gid = INVALID_PID(" << pid << ")";
              }
          }
          else
          {
              std::cout << " | proj_lid_to_index OOB";
          }
      }



        std::cout << "\n";
    }

    std::cout << "--------------------------------------------------\n";
}

struct QuadCandidate {
    double Q;
    int t0, t1;
    std::array<int,4> quad;
    QuadCandidate(int _t0, int _t1, const std::array<int,4>& _quad, double _Q)
        : t0(_t0), t1(_t1), quad(_quad), Q(_Q) {}
};

// Ángulos de un quad ccw
std::vector<double> quad_angles(const std::array<int,4>& q, const std::vector<Vec2>& pts)
{
    std::vector<double> angles(4);
    for(int i=0;i<4;i++)
    {
        const Vec2& A = pts[q[i]];
        const Vec2& B = pts[q[(i+1)%4]];
        const Vec2& C = pts[q[(i+2)%4]];

        Vec2 u = B - A;
        Vec2 v = C - B;

        double cos_theta = (u.x*v.x + u.y*v.y) / (u.norm() * v.norm());
        cos_theta = std::max(-1.0, std::min(1.0, cos_theta)); // clamp
        angles[(i+1)%4] = std::acos(cos_theta);
    }
    return angles;
}

double quad_quality(const std::array<int,4>& q, const std::vector<Vec2>& pts)
{
    std::array<double,4> L;
    for(int i=0;i<4;i++)
        L[i] = (pts[q[i]] - pts[q[(i+1)%4]]).norm();

    double Lmax = *std::max_element(L.begin(), L.end());
    double Lmin = *std::min_element(L.begin(), L.end());

    std::vector<double> angles = quad_angles(q, pts);
    double amin = *std::min_element(angles.begin(), angles.end());

    double Q = std::pow(Lmax/Lmin, 2) * std::pow(1.0/std::sin(amin), 3);
    return Q;
}


//~ void merge_triangles_to_quads(TransitionPatch2D& patch, double Qmin = 5.0)
//~ {
    //~ const auto& tris = patch.triangles;
    //~ const auto& pts  = patch.points;

    //~ std::map<Edge, std::vector<int>> edge_to_tris;

    //~ // 1. Construir edge -> triángulos
    //~ for(size_t t=0; t<tris.size(); ++t)
    //~ {
        //~ const auto& tri = tris[t];
        //~ edge_to_tris[Edge(tri[0],tri[1])].push_back((int)t);
        //~ edge_to_tris[Edge(tri[1],tri[2])].push_back((int)t);
        //~ edge_to_tris[Edge(tri[2],tri[0])].push_back((int)t);
    //~ }

    //~ std::vector<QuadCandidate> candidates;

    //~ // 2. Generar todos los candidatos
    //~ for(const auto& it : edge_to_tris)
    //~ {
        //~ const auto& tri_ids = it.second;
        //~ if(tri_ids.size() != 2) continue; // debe ser compartido por 2 triángulos

        //~ int t0 = tri_ids[0];
        //~ int t1 = tri_ids[1];

        //~ // vértices únicos de ambos triángulos
        //~ std::set<int> verts;
        //~ verts.insert(tris[t0].begin(), tris[t0].end());
        //~ verts.insert(tris[t1].begin(), tris[t1].end());

        //~ if(verts.size() != 4) continue; // no forma un quad

        //~ std::array<int,4> quad;
        //~ int k=0;
        //~ for(int v : verts) quad[k++] = v;

        //~ double Q = quad_quality(quad, pts); // tu función de calidad

        //~ std::cout << "[DEBUG] Candidate quad (" 
                  //~ << quad[0] << "," << quad[1] << "," 
                  //~ << quad[2] << "," << quad[3] 
                  //~ << ") Q = " << Q << "\n";

        //~ candidates.push_back({Q, t0, t1, quad});
    //~ }

    //~ std::cout << "[DEBUG] Total candidates: " << candidates.size() << "\n";

    //~ // 3. Ordenar por calidad (menor Q = peor)
    //~ std::sort(candidates.begin(), candidates.end(),
        //~ [](const QuadCandidate& a, const QuadCandidate& b){
            //~ return a.Q < b.Q;
        //~ });

    //~ std::vector<bool> used_tris(tris.size(), false);
    //~ patch.quads.clear();

    //~ // 4. Selección greedy
    //~ int accepted = 0;
    //~ for(const auto& cand : candidates)
    //~ {
        //~ if(used_tris[cand.t0] || used_tris[cand.t1]) continue;
        //~ if(cand.Q > Qmin) continue; // criterio mínimo

        //~ patch.quads.push_back(cand.quad);
        //~ used_tris[cand.t0] = true;
        //~ used_tris[cand.t1] = true;
        //~ accepted++;
    //~ }

    //~ std::cout << "[DEBUG] Quads accepted: " << accepted << "\n";

    //~ // 5. Triángulos restantes
    //~ patch.tris_left.clear();
    //~ for(size_t t=0; t<tris.size(); ++t)
        //~ if(!used_tris[t])
            //~ patch.tris_left.push_back(tris[t]);

    //~ std::cout << "[DEBUG] Triangles left: " << patch.tris_left.size() << "\n";
//~ }


void merge_triangles_to_quads(TransitionPatch2D& patch, double Qmin = 5.0)
{
    const auto& tris = patch.triangles;
    const auto& pts  = patch.points;

    // --- 1. Map edge -> tris
    std::map<Edge, std::vector<int>> edge_to_tris;
    for(size_t t=0;t<tris.size();++t)
    {
        const auto& tri = tris[t];
        edge_to_tris[Edge(tri[0], tri[1])].push_back((int)t);
        edge_to_tris[Edge(tri[1], tri[2])].push_back((int)t);
        edge_to_tris[Edge(tri[2], tri[0])].push_back((int)t);
    }

    std::vector<QuadCandidate> candidates;

    // --- 2. Generar todos los candidatos
    for(const auto& it : edge_to_tris)
    {
        const auto& tri_ids = it.second;
        if(tri_ids.size() != 2) continue;

        int t0 = tri_ids[0];
        int t1 = tri_ids[1];

        const auto& A = tris[t0];
        const auto& B = tris[t1];

        // buscar borde compartido
        int shared_a=-1, shared_b=-1;
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                if((A[i]==B[j] && A[(i+1)%3]==B[(j+1)%3]) ||
                   (A[i]==B[(j+1)%3] && A[(i+1)%3]==B[j]))
                {
                    shared_a = A[i];
                    shared_b = A[(i+1)%3];
                    break;
                }
            }
            if(shared_a!=-1) break;
        }
        if(shared_a==-1) continue;

        // vértices únicos
        int uniqueA=-1, uniqueB=-1;
        for(int v : A) if(v!=shared_a && v!=shared_b) uniqueA = v;
        for(int v : B) if(v!=shared_a && v!=shared_b) uniqueB = v;

        std::array<int,4> quad = {uniqueA, shared_a, uniqueB, shared_b};

        double Q = quad_quality(quad, pts);

        std::cout << "[DEBUG] Candidate quad (" 
                  << quad[0]<<","<<quad[1]<<","<<quad[2]<<","<<quad[3] 
                  << ") Q=" << Q << "\n";

        if(Q <= Qmin)
            candidates.push_back({t0, t1, quad, Q});
    }

    std::cout << "[DEBUG] Total candidates passing Qmin=" << Qmin 
              << ": " << candidates.size() << "\n";

    // --- 3. Ordenar por calidad (mayor Q primero)
    std::sort(candidates.begin(), candidates.end(),
        [](const QuadCandidate& a, const QuadCandidate& b){ return a.Q < b.Q; });

    // --- 4. Selección greedy
    std::vector<bool> used_tris(tris.size(), false);
    patch.quads.clear();

    for(const auto& cand : candidates)
    {
        if(used_tris[cand.t0] || used_tris[cand.t1]) continue;
        patch.quads.push_back(cand.quad);
        used_tris[cand.t0] = true;
        used_tris[cand.t1] = true;
    }

    // --- 5. Triángulos restantes
    patch.tris_left.clear();
    for(size_t t=0;t<tris.size();++t)
        if(!used_tris[t])
            patch.tris_left.push_back(tris[t]);

    std::cout << "[DEBUG] Quads accepted: " << patch.quads.size() 
              << ", triangles left: " << patch.tris_left.size() << "\n";
}

}
