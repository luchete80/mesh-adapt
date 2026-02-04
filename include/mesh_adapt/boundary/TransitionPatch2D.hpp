#pragma once
#include "mesh_adapt/core/Mesh2D.hpp"
#include "mesh_adapt/geometry/Polyline2D.hpp"
#include "mesh_adapt/geometry/Edge.hpp" // to merge quads
#include <map> 
#include <set>
#include <fstream>

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
    std::vector<std::array<int,4>> quads_fallback;


    std::vector<std::array<int,3>> tris_left;
    
    // loops (orden topológico)
    std::vector<int> ring_loop;   // local ids
    std::vector<int> proj_loop;   // local ids

    // relación projected ↔ ring
    std::vector<int> proj_from_ring_gid;
    // NUEVO: local projected lid -> index en proj_loop
    // size == points.size(), -1 si no es projected
    std::vector<int> proj_lid_to_index;

    //REDUNDANT
    std::set<Edge> subdivided_edges; //EDGES WHICH HAVE fallback quads
    std::map<Edge,int> subdivided_edge_to_node; // map: edge -> nuevo nodo local

    
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



/////////////// ORIGNAL, NO S
/////////////// CONFIDENT OF RING ORDERED NODES

// TransitionPatch2D build_transition_patch_from_band(
    // const Mesh2D& band_mesh,
    // const Polyline2D& contour,
    // double SL
// )
// {
    // TransitionPatch2D patch;

    // auto ring_gids = band_mesh.find_ordered_boundary_nodes();
    // double rmin = 0.5 * SL;
    
    // patch.proj_lid_to_index.clear();
    // // -----------------------------
    // // 1. Ring nodes
    // // -----------------------------
    // for(int gid : ring_gids)
    // {
        // Vec2 p = band_mesh.node(gid);

        // int lid = patch.points.size();

        // patch.points.push_back(p);
        // patch.local_to_global.push_back(gid);
        // patch.flags.push_back(NodeFlag::NODE_RING);

        // patch.ring_loop.push_back(lid);
        // patch.proj_lid_to_index.push_back(-1); // no projected
        
    // }

    // // -----------------------------
    // // 2. Projected nodes
    // // -----------------------------
    // for(size_t i = 0; i < ring_gids.size(); ++i)
    // {
        // Vec2 p = band_mesh.node(ring_gids[i]);
        // Vec2 q = contour.project(p);

        // bool too_close = false;
        // for(int lid : patch.proj_loop)
        // {
            // if((patch.points[lid] - q).norm() < rmin)
            // {
                // too_close = true;
                // break;
            // }
        // }
        // if(too_close) continue;

        // int lid = patch.points.size();
        // int pid = patch.proj_from_ring_gid.size();
        
        // patch.points.push_back(q);
        // patch.local_to_global.push_back(-1);
        // patch.flags.push_back(NodeFlag::NODE_PROJECTED);

        // patch.proj_loop.push_back(lid);
        // patch.proj_from_ring_gid.push_back(ring_gids[i]);
        // patch.proj_lid_to_index.push_back(pid); // 👈 ACÁ SÍ VA EL pid
    // }

    // return patch;
// }


struct ProjData {
    Vec2 point;
    Vec2 tangent;
    int ring_gid;
    double distance;
    int contour_segment_id;
};

TransitionPatch2D build_transition_patch_from_band(
    const Mesh2D& band_mesh,
    Polyline2D& contour,
    double SL
) {
    TransitionPatch2D patch;
    const double rmin = 0.5 * SL;
    const double angle_threshold = 45.0;
    
    // --------------------------------------------------
    // PASO 1: Puntos del ring (YA ORDENADOS)
    // --------------------------------------------------
    auto ring_gids = band_mesh.find_ordered_boundary_nodes();
    
    for(int gid : ring_gids) {
        Vec2 p = band_mesh.node(gid);
        int lid = patch.points.size();
        
        patch.points.push_back(p);
        patch.local_to_global.push_back(gid);
        patch.flags.push_back(NodeFlag::NODE_RING);
        patch.ring_loop.push_back(lid);
        patch.proj_lid_to_index.push_back(-1);
    }
    
    // --------------------------------------------------
    // PASO 2: Recolectar proyecciones
    // --------------------------------------------------
    std::vector<ProjData> all_projections;
    
    //~ for(int gid : ring_gids) {
        //~ Vec2 p = band_mesh.node(gid);
        //~ auto proj = contour.project_with_segment(p);
        
        //~ //auto proj = contour.project_xy(p);
        
        //~ ProjData data;
        //~ data.point = proj.q;
        //~ data.ring_gid = gid;
        //~ data.distance = (p - proj.q).norm();
        //~ data.contour_segment_id = proj.seg_id;
        
        //~ // Calcular tangente
        //~ if(proj.seg_id < static_cast<int>(contour.pts.size()) - 1) {
            //~ data.tangent = (contour.pts[proj.seg_id + 1] - 
                           //~ contour.pts[proj.seg_id]).normalized();
        //~ } else {
            //~ data.tangent = (contour.pts[proj.seg_id] - 
                           //~ contour.pts[proj.seg_id - 1]).normalized();
        //~ }
        
        //~ all_projections.push_back(data);

    // <-- PRINT -->
    //~ std::cout << "[DEBUG] ring_gid=" << gid
              //~ << " p=(" << p.x << "," << p.y << ")"
              //~ << " proj=(" << proj.q.x << "," << proj.q.y << ")"
              //~ << " distance=" << data.distance
              //~ << " seg_id=" << proj.seg_id << "\n";
       

    //~ }

    for(int gid : ring_gids) {
        Vec2 p = band_mesh.node(gid);

        // ==========================================
        // Usar project_corner_aware (obtiene TODAS las proyecciones)
        // ==========================================
        std::vector<ProjectionResult> proj_results = contour.project_corner_aware(p);
        
        if(proj_results.empty()) {
            std::cerr << "[WARNING] No se encontraron proyecciones para ring_gid=" << gid << "\n";
            continue;
        }

        // ==========================================
        // Procesar TODAS las proyecciones, no solo la más cercana
        // ==========================================
        for(const auto& proj : proj_results) {
            ProjData data;
            data.point = proj.q;
            data.ring_gid = gid;  // Mismo ring_gid para todas
            data.distance = (p - proj.q).norm();
            data.contour_segment_id = proj.seg_id;

            // Calcular tangente según segmento
            if(proj.seg_id < static_cast<int>(contour.pts.size()) - 1) {
                data.tangent = (contour.pts[proj.seg_id + 1] - contour.pts[proj.seg_id]).normalized();
            } else {
                data.tangent = (contour.pts[proj.seg_id] - contour.pts[proj.seg_id - 1]).normalized();
            }

            all_projections.push_back(data);

            std::cout << "[DEBUG] ring_gid=" << gid
                      << " p=(" << p.x << "," << p.y << ")"
                      << " proj=(" << proj.q.x << "," << proj.q.y << ")"
                      << " distance=" << data.distance
                      << " seg_id=" << proj.seg_id
                      << " t=" << proj.t << "\n";
        }
    }
        
    // --------------------------------------------------
    // PASO 3: Detectar vértices del contorno
    // --------------------------------------------------
    struct VertexInfo {
        Vec2 point;
        Vec2 tangent_before;
        Vec2 tangent_after;
        int vertex_id; // índice en contour.pts
    };
    
    std::vector<VertexInfo> contour_vertices;
    
    // Asumimos que contour YA está cerrado (pts[0]==pts[n-1])
    //for(size_t i = 1; i < contour.pts.size(); ++i) {
    for(size_t i = 1; i < contour.pts.size(); ++i) {
        Vec2 prev = contour.pts[i-1];
        Vec2 curr = contour.pts[i];
        Vec2 next = contour.pts[i+1];
        
        Vec2 t_before = (curr - prev).normalized();
        Vec2 t_after = (next - curr).normalized();
        
        double cos_angle = t_before.dot(t_after);
        cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
        double angle = std::acos(cos_angle) * 180.0 / 3.14159265358979323846;
        
        if(angle < 150.0) {
            VertexInfo vi;
            vi.point = curr;
            vi.tangent_before = t_before;
            vi.tangent_after = t_after;
            vi.vertex_id = static_cast<int>(i);
            contour_vertices.push_back(vi);
        }
    }
    
    // --------------------------------------------------
    // PASO 4: Filtrado con distancia + dirección
    // --------------------------------------------------
    std::vector<ProjData> filtered_projections;
    std::vector<bool> kept(all_projections.size(), true);
    
    for(size_t i = 0; i < all_projections.size(); ++i) {
        if(!kept[i]) continue;
        
        for(size_t j = i + 1; j < all_projections.size(); ++j) {
            if(!kept[j]) continue;
            
            const auto& pi = all_projections[i];
            const auto& pj = all_projections[j];
            
            double d = (pi.point - pj.point).norm();
            if(d < rmin) {
                double cos_angle = pi.tangent.dot(pj.tangent);
                cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
                double angle = std::acos(cos_angle) * 180.0 / 3.14159265358979323846;
                
                if(angle < angle_threshold) {
                    if(pi.distance < pj.distance) {
                        kept[j] = false;
                    } else {
                        kept[i] = false;
                        break;
                    }
                }
            }
        }
        
        if(kept[i]) {
            filtered_projections.push_back(all_projections[i]);
        }
    }
    
    // --------------------------------------------------
    // PASO 5: Añadir vértices críticos
    // --------------------------------------------------
    for(const auto& vertex : contour_vertices) {
        bool has_nearby = false;
        
        for(const auto& proj : filtered_projections) {
            double d = (proj.point - vertex.point).norm();
            if(d < rmin * 0.5) {
                has_nearby = true;
                break;
            }
        }
        
        if(!has_nearby) {
            ProjData vertex_proj;
            vertex_proj.point = vertex.point;
            vertex_proj.ring_gid = -1; // Es punto crítico
            vertex_proj.distance = 0;
            vertex_proj.tangent = (vertex.tangent_before + vertex.tangent_after).normalized();
            vertex_proj.contour_segment_id = vertex.vertex_id;
            
            filtered_projections.push_back(vertex_proj);
        }
    }
    
    // --------------------------------------------------
    // PASO 6: Insertar en orden TOPOLÓGICO
    // --------------------------------------------------
    // Creamos una polyline temporal para ordenar
    std::vector<Vec2> temp_polyline;
    std::vector<int> temp_indices; // índices a filtered_projections
    
    for(size_t i = 0; i < filtered_projections.size(); ++i) {
        temp_polyline.push_back(filtered_projections[i].point);
        temp_indices.push_back(static_cast<int>(i));
    }
    
    // Ordenar por posición angular alrededor del centroide
    if(!temp_polyline.empty()) {
        // Calcular centroide
        Vec2 centroid(0,0);
        for(const auto& p : temp_polyline) centroid += p;
        centroid = centroid / temp_polyline.size();
        
        // Ordenar por ángulo polar
        std::sort(temp_indices.begin(), temp_indices.end(),
            [&](int a, int b) {
                Vec2 va = temp_polyline[a] - centroid;
                Vec2 vb = temp_polyline[b] - centroid;
                return std::atan2(va.y, va.x) < std::atan2(vb.y, vb.x);
            });
        
        // Insertar EN ORDEN
        for(int idx : temp_indices) {
            const auto& proj = filtered_projections[idx];
            int lid = patch.points.size();
            
            patch.points.push_back(proj.point);
            patch.local_to_global.push_back(-1);
            
            if(proj.ring_gid >= 0) {
                patch.flags.push_back(NodeFlag::NODE_PROJECTED);
                int pid = patch.proj_from_ring_gid.size();
                patch.proj_from_ring_gid.push_back(proj.ring_gid);
                patch.proj_lid_to_index.push_back(pid);
            } else {
                patch.flags.push_back(NodeFlag::NODE_CRITICAL);
                patch.proj_lid_to_index.push_back(-1);
            }
            
            patch.proj_loop.push_back(lid);
        }
        
        // --------------------------------------------------
        // PASO 7: CERRAR proj_loop si no está cerrada
        // --------------------------------------------------
        if(patch.proj_loop.size() >= 2) {
            int first_lid = patch.proj_loop[0];
            int last_lid = patch.proj_loop.back();
            
            if(first_lid != last_lid) {
                // Verificar si el primer y último punto son cercanos
                const Vec2& first_pt = patch.points[first_lid];
                const Vec2& last_pt = patch.points[last_lid];
                
                if(first_pt.distance(last_pt) > rmin * 0.1) {
                    // No están cerrados, agregar copia del primero
                    patch.proj_loop.push_back(first_lid);
                    std::cout << "[INFO] Closed proj_loop (added copy of first point)\n";
                } else {
                    // Ya están esencialmente cerrados (muy cercanos)
                    std::cout << "[INFO] proj_loop is effectively closed\n";
                }
            } else {
                std::cout << "[INFO] proj_loop already closed (first==last)\n";
            }
        }
    }
    
    return patch;
}

// ======================================================
// Función para crear polyline CERRADA desde proj_loop
// ======================================================
Polyline2D make_closed_polyline_from_proj_loop(const TransitionPatch2D& patch) {
    Polyline2D pl;
    
    if(patch.proj_loop.empty()) return pl;
    
    // Copiar todos los puntos
    for(int lid : patch.proj_loop) {
        pl.pts.push_back(patch.points[lid]);
    }
    
    // Asegurar que esté cerrada
    if(pl.pts.size() >= 2 && pl.pts[0] != pl.pts.back()) {
        pl.pts.push_back(pl.pts[0]);
    }
    
    return pl;
}

// ======================================================
// Función para verificar si una polyline está cerrada
// ======================================================
bool is_polyline_closed(const Polyline2D& pl, double eps = 1e-9) {
    if(pl.pts.size() < 2) return false;
    return pl.pts[0].distance(pl.pts.back()) < eps;
}


void visualize_critical_points(const TransitionPatch2D& patch, 
                              const std::string& filename = "critical_points.vtk") {
    std::ofstream fout(filename);
    if(!fout) {
        std::cerr << "Cannot open " << filename << " for writing\n";
        return;
    }
    
    // Contar puntos críticos
    std::vector<Vec2> critical_pts;
    std::vector<int> critical_ids;
    
    for(size_t lid = 0; lid < patch.points.size(); ++lid) {
        if(patch.flags[lid] == NodeFlag::NODE_CRITICAL) {
            critical_pts.push_back(patch.points[lid]);
            critical_ids.push_back(static_cast<int>(lid));
        }
    }
    
    // Escribir VTK
    fout << "# vtk DataFile Version 3.0\n";
    fout << "Critical Points\n";
    fout << "ASCII\n";
    fout << "DATASET UNSTRUCTURED_GRID\n";
    
    // Puntos
    fout << "POINTS " << critical_pts.size() << " float\n";
    for(const auto& p : critical_pts) {
        fout << p.x << " " << p.y << " 0.0\n";
    }
    
    // Celdas (puntos)
    fout << "CELLS " << critical_pts.size() << " " << (2 * critical_pts.size()) << "\n";
    for(size_t i = 0; i < critical_pts.size(); ++i) {
        fout << "1 " << i << "\n";
    }
    
    // Tipos de celda (VTK_VERTEX = 1)
    fout << "CELL_TYPES " << critical_pts.size() << "\n";
    for(size_t i = 0; i < critical_pts.size(); ++i) {
        fout << "1\n";
    }
    
    // Datos (IDs)
    fout << "POINT_DATA " << critical_pts.size() << "\n";
    fout << "SCALARS lid int 1\n";
    fout << "LOOKUP_TABLE default\n";
    for(int id : critical_ids) {
        fout << id << "\n";
    }
    
    fout.close();
    std::cout << "[DEBUG] Critical points saved to " << filename 
              << " (" << critical_pts.size() << " points)\n";
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



// subdivide triangles into quads via barycenter + edge midpoints
inline void subdivide_tris_to_quads(
    TransitionPatch2D& patch
)
{
    const auto& tris = patch.tris_left;
    const auto& pts  = patch.points;

    std::map<Edge,int> midpoint_cache; // edge -> local_id del midpoint

    std::vector<std::array<int,4>> new_quads;

    for(const auto& tri : tris)
    {
        // tri vertices
        int A = tri[0];
        int B = tri[1];
        int C = tri[2];

        // --- 1. Baricentro ---
        Vec2 G = (pts[A] + pts[B] + pts[C]) / 3.0;
        int gid = patch.points.size();
        patch.points.push_back(G);
        patch.local_to_global.push_back(-1);
        patch.flags.push_back(NodeFlag::NODE_CENTER); // tri subdivided
        int Gid = gid;

        // --- 2. Midpoints de aristas ---
        auto get_midpoint = [&](int i, int j) -> int {
            Edge e(i,j);
            auto it = midpoint_cache.find(e);
            if(it != midpoint_cache.end())
                return it->second;

            Vec2 pmid = (pts[i] + pts[j]) / 2.0;
            int lid = patch.points.size();
            patch.points.push_back(pmid);
            patch.local_to_global.push_back(-1);
            patch.flags.push_back(NodeFlag::NODE_SUBDIVIDED);
            midpoint_cache[e] = lid;
            patch.subdivided_edges.insert(e); // <-- guardar edge subdividido
            patch.subdivided_edge_to_node[e] = lid; // <-- NUEVO map
            return lid;
        };

        int AB = get_midpoint(A,B);
        int BC = get_midpoint(B,C);
        int CA = get_midpoint(C,A);

        // --- 3. Crear 3 quads por tri ---
        // Quad 1: A, AB, G, CA
        new_quads.push_back({A, AB, Gid, CA});
        // Quad 2: B, BC, G, AB
        new_quads.push_back({B, BC, Gid, AB});
        // Quad 3: C, CA, G, BC
        new_quads.push_back({C, CA, Gid, BC});
    }

    patch.quads_fallback.insert(patch.quads_fallback.end(), new_quads.begin(), new_quads.end());
    patch.tris_left.clear(); // todos los tri ya subdivididos
}


// Función de debug para comparar set vs map
inline void debug_edges_vs_map(const TransitionPatch2D& patch)
{
    std::cout << "\n[DEBUG] Edges subdivided (set vs map)\n";
    std::cout << "--------------------------------------------------\n";
    std::cout << "Edge(a,b) | in set? | in map? | mapped node\n";
    std::cout << "--------------------------------------------------\n";

    // Recorrer todos los edges que aparecen en set o map
    std::set<Edge> all_edges;
    all_edges.insert(patch.subdivided_edges.begin(), patch.subdivided_edges.end());
    for(const auto& kv : patch.subdivided_edge_to_node)
        all_edges.insert(kv.first);

    for(const auto& e : all_edges)
    {
        bool in_set = patch.subdivided_edges.count(e) > 0;
        auto it_map = patch.subdivided_edge_to_node.find(e);
        bool in_map = it_map != patch.subdivided_edge_to_node.end();
        int mapped_node = in_map ? it_map->second : -1;

        std::cout << "(" << std::setw(3) << e.a << "," << std::setw(3) << e.b << ")"
                  << " | " << std::setw(6) << (in_set ? "YES" : "NO")
                  << " | " << std::setw(6) << (in_map ? "YES" : "NO")
                  << " | " << std::setw(6) << mapped_node
                  << "\n";
    }
    std::cout << "--------------------------------------------------\n";
    std::cout << "[DEBUG] Total edges in set: " << patch.subdivided_edges.size() 
              << ", in map: " << patch.subdivided_edge_to_node.size() << "\n\n";
}


}
