// ===== File: mesh-adapt/subdivision/MeshToSub.hpp =====
#pragma once
#include <vector>
#include <unordered_map>
#include "mesh_adapt/core/Mesh2D.hpp"
#include "mesh_adapt/boundary/TransitionPatch2D.hpp"
#include "mesh_adapt/geometry/EdgeInfo.hpp"

#include <stdexcept>

namespace mesh_adapt {

class MeshToSub {
public:
    Mesh2D mesh;  // La malla temporal
    std::vector<int> local_to_global; // indices globales en el patch

    // Constructor principal: crea malla temporal a partir de band_mesh + patch
    MeshToSub(const Mesh2D& band_mesh, const TransitionPatch2D& patch);

    // Métodos auxiliares
    int add_projected_node(int lid, const TransitionPatch2D& patch);
    void add_patch_quads(const TransitionPatch2D& patch);

    size_t num_nodes() const { return mesh.num_nodes(); }
    size_t num_quads() const { return mesh.num_quads(); }

};


MeshToSub::MeshToSub(const Mesh2D& band_mesh, const TransitionPatch2D& patch) {
    // 1) Copiar nodos y quads de la band_mesh
    mesh.nodes = band_mesh.get_nodes();
    mesh.quads = band_mesh.get_quads();

    // 2) Inicializar local_to_global con -1 (size = número de puntos en el patch)
    local_to_global.resize(patch.points.size(), -1);

    // 3) Insertar nodos proyectados usando proj_lid_to_index
    for(size_t lid = 0; lid < patch.points.size(); ++lid) {
        if(patch.proj_lid_to_index[lid] >= 0) { 
            local_to_global[lid] = add_projected_node(lid, patch);
        }
    }

    // 4) Insertar quads del patch (solo quads "proyectados", sin fallback)
    add_patch_quads(patch);
}

int MeshToSub::add_projected_node(int lid, const TransitionPatch2D& patch) {
    const Vec2& p = patch.points[lid];
    int new_id = static_cast<int>(mesh.nodes.size());
    mesh.add_node(p.x, p.y);
    return new_id;
}

//~ void MeshToSub::add_patch_quads(const TransitionPatch2D& patch) {
    //~ for(const auto& q : patch.quads) {
        //~ std::array<int,4> new_ids;
        //~ bool skip = false;
        //~ for(int i = 0; i < 4; ++i) {
            //~ int lid = q[i];                     // ahora es std::array<int,4>
            //~ int gid = local_to_global[lid];     // índice en la malla temporal
            //~ if(gid < 0) { skip = true; break; } // nodo no proyectado
            //~ new_ids[i] = gid;
        //~ }
        //~ if(!skip) {
            //~ mesh.add_quad(new_ids[0], new_ids[1], new_ids[2], new_ids[3]);
        //~ }
    //~ }
//~ }

void MeshToSub::add_patch_quads(const TransitionPatch2D& patch) {
    for(const auto& q : patch.quads) {
        std::array<int,4> new_ids;
        for(int i=0;i<4;++i) {
            int lid = q[i];
            if(patch.flags[lid] == NodeFlag::NODE_PROJECTED) {
                // projected node → ya está en local_to_global
                new_ids[i] = local_to_global[lid];
            } else {
                // ring node → el ID ya coincide con el global original (band_mesh)
                new_ids[i] = patch.local_to_global[lid];
            }
        }
        mesh.add_quad(new_ids[0], new_ids[1], new_ids[2], new_ids[3]);
    }
}



// Builder de edge_map desde MeshToSub
inline std::map<Edge, EdgeInfo> build_edge_map(const class MeshToSub& mesh_to_sub)
{
    std::map<Edge, EdgeInfo> edge_map;

    const auto& quads = mesh_to_sub.mesh.quads;
    const auto& nodes = mesh_to_sub.mesh.nodes;

    for(size_t qid = 0; qid < quads.size(); ++qid)
    {
        const auto& q = quads[qid];
        for(int i = 0; i < 4; ++i)
        {
            int i0 = q[i];
            int i1 = q[(i+1)%4];
            Edge e(i0, i1);

            auto it = edge_map.find(e);
            if(it == edge_map.end())
            {
                EdgeInfo info(nodes[i0].x, nodes[i1].x);
                info.quad_refs.emplace_back(qid, i);
                edge_map[e] = info;
            }
            else
            {
                it->second.quad_refs.emplace_back(qid, i);
            }
        }
    }

    return edge_map;
}

}

