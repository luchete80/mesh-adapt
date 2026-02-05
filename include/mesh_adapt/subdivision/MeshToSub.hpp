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
    
    std::set<Edge> subdivided_edges_global; //EDGES WHICH HAVE fallback quads
    std::map<Edge,int> subdivided_edge_to_global; // Nuevo: edge del patch → nodo global (midpoint/baricentro)


    // NUEVO: Verificar consistencia
    void verify_consistency(const TransitionPatch2D& patch);

};

MeshToSub::MeshToSub(const Mesh2D& band_mesh, const TransitionPatch2D& patch) {
    // 1) Copiar nodos y quads de la band_mesh
    mesh.nodes = band_mesh.get_nodes();
    mesh.quads = band_mesh.get_quads();

    // 2) Inicializar local_to_global con -1
    local_to_global.resize(patch.points.size(), -1);

    // 3) Insertar TODOS los nodos del patch que NO sean NODE_RING
    for(size_t lid = 0; lid < patch.points.size(); ++lid) {
        if(patch.flags[lid] != NodeFlag::NODE_RING) {
            // PROYECTED o CRITICAL: añadir a la malla
            local_to_global[lid] = add_projected_node(lid, patch);
        } else {
            // RING: el índice global ya está en patch.local_to_global
            local_to_global[lid] = patch.local_to_global[lid];
        }
    }

    // 4) Insertar quads del patch
    add_patch_quads(patch);

    // 5) Guardar edges subdivididos en global indices
    for(const Edge& e_local : patch.subdivided_edges) {
        int i0, i1;
        
        // Convertir índices locales a globales
        auto get_global_id = [&](int lid) -> int {
            if(patch.flags[lid] == NodeFlag::NODE_RING) {
                return patch.local_to_global[lid];
            } else {
                return local_to_global[lid];
            }
        };
        
        i0 = get_global_id(e_local.a);
        i1 = get_global_id(e_local.b);
        
        // Verificar que sean válidos
        if(i0 >= 0 && i1 >= 0) {
            subdivided_edges_global.emplace(Edge(i0, i1));
        } else {
            std::cerr << "[WARNING] Invalid subdivided edge (" 
                      << e_local.a << "," << e_local.b << ") -> ("
                      << i0 << "," << i1 << ")\n";
        }
    }
        
    for(const auto& [edge_local, lid] : patch.subdivided_edge_to_node) {
        int i0 = (patch.flags[edge_local.a] == NodeFlag::NODE_RING) ? 
                 patch.local_to_global[edge_local.a] : local_to_global[edge_local.a];
        int i1 = (patch.flags[edge_local.b] == NodeFlag::NODE_RING) ? 
                 patch.local_to_global[edge_local.b] : local_to_global[edge_local.b];

        Edge e(i0, i1); // global edge

        int gid = local_to_global[lid]; // el nodo asociado sigue siendo local→global
        if(gid >= 0)
            subdivided_edge_to_global[e] = gid;
    }

    // 6) Verificar consistencia
    verify_consistency(patch);

    // Log
    std::cout << "Subdivided edges (global indices):\n";
    for(const auto& e : subdivided_edges_global) {
        std::cout << "(" << e.a << ", " << e.b << ")\n";
    }
}

int MeshToSub::add_projected_node(int lid, const TransitionPatch2D& patch) {
    const Vec2& p = patch.points[lid];
    NodeFlag flag = patch.flags[lid];
    int new_id = static_cast<int>(mesh.nodes.size());
    mesh.add_node(p.x, p.y, flag);
    return new_id;
}

void MeshToSub::add_patch_quads(const TransitionPatch2D& patch) {
    // Primero: quads regulares
    for(const auto& q : patch.quads) {
        std::array<int,4> new_ids;
        bool valid = true;
        
        for(int i = 0; i < 4; ++i) {
            int lid = q[i];
            if(patch.flags[lid] == NodeFlag::NODE_RING) {
                // ring node → índice directo de band_mesh
                new_ids[i] = patch.local_to_global[lid];
            } else {
                // projected/critical node → usar local_to_global
                new_ids[i] = local_to_global[lid];
            }
            
            if(new_ids[i] < 0) {
                std::cerr << "[ERROR] Invalid node in quad: lid=" << lid 
                          << " flag=" << static_cast<int>(patch.flags[lid])
                          << " global_id=" << new_ids[i] << "\n";
                valid = false;
                break;
            }
        }
        
        if(valid) {
            mesh.add_quad(new_ids[0], new_ids[1], new_ids[2], new_ids[3]);
        }
    }
    
    // Segundo: quads fallback (si existen)
    for(const auto& q : patch.quads_fallback) {
        std::array<int,4> new_ids;
        bool valid = true;
        
        for(int i = 0; i < 4; ++i) {
            int lid = q[i];
            if(lid < 0 || lid >= static_cast<int>(patch.points.size())) {
                std::cerr << "[ERROR] Invalid lid in fallback quad: " << lid << "\n";
                valid = false;
                break;
            }
            
            if(patch.flags[lid] == NodeFlag::NODE_RING) {
                new_ids[i] = patch.local_to_global[lid];
            } else {
                new_ids[i] = local_to_global[lid];
            }
            
            if(new_ids[i] < 0) {
                std::cerr << "[ERROR] Invalid fallback node: lid=" << lid 
                          << " flag=" << static_cast<int>(patch.flags[lid])
                          << " global_id=" << new_ids[i] << "\n";
                valid = false;
                break;
            }
        }
        
        if(valid) {
            mesh.add_quad(new_ids[0], new_ids[1], new_ids[2], new_ids[3]);
        }
    }
}

void MeshToSub::verify_consistency(const TransitionPatch2D& patch) {
    std::cout << "\n[VERIFY] MeshToSub consistency check:\n";
    std::cout << "====================================\n";
    
    // 1. Verificar local_to_global
    int missing_count = 0;
    for(size_t lid = 0; lid < patch.points.size(); ++lid) {
        if(local_to_global[lid] < 0) {
            std::cout << "  - lid=" << lid << " has invalid global_id=" 
                      << local_to_global[lid] 
                      << " flag=" << static_cast<int>(patch.flags[lid]) << "\n";
            missing_count++;
        }
    }
    
    // 2. Verificar que todos los nodos referenciados en quads existan
    int quad_errors = 0;
    for(const auto& q : patch.quads) {
        for(int i = 0; i < 4; ++i) {
            int lid = q[i];
            if(lid < 0 || lid >= static_cast<int>(patch.points.size())) {
                std::cout << "  - Quad references invalid lid=" << lid << "\n";
                quad_errors++;
            }
        }
    }
    
    // 3. Verificar tamaños
    std::cout << "  - Patch points: " << patch.points.size() << "\n";
    std::cout << "  - Mesh nodes: " << mesh.num_nodes() << "\n";
    std::cout << "  - Local_to_global size: " << local_to_global.size() << "\n";
    std::cout << "  - Missing global IDs: " << missing_count << "\n";
    std::cout << "  - Quad reference errors: " << quad_errors << "\n";
    std::cout << "====================================\n\n";
    
    if(missing_count > 0 || quad_errors > 0) {
        throw std::runtime_error("MeshToSub consistency check failed");
    }
}

// Builder de edge_map desde MeshToSub
inline std::map<Edge, EdgeInfo> build_edge_map(const MeshToSub& mesh_to_sub) {
    std::map<Edge, EdgeInfo> edge_map;

    const auto& quads = mesh_to_sub.mesh.quads;
    const auto& nodes = mesh_to_sub.mesh.nodes;

    for(size_t qid = 0; qid < quads.size(); ++qid) {
        const auto& q = quads[qid];
        
        // Verificar que el quad sea válido
        bool valid_quad = true;
        for(int i = 0; i < 4; ++i) {
            if(q[i] < 0 || q[i] >= static_cast<int>(nodes.size())) {
                std::cerr << "[ERROR] Quad " << qid << " has invalid node index: " 
                          << q[i] << "\n";
                valid_quad = false;
                break;
            }
        }
        
        if(!valid_quad) continue;
        
        for(int i = 0; i < 4; ++i) {
            int i0 = q[i];
            int i1 = q[(i+1)%4];
            
            if(i0 < 0 || i1 < 0 || i0 >= static_cast<int>(nodes.size()) || i1 >= static_cast<int>(nodes.size())) {
                std::cerr << "[ERROR] Invalid edge in quad " << qid 
                          << ": (" << i0 << "," << i1 << ")\n";
                continue;
            }
            
            Edge e(i0, i1);

            auto it = edge_map.find(e);
            if(it == edge_map.end()) {
                EdgeInfo info(nodes[i0].x, nodes[i1].x);
                info.quad_refs.emplace_back(qid, i);

                if(mesh_to_sub.subdivided_edges_global.count(e) > 0)
                    info.subdivide = true;

                // Marcar si es un edge "externo" según nodos
                if(nodes[i0].flag != NodeFlag::NODE_PROJECTED &&
                   nodes[i0].flag != NodeFlag::NODE_CRITICAL &&
                   nodes[i1].flag != NodeFlag::NODE_PROJECTED &&
                   nodes[i1].flag != NodeFlag::NODE_CRITICAL)
                {
                    info.is_external = true;
                }
    
                edge_map[e] = info;
            } else {
                it->second.quad_refs.emplace_back(qid, i);

                if(mesh_to_sub.subdivided_edges_global.count(e) > 0)
                    it->second.subdivide = true;
            }
        }
    }

    return edge_map;
}

inline void debug_edge_map(const std::map<Edge, EdgeInfo>& edge_map) {
    std::cout << "Edge map debug: (quad_id, local_edge_id)\n";
    int subdivided_count = 0;
    int shared_count = 0;
    
    for(const auto& [edge, info] : edge_map) {
        std::cout << "Edge (" << edge.a << ", " << edge.b << ")";
        
        if(info.subdivide) {
            std::cout << " [subdivided]";
            subdivided_count++;
        }
        
        if(info.quad_refs.size() > 1) {
            std::cout << " [shared]";
            shared_count++;
        }
        
        std::cout << " -> quads: ";
        for(const auto& qr : info.quad_refs) {
            std::cout << "(" << qr.first << ", " << qr.second << ") ";
        }
        std::cout << "\n";
    }
    
    std::cout << "Total edges: " << edge_map.size() << "\n";
    std::cout << "Subdivided edges: " << subdivided_count << "\n";
    std::cout << "Shared edges: " << shared_count << "\n";
}


inline void debug_subdivided_edges(const MeshToSub& mts) {
    std::cout << "\n[DEBUG] Subdivided edges (global indices)\n";
    std::cout << "-----------------------------------------\n";

    if(mts.subdivided_edges_global.empty()) {
        std::cout << "No subdivided edges.\n";
    } else {
        for(const auto& e : mts.subdivided_edges_global) {
            std::cout << "(" << e.a << ", " << e.b << ")";
            // Mostrar si existe nodo asociado en el map
            auto it = mts.subdivided_edge_to_global.find(e);
            if(it != mts.subdivided_edge_to_global.end()) {
                std::cout << " -> midpoint/global node = " << it->second;
            }
            std::cout << "\n";
        }
    }

    std::cout << "-----------------------------------------\n";
    std::cout << "Total subdivided edges: " << mts.subdivided_edges_global.size() << "\n";
    std::cout << "Total edges in edge->node map: " << mts.subdivided_edge_to_global.size() << "\n\n";
}

// Función combinada: imprime tanto set como map
inline void debug_mesh_to_sub(const MeshToSub& mts) {
    std::cout << "[DEBUG] MeshToSub info\n";
    std::cout << "Number of nodes: " << mts.num_nodes() << "\n";
    std::cout << "Number of quads: " << mts.num_quads() << "\n";

    debug_subdivided_edges(mts);

    // Imprimir nodos asociados a edges subdivididos
    std::set<int> nodes_of_subdivided_edges;
    for(const auto& e : mts.subdivided_edges_global) {
        nodes_of_subdivided_edges.insert(e.a);
        nodes_of_subdivided_edges.insert(e.b);

        auto it = mts.subdivided_edge_to_global.find(e);
        if(it != mts.subdivided_edge_to_global.end()) {
            nodes_of_subdivided_edges.insert(it->second);
        }
    }

    std::cout << "Nodes involved in subdivided edges:\n";
    for(int gid : nodes_of_subdivided_edges) {
        std::cout << "global node = " << gid << "\n";
    }

}

} // namespace mesh_adapt
