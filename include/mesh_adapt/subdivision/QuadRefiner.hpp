#pragma once
#include "mesh_adapt/geometry/Edge.hpp"
#include "QuadClassifier.hpp"
#include "SubdivisionResult.hpp"

namespace mesh_adapt {


class QuadRefiner {
public:
    QuadRefiner(std::vector<Quad>& quads,
                std::map<Edge, EdgeInfo>& edge_map,
                const std::map<Edge,int>& initially_refined = {})
        : quads_(quads), edge_map_(edge_map), initially_refined_(initially_refined) {
          
          for(const auto& kv : initially_refined_) {
    std::cout << "Init edge: (" << kv.first.a << "," << kv.first.b << ") -> " << kv.second << "\n";
}
}

        
    void refine_to_conform(int max_iterations = 15) {
        int prev_n_subdivide=0;
        bool end = false;
        int it = 0;
        //for(int it=0; it<max_iterations; ++it){
        while (!end){
            std::cout << "\n--- Iteration " << it << " ---\n";

            int n_subdivide = count_edges_to_subdivide();
            std::cout << "Edges marked for subdivision: " << n_subdivide << "\n";
            //if(n_subdivide == 0) break;
            if(n_subdivide == prev_n_subdivide) end = true;
            prev_n_subdivide = n_subdivide;

            classify_quads();

            // Marcar edges según patrones
            mark_edges_for_subdivision();

            // Subdividir quads
            subdivide_quads();

            // Actualizar edge_map si es necesario
            update_edge_map();
            it ++;
        }
    }

    void debug_edge_map() const {
        for(const auto& [e, info] : edge_map_){
            std::cout << "Edge (" << e.a << ", " << e.b << ") subdiv=" << info.subdivide 
                      << " quads: ";
            for(auto& qr : info.quad_refs) 
                std::cout << "(" << qr.first << "," << qr.second << ") ";
            std::cout << "\n";
        }
    }

//private:
    std::vector<Quad>& quads_;
    std::map<Edge, EdgeInfo>& edge_map_;
    
    std::vector<QuadPattern> quad_patterns_;
    std::vector<int> quad_rotations_;
    //std::set<Edge> initially_refined_; // edges que ya tienen nodos
    std::map<Edge,int> initially_refined_; // edges del patch -> nodo ya creado
    

    int get_max_node_index() const {
        int max_idx = -1;
        for(const auto& q : quads_){
            for(int i=0;i<4;++i)
                if(q[i] > max_idx) max_idx = q[i];
        }
        return max_idx;
    }

    int count_edges_to_subdivide() const {
        int count = 0;
        for(auto& [_, info] : edge_map_) if(info.subdivide) ++count;
        return count;
    }

    void classify_quads() {
        QuadClassifier qc(quads_, edge_map_);
        qc.classify(quad_patterns_, quad_rotations_);
    }

    void mark_edges_for_subdivision() {
        for(size_t qid=0; qid<quads_.size(); ++qid){
            auto& quad = quads_[qid];
            int rot = quad_rotations_[qid];
            QuadPattern pat = quad_patterns_[qid];


            // Inicializar los 4 edges del quad directamente
            std::array<Edge,4> edges = {
                Edge(quad[0], quad[1]),
                Edge(quad[1], quad[2]),
                Edge(quad[2], quad[3]),
                Edge(quad[3], quad[0])
            };

            // Marcar edges según patrón (similar a tu código Python)
            switch(pat){
                case PAT_ONE:
                    edge_map_[edges[rot]].subdivide = true;
                    edge_map_[edges[(rot+1)%4]].subdivide = true;
                    break;
                case PAT_TWO_ADJ:
                    edge_map_[edges[rot]].subdivide = true;
                    edge_map_[edges[(rot+1)%4]].subdivide = true;
                    break;
                case PAT_TWO_OPP:
                    edge_map_[edges[rot]].subdivide = true;
                    edge_map_[edges[(rot+2)%4]].subdivide = true;
                    break;
                case PAT_THREE:
                case PAT_FULL:
                    for(int i=0;i<4;++i) edge_map_[edges[i]].subdivide = true;
                    break;
                default: break;
            }
        }
    }

    void subdivide_quads() {
        // Aquí implementas la subdivisión real de quads y actualización de quads_
    }

    void update_edge_map() {
        // Actualizar los EdgeInfo de edges nuevos creados
    }
    

SubdivisionResult subdivide_quads_with_nodes(const std::vector<Node2D>& nodes) const {
    SubdivisionResult result;

    // Copiamos los nodos originales
    result.nodes = nodes;
    result.max_node_index = static_cast<int>(nodes.size() - 1); // último índice válido

    for(size_t qid = 0; qid < quads_.size(); ++qid) {
        const auto& quad = quads_[qid];
        QuadPattern pat = quad_patterns_[qid];
        int rot = quad_rotations_[qid];

        int i0 = quad[0], i1 = quad[1], i2 = quad[2], i3 = quad[3];

        // --- Helpers ---
        auto get_edge_midpoint = [&](int a, int b) -> int {
            Edge e(a,b);

            // Primero revisamos si el edge ya tenía nodo creado (map)
            auto it_init = initially_refined_.find(e);
            if(it_init != initially_refined_.end()) {
                int idx = it_init->second;
                result.edge_midpoints[e] = idx; // asegurar que quede en edge_midpoints
                std::cout << "[INFO] Edge (" << e.a << "," << e.b << ") already has node " << idx << "\n";
                return idx;
            }

            // Si ya se creó en esta iteración
            auto it_mid = result.edge_midpoints.find(e);
            if(it_mid != result.edge_midpoints.end()) return it_mid->second;

            // Crear nodo nuevo en el medio
            Node2D midpoint = 0.5*(result.nodes[a] + result.nodes[b]);
            result.nodes.push_back(midpoint);

            result.max_node_index = static_cast<int>(result.nodes.size() - 1);
            result.edge_midpoints[e] = result.max_node_index;
            return result.max_node_index;
        };

        auto get_quad_center = [&]() -> int {
            if(result.quad_centers.count(qid)) return result.quad_centers[qid];

            Node2D center = 0.25*(result.nodes[i0] + result.nodes[i1] + result.nodes[i2] + result.nodes[i3]);
            result.nodes.push_back(center);

            result.max_node_index = static_cast<int>(result.nodes.size() - 1);
            result.quad_centers[qid] = result.max_node_index;
            return result.max_node_index;
        };

        // ==========================
        if(pat == PAT_NONE) {
            result.new_quads.push_back(quad);
        }
        else if(pat == PAT_TWO_OPP) {
            std::array<int,4> q;
            for(int i=0;i<4;++i) q[i] = quad[(i + rot)%4];
            int a=q[0], b=q[1], c=q[2], d=q[3];

            int m_ab = get_edge_midpoint(a,b);
            int m_cd = get_edge_midpoint(c,d);

            result.new_quads.push_back({a, m_ab, m_cd, d});
            result.new_quads.push_back({m_ab, b, c, m_cd});
        }
        else if(pat == PAT_TWO_ADJ) {
            std::array<int,4> q;
            for(int i=0;i<4;++i) q[i] = quad[(i + rot)%4];
            int a=q[0], b=q[1], c=q[2], d=q[3];

            int m_ab = get_edge_midpoint(a,b);
            int m_bc = get_edge_midpoint(b,c);
            int cc   = get_quad_center();

            result.new_quads.push_back({a, m_ab, cc, d});
            result.new_quads.push_back({m_ab, b, m_bc, cc});
            result.new_quads.push_back({d, cc, m_bc, c});
        }
        else if(pat == PAT_THREE || pat == PAT_FULL) {
            int m01 = get_edge_midpoint(i0,i1);
            int m12 = get_edge_midpoint(i1,i2);
            int m23 = get_edge_midpoint(i2,i3);
            int m30 = get_edge_midpoint(i3,i0);
            int c   = get_quad_center();

            result.new_quads.push_back({i0, m01, c,   m30});
            result.new_quads.push_back({m01, i1, m12, c  });
            result.new_quads.push_back({c,   m12, i2, m23});
            result.new_quads.push_back({m30, c,   m23, i3});
        }
        else {
            result.new_quads.push_back(quad);
        }
    }

    return result;
}


};

}
