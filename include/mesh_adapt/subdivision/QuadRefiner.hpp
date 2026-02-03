#pragma once
#include "mesh_adapt/geometry/Edge.hpp"
#include "QuadClassifier.hpp"
#include "SubdivisionResult.hpp"

namespace mesh_adapt {


class QuadRefiner {
public:
    QuadRefiner(std::vector<Quad>& quads,
                std::map<Edge, EdgeInfo>& edge_map)
        : quads_(quads), edge_map_(edge_map) {}

    void refine_to_conform(int max_iterations = 10) {
        for(int it=0; it<max_iterations; ++it){
            std::cout << "\n--- Iteration " << it << " ---\n";

            int n_subdivide = count_edges_to_subdivide();
            std::cout << "Edges marked for subdivision: " << n_subdivide << "\n";
            if(n_subdivide == 0) break;

            classify_quads();

            // Marcar edges según patrones
            mark_edges_for_subdivision();

            // Subdividir quads
            subdivide_quads();

            // Actualizar edge_map si es necesario
            update_edge_map();
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
    
    // -----------------------------------------
    // Función auxiliar que genera los quads nuevos
    // Subdivide quads y devuelve información para generar nodos nuevos
    SubdivisionResult subdivide_quads_with_indices(int max_node_idx) const {
        SubdivisionResult result;
        result.max_node_index = max_node_idx;

        for(size_t qid=0; qid<quads_.size(); ++qid){
            const auto& quad = quads_[qid];
            QuadPattern pat = quad_patterns_[qid];
            int rot = quad_rotations_[qid];

            int i0 = quad[0], i1 = quad[1], i2 = quad[2], i3 = quad[3];

            // --- Helpers ---
            auto get_edge_midpoint = [&](int a, int b) -> int {
                Edge e(a,b);
                if(result.edge_midpoints.count(e)) return result.edge_midpoints[e];
                result.max_node_index++;
                result.edge_midpoints[e] = result.max_node_index;
                return result.max_node_index;
            };

            auto get_quad_center = [&]() -> int {
                if(result.quad_centers.count(qid)) return result.quad_centers[qid];
                result.max_node_index++;
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

    //~ int edge_index(int a, int b) const {
        //~ Edge e(a,b);
        //~ auto it = edge_map_.find(e);
        //~ if(it != edge_map_.end()) return std::distance(edge_map_.begin(), it);
        //~ throw std::runtime_error("Edge not found");
    //~ }
};

}
