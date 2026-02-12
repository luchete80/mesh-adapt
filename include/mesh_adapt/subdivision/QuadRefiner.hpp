
#pragma once
#include "mesh_adapt/geometry/Edge.hpp"
#include "QuadClassifier.hpp"
#include "SubdivisionResult.hpp"
#include "MeshToSub.hpp"

namespace mesh_adapt {


class QuadRefiner {
public:

    std::vector<Quad>& quads_;
    std::map<Edge, EdgeInfo>& edge_map_;
    
    std::vector<QuadPattern> quad_patterns_;
    std::vector<int> quad_rotations_;
    std::map<Edge,int> initially_refined_; // edges del patch -> nodo ya creado

    MeshToSub& meshsub;  // referencia a toda la estructura
    
    
    
    QuadRefiner(MeshToSub& meshsub_, std::vector<Quad>& quads,
                std::map<Edge, EdgeInfo>& edge_map,
                const std::map<Edge,int>& initially_refined = {})
        : meshsub(meshsub_), quads_(quads), edge_map_(edge_map), initially_refined_(initially_refined) {
          
          for(const auto& kv : initially_refined_) {
    auto& ei = edge_map_[kv.first];
    add_cause(ei, RefinementCause::INITIAL);
    }

          for(const auto& kv : initially_refined_) {
          std::cout << "Init edge: (" << kv.first.a << "," << kv.first.b << ") -> " << kv.second << "\n";
        }
}

        
    void refine_to_conform(int max_iterations = 2) {
      
        quad_patterns_.resize(quads_.size(), PAT_NONE);
        quad_rotations_.resize(quads_.size(), 0);
    
        int prev_n_subdivide=0;
        bool end = false;
        int it = 0;
        //for(int it=0; it<2; ){ //// ONLY FOR DEBUG
        while (!end ){
            std::cout << "\n--- Iteration " << it << " ---\n";

            int n_subdivide = count_edges_to_subdivide();
            std::cout << "Edges marked for subdivision: " << n_subdivide << "\n";
            //if(n_subdivide == 0) break;


            //classify_quads();
            debug_quad(1422);
            // Marcar edges según patrones
            mark_edges_for_subdivision();
            debug_nonconforming_edges();
            // THIS DOES NNOT 
            //~ if (!has_nonconforming_edge())
              //~ break;

            if(n_subdivide == prev_n_subdivide) {
              if (!has_nonconforming_edge())
              end = true;
            }
            prev_n_subdivide = n_subdivide;
            
            // Subdividir quads
            //subdivide_quads();

            // Actualizar edge_map si es necesario
            //update_edge_map();
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


    bool neighbor_is_refined(const Edge& e) const
    {
        auto it = edge_map_.find(e);
        if(it == edge_map_.end()) return false;

        for(const auto& [qid, local_e] : it->second.quad_refs)
            if(quad_patterns_[qid] != PAT_NONE)
                return true;

        return false;
    }

    bool is_left_neighbor_refined(size_t qid, int rot) const {
        const auto& quad = quads_[qid];
        Edge eL(quad[(rot + 1) % 4], quad[rot]);
        auto it = edge_map_.find(eL);
        return it != edge_map_.end() && it->second.subdivide;
    }

    bool is_right_neighbor_refined(size_t qid, int rot) const {
        const auto& quad = quads_[qid];
        Edge eR(quad[rot], quad[(rot + 3) % 4]);
        auto it = edge_map_.find(eR);
        return it != edge_map_.end() && it->second.subdivide;
    }

    bool is_opposite_refined(size_t qid, int rot) const
    {
        const auto& quad = quads_[qid];
        Edge opp(
            quad[(rot+2)%4],
            quad[(rot+3)%4]
        );

        auto it = edge_map_.find(opp);
        return (it != edge_map_.end() && it->second.subdivide);
    }

    bool is_initial_edge(const Edge& e) const {
        auto it = edge_map_.find(e);
        return it != edge_map_.end() && has_cause(it->second, RefinementCause::INITIAL);
    }

inline bool is_external_node(int nid) const {
    return meshsub.mesh.nodes[nid].is_external();
}
inline int count_external_nodes(const Edge& e) const {
    return is_external_node(e.a) + is_external_node(e.b);
}

void debug_nonconforming_edges() const
{
    for(const auto& [e, info] : edge_map_)
    {
        if(!info.subdivide) continue;

        int refined_quads = 0;

        for(const auto& [qid, _] : info.quad_refs)
        {
            if(quad_patterns_[qid] != PAT_NONE)
                refined_quads++;
        }

        if(refined_quads == 1 && info.quad_refs.size() == 2)
        {
            std::cout << "[NON-CONFORMING EDGE] ("
                      << e.a << "," << e.b << ") "
                      << " refined_quads=" << refined_quads
                      << " total_quads=" << info.quad_refs.size()
                      << "\n";
        }
    }
}

bool has_nonconforming_edge() const
{
    for(const auto& [e, info] : edge_map_)
    {
        if(!info.subdivide) continue;

        int refined_quads = 0;

        for(const auto& [qid, _] : info.quad_refs)
            if(quad_patterns_[qid] != PAT_NONE)
                refined_quads++;

        if(refined_quads == 1 && info.quad_refs.size() == 2)
            return true;
    }
    return false;
}


void mark_edges_for_subdivision()
{
    for(size_t qid = 0; qid < quads_.size(); ++qid)
    {
        QuadPattern pat;
        int rot;
       
        //std::cout<< "Classifying quad "<<qid<<std::endl; 
        classify_quad(quads_[qid], edge_map_, pat, rot);

        // cache opcional
        quad_patterns_[qid]  = pat;
        quad_rotations_[qid] = rot;

        if(pat != PAT_ONE && pat != PAT_THREE)
            continue;

        auto& quad = quads_[qid];

        std::array<Edge,4> edges = {
            Edge(quad[0], quad[1]),
            Edge(quad[1], quad[2]),
            Edge(quad[2], quad[3]),
            Edge(quad[3], quad[0])
        };

        if(pat == PAT_ONE)
        {
            // edge refinado
            Edge& e = edges[rot];

            // vecinos
            Edge& eL = edges[(rot + 1) % 4];
            Edge& eR = edges[(rot + 3) % 4];

            // política simple (ejemplo)
            if(edge_map_[eL].is_external && !edge_map_[eR].is_external)
                edge_map_[eL].subdivide = true;
            else
                edge_map_[eR].subdivide = true;

            ///// NEW LOGIC
    
            //~ Edge& e = edges[rot];
            //~ Edge& eR = edges[(rot+3)%4];
            //~ Edge& eL = edges[(rot+1)%4];

            //~ const Node2D& n0 = meshsub.mesh.nodes[e.a];
            //~ const Node2D& n1 = meshsub.mesh.nodes[e.b];
            
            //~ if(is_initial_edge(e))
            //~ {   

                //~ std::cout << "SUBDIVISION"<< ") INITIAL edge=(" << e.a << "," << e.b<<")"<<std::endl;
                //~ if(edge_map_[eL].is_external && !edge_map_[eR].is_external)
                    //~ std::cout << "SUBDIVISION"<< ") NB LEFT EXTERNAL =(" << eL.a << "," << eL.b<<")"<<std::endl;
                //~ else if(edge_map_[eR].is_external && !edge_map_[eL].is_external)
                    //~ std::cout << "SUBDIVISION"<< ") NB RIGHT EXTERNAL =(" << eR.a << "," << eR.b<<")"<<std::endl;
                //~ else if(edge_map_[eL].is_external && edge_map_[eR].is_external)
                    //~ std::cout << "SUBDIVISION L& R INTERNAL "<< std::endl;

                //~ //if(!edge_map_[eL].is_external){
                //~ if(count_external_nodes(e)==1){
                   //~ if(n0.is_external())
                       //~ edge_map_[eL].subdivide = true; 
                    //~ else
                       //~ edge_map_[eR].subdivide = true;    
                //~ }


                

                //~ initially_refined_.erase(e);
            //~ } else {

                //~ if(edge_map_[eL].is_external)
                    //~ edge_map_[eL].subdivide = true; 
                //~ else if(edge_map_[eR].is_external)
                    //~ edge_map_[eR].subdivide = true; 
                //~ else {
                //~ // fallback geométrico o rotacional
                     //~ edge_map_[eL].subdivide = true;
                //~ }
                    
                  
                            

        }
        else if(pat == PAT_THREE)
        {
            // el único no refinado
            for(int i = 0; i < 4; ++i)
            {
                if(!edge_map_[edges[i]].subdivide)
                {
                    edge_map_[edges[i]].subdivide = true;
                    break;
                }
            }
        }
    }
}
    

void debug_quad(int qid) const
{
    std::cout << "Quad " << qid << "\n";

    std::cout << "  Pattern: " << quad_patterns_[qid] << "\n";
    std::cout << "  Rotation: " << quad_rotations_[qid] << "\n";

    const auto& q = quads_[qid];
    std::cout << "  Nodes: "
              << q[0] << " "
              << q[1] << " "
              << q[2] << " "
              << q[3] << "\n";
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
        else if(pat == PAT_TWO_ADJ_LEFT) {
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
        else if(pat == PAT_TWO_ADJ_RIGHT) {
            std::array<int,4> q;
            for(int i=0;i<4;++i) q[i] = quad[(i + rot)%4];
            int a=q[0], b=q[1], c=q[2], d=q[3];

            int m_ab = get_edge_midpoint(a,b);
            int m_da = get_edge_midpoint(d,a);
            int cc   = get_quad_center();

            result.new_quads.push_back({a, m_ab, cc, m_da});
            result.new_quads.push_back({m_ab, b, c, cc});
            result.new_quads.push_back({m_da, cc, c, d});
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

void add_refined_quads(const std::set<int>& qids,
                       QuadPattern pat,
                       RefinementCause cause)
{
    for(int qid : qids) {
        // Marcar patrón del quad
        quad_patterns_[qid] = pat;

        // Propagar cause a los edges del quad
        const auto& quad = quads_[qid];
        for(int i=0;i<4;++i) {
            Edge e(quad[i], quad[(i+1)%4]);
            add_cause(edge_map_[e], cause);
        }
    }
}


};

}
