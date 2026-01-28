// src/mesh_adapt/boundary/Boundary2D.cpp
#include "mesh_adapt/boundary/Boundary2D.hpp"
#include <map>
#include <set>

namespace mesh_adapt {

void Boundary2D::build_transition_points(
    const std::vector<Vec2>& mesh_nodes,
    const std::vector<Vec2>& proj_nodes,
    std::vector<Vec2>& trans_points,
    std::vector<size_t>& trans_to_global
) const {
    trans_points.clear();
    trans_to_global.clear();
    
    // 1. Agregar boundary nodes del grid
    for(const auto& bn : boundary_nodes_) {
        trans_points.push_back(mesh_nodes[bn.mesh_idx]);
        trans_to_global.push_back(bn.mesh_idx);
    }
    
    // 2. Agregar projected nodes (NUEVOS nodos)
    size_t new_node_start = mesh_nodes.size();
    for(size_t i = 0; i < proj_nodes.size(); i++) {
        trans_points.push_back(proj_nodes[i]);
        trans_to_global.push_back(new_node_start + i);
    }
    
    // Nota: los nodos del contour se pueden agregar opcionalmente
    // según init_boundary_points() del Python
}

void Boundary2D::find_boundary_nodes_from_quads(
    const std::vector<Vec2>& nodes,
    const std::vector<Quad>& quads
) {
    // Contar edges compartidos
    std::map<std::pair<int,int>, int> edge_count;
    
    for(const auto& q : quads) {
        for(int i = 0; i < 4; i++) {
            int n0 = q[i];
            int n1 = q[(i+1) % 4];
            
            auto edge = std::make_pair(
                std::min(n0, n1),
                std::max(n0, n1)
            );
            edge_count[edge]++;
        }
    }
    
    // Nodos en edges con count == 1 son boundary
    std::set<int> boundary_set;
    for(const auto& [edge, count] : edge_count) {
        if(count == 1) {
            boundary_set.insert(edge.first);
            boundary_set.insert(edge.second);
        }
    }
    
    boundary_nodes_.clear();
    for(int idx : boundary_set) {
        boundary_nodes_.emplace_back(idx, NODE_INTERIOR);
    }
}

std::vector<Vec2> Boundary2D::generate_projected_nodes(
    const std::vector<Vec2>& mesh_nodes,
    const ProjectionParams& params
) const {
    std::vector<Vec2> proj_nodes;
    
    for(const auto& bn : boundary_nodes_) {
        Vec2 p = mesh_nodes[bn.mesh_idx];
        
        // Proyección geométrica
        Vec2 proj = project_single(p);
        double dist = distance(p, proj);
        
        // Filtro C: distancia máxima
        if(dist > params.rmax) {
            continue;
        }
        
        // Filtro A: evitar nodos proyectados muy cercanos
        bool too_close = false;
        for(const auto& pk : proj_nodes) {
            if(distance(pk, proj) < params.rmin) {
                too_close = true;
                break;
            }
        }
        if(too_close) continue;
        
        proj_nodes.push_back(proj);
    }
    
    return proj_nodes;
}

Vec2 Boundary2D::project_single(const Vec2& p) const {
    return contour_.project(p);
}

double Boundary2D::distance_to_contour(const Vec2& p) const {
    return contour_.distance(p);
}

std::vector<Vec2> Boundary2D::get_contour_points() const {
    // Asumiendo que Polyline2D tiene método get_points()
    // Si no, necesitas agregarlo a Polyline2D
    return contour_.get_points();  
}

} // namespace mesh_adapt
