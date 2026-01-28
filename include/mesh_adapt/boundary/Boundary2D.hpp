#pragma once
#include <vector>
#include "mesh_adapt/geometry/Vec2.hpp"
#include "mesh_adapt/geometry/Polyline2D.hpp"
#include "mesh_adapt/core/Mesh2D.hpp"  // para Quad

namespace mesh_adapt {

// Flags de nodos (del código Python)
enum NodeFlag {
    NODE_PROJECTED = 0,
    NODE_CRITICAL = 1,   // nodos del contorno original
    NODE_INTERIOR = 2,
    NODE_AXIS = 3,
    NODE_EXTERNAL = 4,
    NODE_SUBDIVIDED = 5
};

struct BoundaryNode {
    size_t mesh_idx;      // índice en el array global de nodos
    NodeFlag flag;
    
    BoundaryNode(size_t idx, NodeFlag f = NODE_INTERIOR) 
        : mesh_idx(idx), flag(f) {}
};

// Parámetros de Martins para proyección
struct ProjectionParams {
    double SL;      // spacing length
    double rmin;    // 0.5 * SL
    double rmax;    // 1.6 * SL
    
    explicit ProjectionParams(double spacing_length)
        : SL(spacing_length)
        , rmin(0.5 * spacing_length)
        , rmax(1.6 * spacing_length)
    {}
};

class Boundary2D {
public:
    Boundary2D() = default;
    Boundary2D(const Polyline2D& c) 
        : contour_(c) 
    {}  
    // ============================================
    // PASO 1: Identificar nodos de borde (TOPOLÓGICO)
    // ============================================
    // Python: find_boundary_nodes()
    void find_boundary_nodes_from_quads(
        const std::vector<Vec2>& nodes,
        const std::vector<Quad>& quads
    );
    
    // ============================================
    // PASO 2: Generar nodos proyectados (GEOMÉTRICO)
    // ============================================
    // Python: generate_projection_nodes()
    std::vector<Vec2> generate_projected_nodes(
        const std::vector<Vec2>& mesh_nodes,
        const ProjectionParams& params
    ) const;
    
    // ============================================
    // PASO 3: Construir conjunto de puntos de transición
    // ============================================
    // Python: build_transition_point_set_with_idx()
    // Combina: boundary_nodes + projected_nodes + contour_nodes
    void build_transition_points(
        const std::vector<Vec2>& mesh_nodes,
        const std::vector<Vec2>& proj_nodes,
        std::vector<Vec2>& trans_points,        // OUTPUT
        std::vector<size_t>& trans_to_global    // OUTPUT: mapeo a mesh global
    ) const;
    
    // ============================================
    // PASO 4: Filtrado (opcional)
    // ============================================
    // Python: remove_axis_nodes()
    void remove_axis_nodes(
        const std::vector<Vec2>& mesh_nodes,
        double axis_tol = 1e-12
    );
    
    // Python: remove_projected_too_close()
    static void filter_close_projections(
        std::vector<Vec2>& proj_nodes,
        const std::vector<Vec2>& critical_nodes,
        double rmin
    );
    
    // ============================================
    // Acceso a datos
    // ============================================
    const std::vector<BoundaryNode>& get_boundary_nodes() const { 
        return boundary_nodes_; 
    }
    
    const Polyline2D& get_contour() const { 
        return contour_; 
    }
    
    std::vector<Vec2> get_contour_points() const;

private:
    Polyline2D contour_;
    std::vector<BoundaryNode> boundary_nodes_;
    
    // Helper: proyecta UN punto al contour
    Vec2 project_single(const Vec2& p) const;
    double distance_to_contour(const Vec2& p) const;
};

} // namespace mesh_adapt
