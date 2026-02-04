#pragma once
#include <vector>
#include <map>
#include <array>
#include "mesh_adapt/geometry/Edge.hpp"
#include "QuadClassifier.hpp"
#include "mesh_adapt/core/Node2D.hpp"


namespace mesh_adapt {

// Estructura para devolver resultados de subdivisión
struct SubdivisionResult {
    std::vector<Quad> new_quads;           // quads subdivididos
    std::map<Edge,int> edge_midpoints;     // Edge → índice del nodo creado en el midpoint
    std::map<int,int> quad_centers;        // QuadID → índice del nodo centro
    std::vector<Node2D> nodes;         // nodos generados (midpoints + centers)
    int max_node_index;                     // índice máximo de nodos antes de agregar nuevos
};

Mesh2D build_final_mesh(
    const SubdivisionResult& result
)
{
    Mesh2D out;

    // agregar nodos nuevos
    for(const auto& n : result.nodes)
        out.add_node(n.x.x, n.x.y);

    // agregar quads refinados
    for(const auto& q : result.new_quads)
        out.add_quad(q[0], q[1], q[2], q[3]);

    return out;
}

}
