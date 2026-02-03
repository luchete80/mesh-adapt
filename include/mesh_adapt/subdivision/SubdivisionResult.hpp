#pragma once
#include <vector>
#include <map>
#include <array>
#include "mesh_adapt/geometry/Edge.hpp"
#include "QuadClassifier.hpp"


namespace mesh_adapt {

// Estructura para devolver resultados de subdivisión
struct SubdivisionResult {
    std::vector<Quad> new_quads;           // quads subdivididos
    std::map<Edge,int> edge_midpoints;     // Edge → índice del nodo creado en el midpoint
    std::map<int,int> quad_centers;        // QuadID → índice del nodo centro
    int max_node_index;                     // índice máximo de nodos antes de agregar nuevos
};

}
