#pragma once
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <limits>
#include <algorithm>
#include "mesh_adapt/core/Mesh2D.hpp"
#include "mesh_adapt/geometry/Polyline2D.hpp"
#include "mesh_adapt/geometry/Edge.hpp"

namespace mesh_adapt {

// =========================
// Edge representado como par ordenado (menor_idx, mayor_idx)
// =========================
// using Edge = std::pair<size_t, size_t>;
// inline Edge make_edge(size_t i, size_t j) {
    // return { std::min(i,j), std::max(i,j) };
// }

// =========================
// Hash para Edge (para usar en unordered_map)
// =========================
struct EdgeHash {
    std::size_t operator()(const Edge& e) const noexcept {
        return std::hash<size_t>{}(e.a) ^ (std::hash<size_t>{}(e.b) << 1);
    }
};

// =========================
// Extraer nodos de frontera ordenados a partir de Mesh2D
// =========================
inline std::vector<size_t> extract_boundary_nodes(const Mesh2D& mesh) {
    const auto& quads = mesh.get_quads();
    std::unordered_map<Edge,int,EdgeHash> edge_count;

    // Contar cuántas veces aparece cada edge
    for(const auto& q : quads) {
        edge_count[Edge(q[0], q[1])]++;
        edge_count[Edge(q[1], q[2])]++;
        edge_count[Edge(q[2], q[3])]++;
        edge_count[Edge(q[3], q[0])]++;
    }

    // Extraer edges que solo aparecen una vez (frontera)
    std::vector<Edge> boundary_edges;
    for(const auto& [e,c] : edge_count)
        if(c == 1) boundary_edges.push_back(e);

    // Conjunto de nodos frontera
    std::unordered_set<size_t> boundary_set;
    for(auto &e : boundary_edges) {
        boundary_set.insert(e.a);
        boundary_set.insert(e.b);
    }

    // ---------------- Ordenar nodos ----------------
    std::vector<size_t> ordered_nodes;
    if(boundary_set.empty()) return ordered_nodes;

    size_t current = *boundary_set.begin();
    ordered_nodes.push_back(current);

    // Conjunto de nodos ya usados
    std::unordered_set<size_t> used_nodes;
    used_nodes.insert(current);

    while(ordered_nodes.size() < boundary_set.size()) {
        size_t next_node = current;

        // Buscar vecino conectado no usado
        for(auto &e : boundary_edges) {
            size_t candidate = SIZE_MAX;
            if(e.a == current) candidate = e.b;
            else if(e.b == current) candidate = e.a;
            else continue;

            if(used_nodes.count(candidate)) continue;

            next_node = candidate;
            break;
        }

        if(next_node == current) break; // no hay vecino válido, borde terminado
        ordered_nodes.push_back(next_node);
        used_nodes.insert(next_node);
        current = next_node;
    }

    // Cerrar el loop si es un contorno cerrado
    if(ordered_nodes.size() > 1 && ordered_nodes.front() != ordered_nodes.back())
        ordered_nodes.push_back(ordered_nodes.front());

    return ordered_nodes;
}

// =========================
// Polyline2D from Mesh2D
// =========================
inline Polyline2D build_polyline_from_mesh(const Mesh2D& mesh) {
    std::vector<size_t> boundary_idx = extract_boundary_nodes(mesh);
    std::vector<Vec2> pts;
    for(size_t i : boundary_idx) pts.push_back(mesh.node(i));

    // Cerrar loop si no está cerrado
    if(pts.front() != pts.back())
        pts.push_back(pts.front());

    // Calcular área firmada (shoelace)
    double area = 0.0;
    for(size_t i = 0; i < pts.size() - 1; ++i) {
        area += (pts[i].x * pts[i+1].y) - (pts[i+1].x * pts[i].y);
    }

    // Si área < 0, es CW, invertimos para CCW
    if(area < 0.0) {
        std::reverse(pts.begin(), pts.end());
    }

    return Polyline2D(pts);
}


} // namespace mesh_adapt
