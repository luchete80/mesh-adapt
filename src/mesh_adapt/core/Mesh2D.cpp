#include "mesh_adapt/core/Mesh2D.hpp"
#include <map>
#include <set>

namespace mesh_adapt {

int Mesh2D::add_node(double x, double y) {
    nodes.emplace_back(x,y); //more efficent thgan nodes.push_back(Node2D(x, y));
    return (int)nodes.size() - 1;
}

void Mesh2D::add_quad(int a, int b, int c, int d)
{
    quads.push_back({a, b, c, d});
}

size_t Mesh2D::num_nodes() const
{
    return (int)nodes.size();
}

size_t Mesh2D::num_quads() const
{
    return (int)quads.size();
}

std::vector<int> Mesh2D::find_boundary_nodes() const {

    using Edge = std::pair<int,int>;

    std::map<Edge,int> edge_count;

    auto add_edge = [&](int a, int b) {
        if(a > b) std::swap(a,b);
        edge_count[{a,b}]++;
    };

    // contar aristas
    for(const auto& q : quads) {

        add_edge(q[0], q[1]);
        add_edge(q[1], q[2]);
        add_edge(q[2], q[3]);
        add_edge(q[3], q[0]);
    }

    // nodos frontera = edges con count=1
    std::set<int> boundary_nodes;

    for(auto& [e,count] : edge_count) {
        if(count == 1) {
            boundary_nodes.insert(e.first);
            boundary_nodes.insert(e.second);
        }
    }

    return std::vector<int>(
        boundary_nodes.begin(),
        boundary_nodes.end()
    );
}


std::vector<int> Mesh2D::find_ordered_boundary_nodes() const {
    using Edge = std::pair<int,int>;
    std::map<Edge,int> edge_count;
    auto add_edge = [&](int a, int b){ if(a>b) std::swap(a,b); edge_count[{a,b}]++; };

    // contar aristas
    for(const auto& q : quads) {
        add_edge(q[0],q[1]);
        add_edge(q[1],q[2]);
        add_edge(q[2],q[3]);
        add_edge(q[3],q[0]);
    }

    // construir vecinos de nodos frontera
    std::map<int,std::vector<int>> neighbors;
    for(auto& [e,count]: edge_count){
        if(count==1){
            neighbors[e.first].push_back(e.second);
            neighbors[e.second].push_back(e.first);
        }
    }

    std::vector<int> ordered;
    if(neighbors.empty()) return ordered;

    // empezar desde un nodo cualquiera
    int start = neighbors.begin()->first;
    int prev = -1;
    int current = start;
    do {
        ordered.push_back(current);
        auto& nbs = neighbors[current];
        int next = (nbs[0]!=prev) ? nbs[0] : nbs[1];
        prev = current;
        current = next;
    } while(current != start && ordered.size()<neighbors.size());

    return ordered;
}



} // namespace mesh_adapt
