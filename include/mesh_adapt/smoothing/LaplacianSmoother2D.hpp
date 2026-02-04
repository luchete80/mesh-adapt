#pragma once
#include <vector>
#include <unordered_map>
#include <cmath>
#include <iostream>

#include "mesh_adapt/core/Node2D.hpp"
#include "mesh_adapt/core/Mesh2D.hpp"

namespace mesh_adapt {

class LaplacianSmoother2D {
public:
    struct Params {
        double delta;
        int max_iter;
        bool verbose;

        Params(double d=1e-3, int m=50, bool v=true)
            : delta(d), max_iter(m), verbose(v) {}
    };

    // nodes      : nodos a suavizar (se modifican)
    // quads      : conectividad
    // fixed_mask : fixed_mask[i] = 1 → nodo fijo
    LaplacianSmoother2D(
        std::vector<Node2D>& nodes,
        const std::vector<Quad>& quads,
        const std::vector<char>& fixed_mask,
        const Params& params = Params()
    );

    void smooth();

//private:
    std::vector<Node2D>& nodes_;
    const std::vector<Quad>& quads_;
    const std::vector<char>& fixed_;
    Params params_;

    // nodo → vecinos
    std::unordered_map<int, std::vector<int>> adjacency_;

    void build_adjacency();
};

LaplacianSmoother2D::LaplacianSmoother2D(
    std::vector<Node2D>& nodes,
    const std::vector<Quad>& quads,
    const std::vector<char>& fixed_mask,
    const Params& params
)
: nodes_(nodes)
, quads_(quads)
, fixed_(fixed_mask)
, params_(params)
{
    build_adjacency();
}

void LaplacianSmoother2D::build_adjacency()
{
    adjacency_.clear();

    for(const auto& q : quads_) {
        for(int i = 0; i < 4; ++i) {
            int a = q[i];
            int b = q[(i+1)%4];
            adjacency_[a].push_back(b);
            adjacency_[b].push_back(a);
        }
    }
}

void LaplacianSmoother2D::smooth()
{
    std::vector<Node2D> old(nodes_.size());

    // nodos internos
    std::vector<int> internal;
    internal.reserve(adjacency_.size());

    for(const auto& kv : adjacency_) {
        int i = kv.first;
        if(!fixed_[i])
            internal.push_back(i);
    }

    if(params_.verbose)
        std::cout << "Laplacian smoothing: "
                  << internal.size() << " nodos internos\n";

    for(int it = 0; it < params_.max_iter; ++it) {
        old = nodes_;

        // -------- paso laplaciano --------
        for(int i : internal) {
            const auto& neigh = adjacency_[i];
            if(neigh.empty()) continue;

            Vec2 acc(0.0, 0.0);
            for(int j : neigh)
                acc += old[j].x;

            nodes_[i].x = acc / double(neigh.size());
        }

        // -------- convergencia --------
        double num = 0.0;
        double den = 0.0;

        for(size_t i = 0; i < nodes_.size(); ++i) {
            Vec2 d = nodes_[i].x - old[i].x;
            num += d.dot(d);
            den += old[i].x.dot(old[i].x);
        }

        double ratio = std::sqrt(num) / (std::sqrt(den) + 1e-14);

        if(params_.verbose)
            std::cout << "  iter " << it
                      << " → ratio = " << ratio << "\n";

        if(ratio < params_.delta)
            break;
    }
}

} // namespace mesh_adapt

