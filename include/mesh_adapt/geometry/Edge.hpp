#pragma once
#include <cstddef>
#include <algorithm>

namespace mesh_adapt {

struct Edge
{
    size_t a;
    size_t b;

    Edge(size_t i, size_t j)
    {
        a = std::min(i, j);
        b = std::max(i, j);
    }

    bool operator==(const Edge& other) const
    {
        return a == other.a && b == other.b;
    }
};


inline bool operator<(const Edge& e1, const Edge& e2) {
    return std::tie(e1.a, e1.b) < std::tie(e2.a, e2.b);
}



}