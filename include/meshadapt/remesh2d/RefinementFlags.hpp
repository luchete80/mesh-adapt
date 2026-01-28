#pragma once
#include <array>

namespace mesh_adapt {

struct QuadRefinementFlag {
    // edges: 0-1,1-2,2-3,3-0
    std::array<bool,4> refine_edge{false,false,false,false};
};

}

