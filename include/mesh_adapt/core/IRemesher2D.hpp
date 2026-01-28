#pragma once
#include "Mesh2D.hpp"

namespace mesh_adapt {

class IRemesher2D {
public:
    virtual ~IRemesher2D() = default;

    virtual void remesh(Mesh2D& mesh) = 0;
};

}

