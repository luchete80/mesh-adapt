#pragma once
#include "../core/IRemesher2D.hpp"

namespace mesh_adapt {

class MartinsRemesher2D : public IRemesher2D {
public:
    void remesh(Mesh2D& mesh) override;

private:
    void refine_quad(Mesh2D& mesh, int quad_id);
};

}

