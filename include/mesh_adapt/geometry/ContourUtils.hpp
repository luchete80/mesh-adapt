#pragma once
#include "Vec2.hpp"
#include "mesh_adapt/core/Mesh2D.hpp"

namespace mesh_adapt {
  
struct BoundingBox2D {
    double xmin, xmax;
    double ymin, ymax;
};

inline BoundingBox2D compute_bbox(const std::vector<Vec2>& pts)
{
    BoundingBox2D box;
    box.xmin = box.xmax = pts[0].x;
    box.ymin = box.ymax = pts[0].y;

    for(const auto& p : pts) {
        box.xmin = std::min(box.xmin, p.x);
        box.xmax = std::max(box.xmax, p.x);
        box.ymin = std::min(box.ymin, p.y);
        box.ymax = std::max(box.ymax, p.y);
    }
    return box;
}

}
