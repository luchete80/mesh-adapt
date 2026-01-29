
#pragma once
#include "Vec2.hpp"


namespace mesh_adapt {
inline bool point_in_polygon(const Vec2& p,
                             const std::vector<Vec2>& poly)
{
    bool inside = false;
    int n = poly.size();

    for(int i=0, j=n-1; i<n; j=i++) {
        const auto& pi = poly[i];
        const auto& pj = poly[j];

        bool intersect =
            ((pi.y > p.y) != (pj.y > p.y)) &&
            (p.x < (pj.x - pi.x) * (p.y - pi.y) /
                   (pj.y - pi.y + 1e-14) + pi.x);

        if(intersect)
            inside = !inside;
    }

    return inside;
}


}
