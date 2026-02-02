
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

inline bool triangle_in_transition_region(
    const std::array<int,3>& tri,
    const std::vector<Vec2>& pts,
    const Polyline2D& outer,
    const Polyline2D& inner
)
{
    Vec2 c = (pts[tri[0]] + pts[tri[1]] + pts[tri[2]]) * (1.0 / 3.0);

    if(!point_in_polygon(c, outer.get_points()))
        return false;

    if(point_in_polygon(c, inner.get_points()))
        return false;

    return true;
}


}
