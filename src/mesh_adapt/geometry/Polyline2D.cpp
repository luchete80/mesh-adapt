#include "mesh_adapt/geometry/Polyline2D.hpp"
#include <limits>

namespace mesh_adapt {


double Polyline2D::distance(const Vec2& p) const {

    if (pts.size() < 2)
        return 1e30;

    double dmin = std::numeric_limits<double>::max();

    for (size_t i = 0; i < pts.size() - 1; ++i) {

        Vec2 a = pts[i];
        Vec2 b = pts[i+1];

        Vec2 ab = b - a;
        Vec2 ap = p - a;

        double t = dot(ap, ab) / norm2(ab);

        // clamp to segment
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;

        Vec2 closest = a + t * ab;

        double d = norm2(p - closest);

        if (d < dmin)
            dmin = d;
    }

    return dmin;
}

Vec2 Polyline2D::project(const Vec2& p) const {

    if (pts.size() < 2)
        return p;

    double dmin = std::numeric_limits<double>::max();
    Vec2 best = p;

    for (size_t i = 0; i < pts.size() - 1; ++i) {

        Vec2 a = pts[i];
        Vec2 b = pts[i+1];

        Vec2 ab = b - a;
        Vec2 ap = p - a;

        double t = dot(ap, ab) / norm2(ab);

        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;

        Vec2 closest = a + t * ab;
        double d = norm2(p - closest);

        if (d < dmin) {
            dmin = d;
            best = closest;
        }
    }

    return best;
}

std::vector<Vec2> Polyline2D::sample(int n) const {
    (void)n; // unused
    return pts;
}

}
