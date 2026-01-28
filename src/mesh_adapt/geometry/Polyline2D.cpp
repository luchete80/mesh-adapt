#include "mesh_adapt/geometry/Polyline2D.hpp"
#include <algorithm>
#include <limits>
#include <cmath>

namespace mesh_adapt {

double Polyline2D::distance(const Vec2& p) const {
    double min_dist = std::numeric_limits<double>::max();
    
    for(size_t i = 0; i + 1 < pts.size(); i++) {
        Vec2 a = pts[i];
        Vec2 b = pts[i+1];
        Vec2 ab = b - a;
        
        double denom = dot(ab, ab);
        if(denom < 1e-14) {  // segmento degenerado
            double d = ::mesh_adapt::distance(p, a);
            min_dist = std::min(min_dist, d);
            continue;
        }
        
        double t = dot(p - a, ab) / denom;
        t = std::clamp(t, 0.0, 1.0);
        
        Vec2 closest = a + t * ab;
        double d = ::mesh_adapt::distance(p, closest);
        min_dist = std::min(min_dist, d);
    }
    
    return min_dist;
}

Vec2 Polyline2D::project(const Vec2& p) const {
    Vec2 closest_point = pts[0];  // ← Initializer with first point
    double min_dist = std::numeric_limits<double>::max();
    
    for(size_t i = 0; i + 1 < pts.size(); i++) {
        Vec2 a = pts[i];
        Vec2 b = pts[i+1];
        Vec2 ab = b - a;
        
        double denom = dot(ab, ab);
        if(denom < 1e-14) {  // segmento degenerado
            double d = ::mesh_adapt::distance(p, a);
            if(d < min_dist) {
                min_dist = d;
                closest_point = a;
            }
            continue;
        }
        
        double t = dot(p - a, ab) / denom;
        t = std::clamp(t, 0.0, 1.0);
        
        Vec2 proj = a + t * ab;
        double d = ::mesh_adapt::distance(p, proj);
        
        if(d < min_dist) {
            min_dist = d;
            closest_point = proj;
        }
    }
    
    return closest_point;
}


std::vector<Vec2> Polyline2D::sample(int n) const {
    (void)n; // unused
    return pts;
}

}
