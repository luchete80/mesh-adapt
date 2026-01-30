#include "mesh_adapt/boundary/Delaunay2D.hpp"
#include <algorithm>
#include <cmath>

//  USAGE
//~ mesh_adapt::Delaunay2D dt(patch.points);
//~ dt.insert_constraints(patch.ring_polyline);    // Polyline2D
//~ dt.insert_constraints(patch.proj_polyline);    // Polyline2D
//~ dt.triangulate();
//~ patch.triangles = dt.get_triangles();


namespace mesh_adapt {

// Check if point p is inside circumcircle of triangle tri
bool Delaunay2D::circumcircle_contains(const std::array<int,3>& tri, const Vec2& p) const {
    const Vec2& a = points[tri[0]];
    const Vec2& b = points[tri[1]];
    const Vec2& c = points[tri[2]];

    double ax = a.x - p.x;
    double ay = a.y - p.y;
    double bx = b.x - p.x;
    double by = b.y - p.y;
    double cx = c.x - p.x;
    double cy = c.y - p.y;

    double det = (ax*ax + ay*ay) * (bx*cy - cx*by)
               - (bx*bx + by*by) * (ax*cy - cx*ay)
               + (cx*cx + cy*cy) * (ax*by - bx*ay);

    return det > 0;
}

// Compare edges
bool Delaunay2D::edges_equal(const Edge2D& e1, const Edge2D& e2) const {
    return (e1.a == e2.a && e1.b == e2.b) || (e1.a == e2.b && e1.b == e2.a);
}

// Insert constraints from polyline
void Delaunay2D::insert_constraints(const Polyline2D& polyline) {
    const std::vector<Vec2>& pts = polyline.get_points();
    for(size_t i = 0; i+1 < pts.size(); ++i){
        int a = int(i);
        int b = int(i+1);
        constraints.emplace_back(a,b);
    }
}

// Simple Bowyer-Watson triangulation
void Delaunay2D::triangulate() {
    triangles.clear();
    if(points.size() < 3) return;

    // Super-triangle
    double minX=points[0].x, minY=points[0].y;
    double maxX=points[0].x, maxY=points[0].y;
    for(const auto& p : points){
        if(p.x < minX) minX=p.x; if(p.y<minY) minY=p.y;
        if(p.x>maxX) maxX=p.x; if(p.y>maxY) maxY=p.y;
    }
    double dx=maxX-minX, dy=maxY-minY;
    double deltaMax=std::max(dx,dy);
    Vec2 p1(minX-10*deltaMax, minY-10*deltaMax);
    Vec2 p2(minX+10*deltaMax, minY-10*deltaMax);
    Vec2 p3(minX, minY+10*deltaMax);

    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);
    triangles.push_back({int(points.size()-3), int(points.size()-2), int(points.size()-1)});

    // Incremental Bowyer-Watson
    for(size_t i=0; i<points.size()-3; ++i){
        std::vector<std::array<int,3>> bad_triangles;
        for(auto& t : triangles)
            if(circumcircle_contains(t, points[i]))
                bad_triangles.push_back(t);

        std::vector<Edge2D> polygon;
        for(auto& t : bad_triangles){
            Edge2D e1(t[0],t[1]), e2(t[1],t[2]), e3(t[2],t[0]);
            Edge2D edges[3] = {e1,e2,e3};

            for(auto& e : edges){
                bool shared=false;
                for(auto& ot : bad_triangles){
                    if(&ot==&t) continue;
                    Edge2D oe1(ot[0],ot[1]), oe2(ot[1],ot[2]), oe3(ot[2],ot[0]);
                    if(edges_equal(e,oe1)||edges_equal(e,oe2)||edges_equal(e,oe3)){
                        shared=true; break;
                    }
                }
                if(!shared) polygon.push_back(e);
            }
        }

        // remove bad triangles
        triangles.erase(std::remove_if(triangles.begin(), triangles.end(),
            [&bad_triangles](const std::array<int,3>& t){
                return std::find(bad_triangles.begin(), bad_triangles.end(), t)!=bad_triangles.end();
            }), triangles.end());

        for(auto& e : polygon)
            triangles.push_back({e.a, e.b, int(i)});
    }

    // Remove triangles with super-triangle vertices
    triangles.erase(std::remove_if(triangles.begin(), triangles.end(),
        [n=int(points.size())](const std::array<int,3>& t){
            return t[0]>=n-3 || t[1]>=n-3 || t[2]>=n-3;
        }), triangles.end());

    points.resize(points.size()-3);

    // TODO: enforce constraints if needed
}

}
