#include "mesh_adapt/boundary/Delaunay2D.hpp"
#include <algorithm>
#include <cmath>

namespace mesh_adapt {


//==============================================================
// Build from ONLY two boundary loops
//==============================================================
void Delaunay2D::build_from_two_polylines(
    const Polyline2D& outer,
    const Polyline2D& inner
){
    points.clear();
    constraints.clear();
    triangles.clear();

    const auto& outer_pts = outer.get_points();
    const auto& inner_pts = inner.get_points();

    std::vector<int> outer_ids;
    std::vector<int> inner_ids;

    //----------------------------------------------------------
    // Insert INNER loop points
    //----------------------------------------------------------
    for(const auto& p : inner_pts){
        inner_ids.push_back((int)points.size());
        points.push_back(p);
    }

    //----------------------------------------------------------
    // Insert OUTER loop points
    //----------------------------------------------------------
    for(const auto& p : outer_pts){
        outer_ids.push_back((int)points.size());
        points.push_back(p);
    }


    //----------------------------------------------------------
    // Insert constraint edges
    //----------------------------------------------------------
    add_constraints_from_loop(outer_ids);
    add_constraints_from_loop(inner_ids);
}


//==============================================================
// Add constraint edges for a closed loop
//==============================================================
void Delaunay2D::add_constraints_from_loop(
    const std::vector<int>& loop_ids
){
    int n = (int)loop_ids.size();
    if(n < 3) return;

    for(int i=0; i<n; ++i){
        int a = loop_ids[i];
        int b = loop_ids[(i+1)%n]; // closed loop wrap

        constraints.emplace_back(a,b);
    }
}


//==============================================================
// Circumcircle test (Bowyer-Watson)
//==============================================================
bool Delaunay2D::circumcircle_contains(
    const std::array<int,3>& tri,
    const Vec2& p
) const {
    const Vec2& a = points[tri[0]];
    const Vec2& b = points[tri[1]];
    const Vec2& c = points[tri[2]];

    double ax = a.x - p.x;
    double ay = a.y - p.y;
    double bx = b.x - p.x;
    double by = b.y - p.y;
    double cx = c.x - p.x;
    double cy = c.y - p.y;

    double det =
        (ax*ax + ay*ay) * (bx*cy - cx*by)
      - (bx*bx + by*by) * (ax*cy - cx*ay)
      + (cx*cx + cy*cy) * (ax*by - bx*ay);

    return det > 0;
}


//==============================================================
// Edge equality ignoring orientation
//==============================================================
bool Delaunay2D::edges_equal(
    const Edge2D& e1,
    const Edge2D& e2
) const {
    return (e1.a == e2.a && e1.b == e2.b) ||
           (e1.a == e2.b && e1.b == e2.a);
}


//==============================================================
// Bowyer-Watson triangulation using ONLY boundary points
//==============================================================
void Delaunay2D::triangulate()
{
    triangles.clear();
    if(points.size() < 3) return;

    //----------------------------------------------------------
    // Step 1: Super triangle
    //----------------------------------------------------------
    double minX=points[0].x, minY=points[0].y;
    double maxX=points[0].x, maxY=points[0].y;

    for(const auto& p : points){
        minX = std::min(minX,p.x);
        minY = std::min(minY,p.y);
        maxX = std::max(maxX,p.x);
        maxY = std::max(maxY,p.y);
    }

    double dx = maxX-minX;
    double dy = maxY-minY;
    double delta = std::max(dx,dy);

    Vec2 p1(minX-10*delta, minY-10*delta);
    Vec2 p2(maxX+10*delta, minY-10*delta);
    Vec2 p3((minX+maxX)/2.0, maxY+10*delta);

    int i1 = (int)points.size();
    int i2 = i1+1;
    int i3 = i1+2;

    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);

    triangles.push_back({i1,i2,i3});

    //----------------------------------------------------------
    // Step 2: Insert points incrementally
    //----------------------------------------------------------
    int real_points = (int)points.size()-3;

    for(int i=0; i<real_points; ++i){

        std::vector<std::array<int,3>> bad;

        for(auto& t : triangles){
            if(circumcircle_contains(t, points[i]))
                bad.push_back(t);
        }

        //------------------------------------------------------
        // Find polygon cavity boundary
        //------------------------------------------------------
        std::vector<Edge2D> polygon;

        for(auto& t : bad){

            Edge2D edges[3] = {
                Edge2D(t[0],t[1]),
                Edge2D(t[1],t[2]),
                Edge2D(t[2],t[0])
            };

            for(auto& e : edges){

                bool shared=false;

                for(auto& ot : bad){
                    if(t==ot) continue;

                    Edge2D oe[3] = {
                        Edge2D(ot[0],ot[1]),
                        Edge2D(ot[1],ot[2]),
                        Edge2D(ot[2],ot[0])
                    };

                    if(edges_equal(e,oe[0]) ||
                       edges_equal(e,oe[1]) ||
                       edges_equal(e,oe[2])){
                        shared=true;
                        break;
                    }
                }

                if(!shared)
                    polygon.push_back(e);
            }
        }

        //------------------------------------------------------
        // Remove bad triangles
        //------------------------------------------------------
        triangles.erase(
            std::remove_if(triangles.begin(), triangles.end(),
            [&](const std::array<int,3>& t){
                return std::find(bad.begin(),bad.end(),t)!=bad.end();
            }),
            triangles.end()
        );

        //------------------------------------------------------
        // Retriangulate cavity
        //------------------------------------------------------
        for(auto& e : polygon){
            triangles.push_back({e.a,e.b,i});
        }
    }

    //----------------------------------------------------------
    // Step 3: Remove super triangle triangles
    //----------------------------------------------------------
    int N = (int)points.size();

    triangles.erase(
        std::remove_if(triangles.begin(), triangles.end(),
        [&](const std::array<int,3>& t){
            return (t[0]>=N-3 || t[1]>=N-3 || t[2]>=N-3);
        }),
        triangles.end()
    );

    points.resize(N-3);

    //----------------------------------------------------------
    // Step 4: TODO Sloan constraint enforcement
    //----------------------------------------------------------
}

} // namespace mesh_adapt
