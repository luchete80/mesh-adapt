#pragma once
#include <vector>
#include <cmath>
#include "Contour2D.hpp"
#include <algorithm>


namespace mesh_adapt {
  
  struct ProjectionResult {
    Vec2 q;        // punto proyectado
    int seg_id;    // segmento [pts[seg_id], pts[seg_id+1]]
    double t;      // parámetro local en ese segmento
};

struct ProjectionXY {
    Vec2 q;      // punto proyectado
    int seg_id;  // índice del segmento
};

// Proyección solo en X o en Y
enum class ProjAxis { X, Y };

class Polyline2D : public Contour2D {
public:
    std::vector<Vec2> pts;

    Polyline2D() = default;

    Polyline2D(const std::vector<Vec2>& points)
        : pts(points) {}

    std::vector<double> prefix;  // longitud acumulada
    
    double distance(const Vec2& p) const override;
    Vec2 project(const Vec2& p) const override;
    std::vector<Vec2> sample(int n) const override;
    const std::vector<Vec2>& get_points() const { return pts; }  // NUEVO


    

ProjectionResult project_with_segment(const Vec2& p) const
{
    ProjectionResult res;
    res.q = pts[0];
    res.seg_id = 0;
    res.t = 0.0;

    double min_dist = std::numeric_limits<double>::max();

    for(size_t i = 0; i + 1 < pts.size(); ++i)
    {
        Vec2 a = pts[i];
        Vec2 b = pts[i+1];
        Vec2 ab = b - a;

        double denom = dot(ab, ab);
        if(denom < 1e-14)
            continue;

        double t = dot(p - a, ab) / denom;
        t = std::clamp(t, 0.0, 1.0);

        Vec2 proj = a + t * ab;
        double d = (p - proj).norm();

        if(d < min_dist)
        {
            min_dist = d;
            res.q = proj;
            res.seg_id = (int)i;
            res.t = t;
        }
    }

    return res;
}

ProjectionResult project_best_xy(const Vec2& p) const {
    ProjectionResult best;
    best.q = pts[0];
    best.seg_id = 0;
    best.t = 0.0;

    double min_score = std::numeric_limits<double>::max();

    for(size_t i = 0; i + 1 < pts.size(); ++i) {
        Vec2 a = pts[i];
        Vec2 b = pts[i+1];
        Vec2 ab = b - a;
        Vec2 proj;

        // Detectar orientación de la arista
        bool is_horizontal = std::abs(ab.y) < 1e-12;
        bool is_vertical   = std::abs(ab.x) < 1e-12;

        if(is_horizontal) {
            // proyectar solo en Y
            proj.y = p.y;
            proj.y = std::clamp(proj.y, std::min(a.y,b.y), std::max(a.y,b.y));
            proj.x = a.x; // mantener X constante
        } else if(is_vertical) {
            // proyectar solo en X
            proj.x = p.x;
            proj.x = std::clamp(proj.x, std::min(a.x,b.x), std::max(a.x,b.x));
            proj.y = a.y; // mantener Y constante
        } else {
            // proyectar ortogonal al segmento
            double denom = dot(ab, ab);
            double t = dot(p - a, ab) / denom;
            t = std::clamp(t, 0.0, 1.0);
            proj = a + t * ab;
        }

        double dist = (p - proj).norm();

        // Opcional: penalizar según ángulo si quieres preferir segmentos más "alineados"
        double score = dist; // + angle_penalty si quieres

        if(score < min_score) {
            min_score = score;
            best.q = proj;
            best.seg_id = static_cast<int>(i);

            // calcular t relativo sobre el segmento original
            Vec2 ab_seg = b - a;
            double denom = dot(ab_seg, ab_seg);
            if(denom > 1e-14)
                best.t = dot(proj - a, ab_seg) / denom;
            else
                best.t = 0.0;
        }
    }

    return best;
}


void build_arc_length()
{
    prefix.resize(pts.size());
    prefix[0] = 0.0;
    for(size_t i = 1; i < pts.size(); ++i)
        prefix[i] = prefix[i-1] + (pts[i] - pts[i-1]).norm();
}

double total_length() const
{
    return prefix.back();
}

double arc_length_at_segment(size_t seg_id, const Vec2& q) const
{
    return prefix[seg_id] + (q - pts[seg_id]).norm();
}


    
};

}
