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

//~ std::vector<ProjectionResult> project_corner_aware(const Vec2& p) const
//~ {
    //~ std::vector<ProjectionResult> results;

    //~ // Recorremos todos los segmentos
    //~ for(size_t i = 0; i + 1 < pts.size(); ++i)
    //~ {
        //~ Vec2 a = pts[i];
        //~ Vec2 b = pts[i+1];
        //~ Vec2 ab = b - a;

        //~ double denom = dot(ab, ab);
        //~ if(denom < 1e-14) continue;

        //~ double t = dot(p - a, ab) / denom;
        //~ t = std::clamp(t, 0.0, 1.0);

        //~ Vec2 proj = a + t * ab;
        //~ double d = (p - proj).norm();

        //~ ProjectionResult pr;
        //~ pr.q = proj;
        //~ pr.seg_id = static_cast<int>(i);
        //~ pr.t = t;
        //~ results.push_back(pr);
    //~ }

    //~ // Chequeamos proximidad a vértices: si está “cerca de una esquina”, duplicamos la proyección
    //~ const double corner_thresh = 1e-6; // tolerancia pequeña
    //~ for(size_t i = 0; i < pts.size(); ++i)
    //~ {
        //~ double d = (p - pts[i]).norm();
        //~ if(d < corner_thresh)
        //~ {
            //~ ProjectionResult pr;
            //~ pr.q = pts[i];
            //~ pr.seg_id = static_cast<int>(i);
            //~ pr.t = 0.0;
            //~ results.push_back(pr);
        //~ }
    //~ }

    //~ // Opcional: filtrar para devolver solo la proyección más cercana por segmento/vértice
    //~ // Se puede ordenar por distancia si querés
    //~ return results;
//~ }



/////////////// ORIGINAL 

//~ std::vector<ProjectionResult> project_corner_aware(const Vec2& p) const
//~ {
    //~ std::vector<ProjectionResult> results;
    //~ const double corner_thresh = 1e-6;
    //~ const double dup_thresh = 1e-12;  // Para detectar duplicados

    //~ // 1. Proyecciones sobre segmentos (incluyendo extremos)
    //~ for(size_t i = 0; i + 1 < pts.size(); ++i)
    //~ {
        //~ Vec2 a = pts[i];
        //~ Vec2 b = pts[i+1];
        //~ Vec2 ab = b - a;

        //~ double denom = dot(ab, ab);
        //~ if(denom < 1e-14) continue;

        //~ double t = dot(p - a, ab) / denom;
        
        //~ // Solo considerar proyecciones dentro del segmento o muy cerca
        //~ if (t < -corner_thresh || t > 1.0 + corner_thresh) {
            //~ continue;
        //~ }
        
        //~ t = std::clamp(t, 0.0, 1.0);
        //~ Vec2 proj = a + t * ab;

        //~ ProjectionResult pr;
        //~ pr.q = proj;
        //~ pr.seg_id = static_cast<int>(i);
        //~ pr.t = t;
        //~ results.push_back(pr);
    //~ }

    //~ // 2. Proyecciones especiales para esquinas (solo si NO hay ya una proyección cercana)
    //~ for(size_t i = 0; i < pts.size(); ++i)
    //~ {
        //~ double d = (p - pts[i]).norm();
        //~ if(d < corner_thresh)
        //~ {
            //~ // Verificar si ya tenemos una proyección suficientemente cercana a este vértice
            //~ bool already_exists = false;
            //~ for(const auto& existing : results) {
                //~ if((existing.q - pts[i]).norm() < dup_thresh) {
                    //~ already_exists = true;
                    //~ break;
                //~ }
            //~ }
            
            //~ if(already_exists) continue;  // Saltar si ya existe

            //~ // Para vértices interiores, añadir ambas posibilidades
            //~ if (i > 0 && i < pts.size() - 1) {
                //~ // Segmento anterior
                //~ ProjectionResult pr_prev;
                //~ pr_prev.q = pts[i];
                //~ pr_prev.seg_id = static_cast<int>(i-1);
                //~ pr_prev.t = 1.0;
                //~ results.push_back(pr_prev);
                
                //~ // Segmento siguiente  
                //~ ProjectionResult pr_next;
                //~ pr_next.q = pts[i];
                //~ pr_next.seg_id = static_cast<int>(i);
                //~ pr_next.t = 0.0;
                //~ results.push_back(pr_next);
            //~ }
            //~ else if (i == 0 && pts.size() > 1) {
                //~ ProjectionResult pr;
                //~ pr.q = pts[i];
                //~ pr.seg_id = 0;
                //~ pr.t = 0.0;
                //~ results.push_back(pr);
            //~ }
            //~ else if (i == pts.size() - 1 && pts.size() > 1) {
                //~ ProjectionResult pr;
                //~ pr.q = pts[i];
                //~ pr.seg_id = static_cast<int>(pts.size() - 2);
                //~ pr.t = 1.0;
                //~ results.push_back(pr);
            //~ }
        //~ }
    //~ }

    //~ return results;
//~ }

std::vector<ProjectionResult> project_corner_aware(const Vec2& p) const
{
    std::vector<ProjectionResult> results;
    const double corner_thresh = 1e-6; // distancia para considerar "sobre el vértice"
    const double dup_thresh = 1e-12;   // para evitar duplicados exactos

    // 1️⃣ Proyecciones sobre todos los segmentos
    for(size_t i = 0; i + 1 < pts.size(); ++i)
    {
        Vec2 a = pts[i];
        Vec2 b = pts[i+1];
        Vec2 ab = b - a;
        double denom = dot(ab, ab);
        if(denom < 1e-14) continue;

        double t = dot(p - a, ab) / denom;

        // Solo considerar proyecciones dentro del segmento extendido por corner_thresh
        if(t < -corner_thresh || t > 1.0 + corner_thresh) continue;

        t = std::clamp(t, 0.0, 1.0);
        Vec2 proj = a + t * ab;

        ProjectionResult pr;
        pr.q = proj;
        pr.seg_id = static_cast<int>(i);
        pr.t = t;
        results.push_back(pr);
    }

    // 2️⃣ Proyecciones sobre vértices (esquinas)
    for(size_t i = 0; i < pts.size(); ++i)
    {
        double d = (p - pts[i]).norm();
        if(d > corner_thresh) continue; // no estamos sobre el vértice

        // Evitar duplicados
        bool already_exists = false;
        for(const auto& existing : results) {
            if((existing.q - pts[i]).norm() < dup_thresh) {
                already_exists = true;
                break;
            }
        }
        if(already_exists) continue;

        // Añadir proyecciones para los segmentos adyacentes
        if(i > 0) {
            ProjectionResult pr_prev;
            pr_prev.q = pts[i];
            pr_prev.seg_id = static_cast<int>(i-1);
            pr_prev.t = 1.0; // extremo del segmento anterior
            results.push_back(pr_prev);
        }
        if(i < pts.size() - 1) {
            ProjectionResult pr_next;
            pr_next.q = pts[i];
            pr_next.seg_id = static_cast<int>(i);
            pr_next.t = 0.0; // inicio del segmento siguiente
            results.push_back(pr_next);
        }
    }

    return results;
}

std::vector<ProjectionResult> project_corner_aware_xy(const Vec2& p) const
{
    std::vector<ProjectionResult> results;
    const double corner_thresh = 1e-6; // tolerancia para vértices
    const double dup_thresh = 1e-12;   // para evitar duplicados exactos

    auto push_if_new = [&](const ProjectionResult& pr){
        for(const auto& existing : results)
            if((existing.q - pr.q).norm() < dup_thresh) return;
        results.push_back(pr);
    };

    // 1️⃣ Proyección sobre segmentos según tangente
    for(size_t i = 0; i + 1 < pts.size(); ++i)
    {
        Vec2 a = pts[i];
        Vec2 b = pts[i+1];
        Vec2 ab = b - a;
        if(dot(ab, ab) < 1e-14) continue;

        bool dominant_x = std::abs(ab.x) >= std::abs(ab.y);

        ProjectionResult pr;
        pr.seg_id = static_cast<int>(i);

        if(dominant_x) {
            pr.q.x = p.x;
            pr.q.y = a.y + (p.x - a.x) * ab.y / ab.x;  // interpola en Y
            pr.t = (p.x - a.x) / ab.x;
        } else {
            pr.q.y = p.y;
            pr.q.x = a.x + (p.y - a.y) * ab.x / ab.y;  // interpola en X
            pr.t = (p.y - a.y) / ab.y;
        }

        pr.t = std::clamp(pr.t, 0.0, 1.0);
        push_if_new(pr);
    }

    // 2️⃣ Proyecciones sobre vértices (duplicar si es esquina)
    for(size_t i = 0; i < pts.size(); ++i)
    {
        double d = (p - pts[i]).norm();
        if(d > corner_thresh) continue;

        // Segmento anterior
        if(i > 0) {
            ProjectionResult pr;
            pr.q = pts[i];
            pr.seg_id = static_cast<int>(i-1);
            pr.t = 1.0;
            push_if_new(pr);
        }
        // Segmento siguiente
        if(i < pts.size() - 1) {
            ProjectionResult pr;
            pr.q = pts[i];
            pr.seg_id = static_cast<int>(i);
            pr.t = 0.0;
            push_if_new(pr);
        }
    }

    return results;
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
