#pragma once
#include <vector>
#include "mesh_adapt/geometry/EdgeInfo.hpp"
#include <array>

namespace mesh_adapt {


enum QuadPattern {
    PAT_NONE,
    PAT_ONE,
    PAT_TWO_ADJ_LEFT,
    PAT_TWO_ADJ_RIGHT,
    PAT_TWO_OPP,
    PAT_THREE,
    PAT_FULL
};

inline void classify_quad(
    const Quad& q,
    const std::map<Edge, EdgeInfo>& edge_map,
    QuadPattern& pat,
    int& rot
)
{
    std::array<bool,4> edge_sub = {false,false,false,false};

    // Detectar qué edges están subdivididos
    for(int i = 0; i < 4; ++i)
    {
        Edge e(q[i], q[(i+1)%4]);
        auto it = edge_map.find(e);
        edge_sub[i] = (it != edge_map.end()) ? it->second.subdivide : false;
    }

    std::vector<int> refined;
    for(int i = 0; i < 4; ++i)
        if(edge_sub[i]) refined.push_back(i);

    const int n = refined.size();

    switch(n)
    {
        case 0:
            pat = PAT_NONE;
            rot = 0;
            return;

        case 1:
            pat = PAT_ONE;
            rot = refined[0];
            return;

        case 2:
        {
            int e0 = refined[0];
            int e1 = refined[1];

            // --------- Opuestos ----------
            if( (e0 + 2) % 4 == e1 )
            {
                pat = PAT_TWO_OPP;
                rot = e0;   // rot determinística
                return;
            }

            // --------- Adyacentes ----------
            // Buscamos la rotación i tal que:
            // edge i y edge i+1 estén ambos refinados.
            for(int i = 0; i < 4; ++i)
            {
                if(edge_sub[i] && edge_sub[(i+1)%4])
                {
                    pat = PAT_TWO_ADJ_LEFT;
                    rot = i;
                    return;
                }
            }

            // Si llegamos acá algo está inconsistente
            pat = PAT_NONE;
            rot = 0;
            return;
        }

        case 3:
            pat = PAT_THREE;
            rot = 0;
            return;

        case 4:
            pat = PAT_FULL;
            rot = 0;
            return;
    }

    // fallback defensivo
    pat = PAT_NONE;
    rot = 0;
}


}
