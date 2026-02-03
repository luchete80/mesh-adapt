#pragma once
#include <vector>
#include "EdgeInfo.hpp"
#include <array>

enum QuadPattern {
    PAT_NONE,
    PAT_ONE,
    PAT_TWO_ADJ,
    PAT_TWO_OPP,
    PAT_THREE,
    PAT_FULL
};

class QuadClassifier
{
public:
    QuadClassifier(
        const std::vector<std::array<int,4>>& quads,
        const std::map<Edge, EdgeInfo>& edge_map
    ) : quads_(quads), edge_map_(edge_map)
    {}

    void classify(std::vector<QuadPattern>& patterns,
                  std::vector<int>& rotations) const
    {
        patterns.clear();
        rotations.clear();

        for(const auto& q : quads_) {
            std::array<bool,4> edge_sub = {false,false,false,false};

            for(int i=0;i<4;++i){
                Edge e(q[i], q[(i+1)%4]);
                auto it = edge_map_.find(e);
                edge_sub[i] = (it != edge_map_.end()) ? it->second.subdivide : false;
            }

            // calcular patrón y rotación
            std::vector<int> refined_edges;
            for(int i=0;i<4;++i)
                if(edge_sub[i]) refined_edges.push_back(i);

            int n = refined_edges.size();
            QuadPattern pat;
            int rot = 0;

            switch(n){
                case 0: pat = PAT_NONE; rot=0; break;
                case 1: pat = PAT_ONE; rot = refined_edges[0]; break;
                case 2:
                    if((refined_edges[1]-refined_edges[0]+4)%4==1)
                        { pat = PAT_TWO_ADJ; rot = refined_edges[0]; }
                    else
                        { pat = PAT_TWO_OPP; rot = refined_edges[0]; }
                    break;
                case 3: pat = PAT_THREE; rot = 0; break; // puede rotarse según convenga
                case 4: pat = PAT_FULL; rot = 0; break;
            }

            patterns.push_back(pat);
            rotations.push_back(rot);
        }
    }

private:
    const std::vector<std::array<int,4>>& quads_;
    const std::map<Edge, EdgeInfo>& edge_map_;
};
