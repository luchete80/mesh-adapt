#include "mesh_adapt/core/Mesh2D.hpp"
#include <map>
#include <set>

#include <unordered_set>

//#define DEBUG_BOUNDARY

namespace mesh_adapt {

int Mesh2D::add_node(double x, double y) {
    nodes.emplace_back(x,y); //more efficent thgan nodes.push_back(Node2D(x, y));
    return (int)nodes.size() - 1;
}

void Mesh2D::add_quad(int a, int b, int c, int d)
{
    quads.push_back({a, b, c, d});
}

size_t Mesh2D::num_nodes() const
{
    return (int)nodes.size();
}

size_t Mesh2D::num_quads() const
{
    return (int)quads.size();
}

std::vector<int> Mesh2D::find_boundary_nodes() const {

    using Edge = std::pair<int,int>;

    std::map<Edge,int> edge_count;

    auto add_edge = [&](int a, int b) {
        if(a > b) std::swap(a,b);
        edge_count[{a,b}]++;
    };

    // contar aristas
    for(const auto& q : quads) {

        add_edge(q[0], q[1]);
        add_edge(q[1], q[2]);
        add_edge(q[2], q[3]);
        add_edge(q[3], q[0]);
    }

    // nodos frontera = edges con count=1
    std::set<int> boundary_nodes;

    for(auto& [e,count] : edge_count) {
        if(count == 1) {
            boundary_nodes.insert(e.first);
            boundary_nodes.insert(e.second);
        }
    }

    return std::vector<int>(
        boundary_nodes.begin(),
        boundary_nodes.end()
    );
}


//~ std::vector<int> Mesh2D::find_ordered_boundary_nodes() const {
    //~ using Edge = std::pair<int,int>;
    //~ std::map<Edge,int> edge_count;

    //~ auto add_edge = [&](int a, int b){
        //~ if(a>b) std::swap(a,b);
        //~ edge_count[{a,b}]++;
    //~ };

    //~ // contar todas las aristas
    //~ for(const auto& q : quads) {
        //~ add_edge(q[0],q[1]);
        //~ add_edge(q[1],q[2]);
        //~ add_edge(q[2],q[3]);
        //~ add_edge(q[3],q[0]);
    //~ }

    //~ // construir vecinos de nodos frontera (aristas únicas)
    //~ std::map<int,std::vector<int>> neighbors;
    //~ for(const auto& [e,count] : edge_count) {
        //~ if(count==1) {
            //~ neighbors[e.first].push_back(e.second);
            //~ neighbors[e.second].push_back(e.first);
        //~ }
    //~ }

    //~ // --- validar que no haya nodos con más de 2 vecinos ---
    //~ for(const auto& [node,nbs] : neighbors) {
        //~ if(nbs.size() != 2) {
            //~ std::cerr << "[Warning] Boundary node " << node 
                      //~ << " tiene " << nbs.size() << " vecinos.\n";
        //~ }
    //~ }

    //~ std::vector<int> ordered;
    //~ if(neighbors.empty()) return ordered;

    //~ std::unordered_set<int> visited;
    //~ int start = neighbors.begin()->first;
    //~ int prev = -1;
    //~ int current = start;

    //~ while(visited.size() < neighbors.size()) {
        //~ ordered.push_back(current);
        //~ visited.insert(current);

        //~ auto& nbs = neighbors[current];
        //~ int next = -1;
        //~ for(int nb : nbs) {
            //~ if(nb != prev && visited.find(nb) == visited.end()) {
                //~ next = nb;
                //~ break;
            //~ }
        //~ }

        //~ if(next == -1) {
            //~ // puede ser ciclo cerrado
            //~ for(int nb : nbs) {
                //~ if(nb != prev) {
                    //~ next = nb;
                    //~ break;
                //~ }
            //~ }
        //~ }

        //~ prev = current;
        //~ current = next;

        //~ if(current == -1 || current == start) break; // cerramos el ciclo
    //~ }

    //~ return ordered;
//~ }


std::vector<int> Mesh2D::find_ordered_boundary_nodes() const {
    using Edge = std::pair<int,int>;
    std::map<Edge,int> edge_count;

    auto add_edge = [&](int a, int b){
        if(a>b) std::swap(a,b);
        edge_count[{a,b}]++;
        #ifdef DEBUG_BOUNDARY
        std::cout << "  Arista agregada: " << a << "->" << b 
                  << " (guardada como " << std::min(a,b) << "," << std::max(a,b) << ")" << std::endl;
        #endif
    };

    // contar todas las aristas
    #ifdef DEBUG_BOUNDARY
    std::cout << "=== CONTANDO ARISTAS DE " << quads.size() << " QUADS ===" << std::endl;
    #endif
    
    for(int qi = 0; qi < (int)quads.size(); ++qi) {
        const auto& q = quads[qi];
        #ifdef DEBUG_BOUNDARY
        std::cout << "Quad " << qi << ": [" << q[0] << "," << q[1] 
                  << "," << q[2] << "," << q[3] << "]" << std::endl;
        #endif
        add_edge(q[0],q[1]);
        add_edge(q[1],q[2]);
        add_edge(q[2],q[3]);
        add_edge(q[3],q[0]);
    }

    #ifdef DEBUG_BOUNDARY
    std::cout << "\n=== CONTEOS DE ARISTAS ===" << std::endl;
    for(const auto& [e,count] : edge_count) {
        std::cout << "  (" << e.first << "," << e.second << "): " 
                  << count << " veces" << std::endl;
    }
    #endif

    // construir vecinos de nodos frontera (aristas únicas)
    std::map<int,std::vector<int>> neighbors;
    
    #ifdef DEBUG_BOUNDARY
    std::cout << "\n=== ARISTAS DE FRONTERA (count==1) ===" << std::endl;
    #endif
    
    for(const auto& [e,count] : edge_count) {
        if(count==1) {
            neighbors[e.first].push_back(e.second);
            neighbors[e.second].push_back(e.first);
            #ifdef DEBUG_BOUNDARY
            std::cout << "  (" << e.first << "," << e.second << ") es frontera" << std::endl;
            #endif
        }
    }

    #ifdef DEBUG_BOUNDARY
    std::cout << "\n=== VECINOS POR NODO ===" << std::endl;
    for(const auto& [node,nbs] : neighbors) {
        std::cout << "  Nodo " << node << " tiene " << nbs.size() 
                  << " vecinos: [";
        for(size_t i = 0; i < nbs.size(); ++i) {
            std::cout << nbs[i];
            if(i < nbs.size() - 1) std::cout << ",";
        }
        std::cout << "]" << std::endl;
    }
    #endif

    // --- validar que no haya nodos con más de 2 vecinos ---
    bool has_problem = false;
    for(const auto& [node,nbs] : neighbors) {
        if(nbs.size() != 2) {
            has_problem = true;
            std::cerr << "[ERROR] Boundary node " << node 
                      << " tiene " << nbs.size() << " vecinos: [";
            for(size_t i = 0; i < nbs.size(); ++i) {
                std::cerr << nbs[i];
                if(i < nbs.size() - 1) std::cerr << ",";
            }
            std::cerr << "]" << std::endl;
        }
    }

    #ifdef DEBUG_BOUNDARY
    if(!has_problem) {
        std::cout << "\n✓ Todos los nodos tienen exactamente 2 vecinos" << std::endl;
    }
    #endif

    std::vector<int> ordered;
    if(neighbors.empty()) {
        #ifdef DEBUG_BOUNDARY
        std::cout << "\n✗ No hay nodos de frontera!" << std::endl;
        #endif
        return ordered;
    }

    // Determinar orientación
    bool is_ccw = true; // asumimos CCW inicialmente
    if(ordered.size() >= 3) {
        // Calcular área con signo para determinar orientación
        double area = 0.0;
        for(size_t i = 0; i < ordered.size(); ++i) {
            const Vec2& p0 = nodes[ordered[i]].x;
            const Vec2& p1 = nodes[ordered[(i+1)%ordered.size()]].x;
            area += p0.x * p1.y - p1.x * p0.y;
        }
        is_ccw = (area > 0.0);
        #ifdef DEBUG_BOUNDARY
        std::cout << "\nÁrea con signo: " << area 
                  << " => " << (is_ccw ? "CCW" : "CW") << std::endl;
        #endif
    }

    std::unordered_set<int> visited;
    int start = neighbors.begin()->first;
    int prev = -1;
    int current = start;

    #ifdef DEBUG_BOUNDARY
    std::cout << "\n=== CONSTRUYENDO ORDEN ===" << std::endl;
    std::cout << "Start: " << start << std::endl;
    #endif

    while(visited.size() < neighbors.size()) {
        ordered.push_back(current);
        visited.insert(current);

        #ifdef DEBUG_BOUNDARY
        std::cout << "  Agregado nodo " << current << " (visitados: " 
                  << visited.size() << "/" << neighbors.size() << ")" << std::endl;
        #endif

        auto& nbs = neighbors[current];
        int next = -1;
        for(int nb : nbs) {
            if(nb != prev && visited.find(nb) == visited.end()) {
                next = nb;
                break;
            }
        }

        if(next == -1) {
            // puede ser ciclo cerrado
            for(int nb : nbs) {
                if(nb != prev) {
                    next = nb;
                    #ifdef DEBUG_BOUNDARY
                    std::cout << "  Última arista: " << current << "->" << next 
                              << " (cerrando ciclo)" << std::endl;
                    #endif
                    break;
                }
            }
        }

        prev = current;
        current = next;

        if(current == -1) {
            #ifdef DEBUG_BOUNDARY
            std::cout << "  ✗ No hay siguiente nodo!" << std::endl;
            #endif
            break;
        }
        
        if(current == start) {
            #ifdef DEBUG_BOUNDARY
            std::cout << "  ✓ Ciclo cerrado en nodo " << start << std::endl;
            #endif
            break;
        }
    }

    #ifdef DEBUG_BOUNDARY
    std::cout << "\n=== BOUNDARY FINAL ===" << std::endl;
    std::cout << "Orden (" << ordered.size() << " nodos): [";
    for(size_t i = 0; i < ordered.size(); ++i) {
        std::cout << ordered[i];
        if(i < ordered.size() - 1) std::cout << "->";
    }
    std::cout << "]" << std::endl;
    
    // Mostrar coordenadas
    std::cout << "Coordenadas:" << std::endl;
    for(int node_id : ordered) {
        const Vec2& p = nodes[node_id].x;
        std::cout << "  " << node_id << ": (" << p.x << ", " << p.y << ")" << std::endl;
    }
    #endif

    return ordered;
}


} // namespace mesh_adapt
