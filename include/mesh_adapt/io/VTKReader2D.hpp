// En mesh_adapt/io/VTKReader2D.hpp
#pragma once
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <stdexcept>
#include "mesh_adapt/core/Mesh2D.hpp"
#include "mesh_adapt/core/Node2D.hpp"

namespace mesh_adapt {

inline Mesh2D read_mesh_from_vtk(const std::string& filename) {
    std::ifstream in(filename);
    if (!in.is_open()) {
        throw std::runtime_error("Cannot open VTK file: " + filename);
    }

    Mesh2D mesh;
    std::string line;
    
    // Variables para almacenar datos temporales
    std::vector<Vec2> points;
    std::vector<std::array<int, 4>> quads;
    
    bool reading_points = false;
    bool reading_cells = false;
    bool reading_cell_types = false;
    bool reading_point_data = false;
    
    int num_points = 0;
    int num_cells = 0;
    int points_read = 0;
    int cells_read = 0;
    
    while (std::getline(in, line)) {
        // Saltar líneas vacías y comentarios
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        
        if (token == "POINTS") {
            reading_points = true;
            reading_cells = false;
            reading_cell_types = false;
            reading_point_data = false;
            
            iss >> num_points;
            std::string type_str;
            iss >> type_str; // "float" o "double"
            
            points.clear();
            points.reserve(num_points);
            points_read = 0;
            
            continue;
        }
        else if (token == "CELLS") {
            reading_points = false;
            reading_cells = true;
            reading_cell_types = false;
            reading_point_data = false;
            
            int total_entries;
            iss >> num_cells >> total_entries;
            
            quads.clear();
            quads.reserve(num_cells);
            cells_read = 0;
            
            continue;
        }
        else if (token == "CELL_TYPES") {
            reading_points = false;
            reading_cells = false;
            reading_cell_types = true;
            reading_point_data = false;
            
            int temp;
            iss >> temp; // Debería ser igual a num_cells
            
            continue;
        }
        
        // Leer puntos
        if (reading_points && points_read < num_points) {
            double x, y, z;
            std::istringstream point_iss(line);
            point_iss >> x >> y >> z;
            
            points.push_back(Vec2(x, y));
            points_read++;
            
            if (points_read == num_points) {
                reading_points = false;
            }
        }
        // Leer celdas
        else if (reading_cells && cells_read < num_cells) {
            int num_vertices;
            std::istringstream cell_iss(line);
            cell_iss >> num_vertices;
            
            if (num_vertices == 4) { // Quad
                std::array<int, 4> quad;
                cell_iss >> quad[0] >> quad[1] >> quad[2] >> quad[3];
                quads.push_back(quad);
                cells_read++;
            }
            else if (num_vertices == 3) { // Triangle
                // Convertir triángulo a quad degenerado
                std::array<int, 4> quad;
                int v0, v1, v2;
                cell_iss >> v0 >> v1 >> v2;
                quad = {v0, v1, v2, v2}; // Último vértice repetido
                quads.push_back(quad);
                cells_read++;
            }
            // Ignorar otros tipos de celdas
        }
    }
    
    in.close();
    
    // Construir la malla - USANDO LA INTERFAZ CORRECTA
    for (const auto& point : points) {
        // Verifica si Mesh2D::add_node espera (double, double) o (Vec2)
        // Dependiendo de tu implementación:
        mesh.add_node(point.x, point.y);  // Si espera (double, double)
        // O: mesh.add_node(point);        // Si espera (Vec2)
    }
    
    // Añadir quads
    for (const auto& quad : quads) {
        mesh.add_quad(quad[0], quad[1], quad[2], quad[3]);
    }
    
    std::cout << "[VTK] Mesh loaded from " << filename << "\n";
    std::cout << "  - Nodes: " << mesh.num_nodes() << "\n";
    std::cout << "  - Quads: " << mesh.num_quads() << "\n";
    
    return mesh;
}

}
