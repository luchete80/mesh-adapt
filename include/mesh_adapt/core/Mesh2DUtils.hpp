Mesh2D generate_structured_grid(double x_min, double x_max,
                                double y_min, double y_max,
                                double SL) {
    Mesh2D mesh;
    int nx = static_cast<int>((x_max - x_min) / SL) + 1;
    int ny = static_cast<int>((y_max - y_min) / SL) + 1;

    for(int j = 0; j < ny; ++j) {
        for(int i = 0; i < nx; ++i) {
            double x = x_min + i * SL;
            double y = y_min + j * SL;
            mesh.add_node(x, y);
        }
    }

    // Crear quads
    for(int j = 0; j < ny-1; ++j) {
        for(int i = 0; i < nx-1; ++i) {
            int n0 = j*nx + i;
            int n1 = n0 + 1;
            int n2 = n0 + nx + 1;
            int n3 = n0 + nx;
            mesh.add_quad(n0, n1, n2, n3);
        }
    }

    return mesh;
}

std::vector<int> filter_near_boundary_nodes(
        const Mesh2D& mesh,
        const Polyline2D& contour,
        double rmax) 
{
    std::vector<int> near_nodes;
    const auto& nodes = mesh.get_nodes();
    for(size_t i = 0; i < nodes.size(); ++i) {
        if(contour.distance(nodes[i].x) <= rmax) {
            near_nodes.push_back(i);
        }
    }
    return near_nodes;
}