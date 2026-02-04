// Leer una malla completa
Mesh2D mesh = read_mesh_from_vtk("mi_malla.vtk");

// Leer solo puntos (nube de puntos)
std::vector<Vec2> points = read_points_from_vtk("puntos.vtk");

// Exportar y luego re-importar
export_mesh_to_vtk(mesh, "output.vtk");
Mesh2D mesh2 = read_mesh_from_vtk("output.vtk");
