mesh-adapt/
  include/
    mesh_adapt/
      core/
        Mesh2D.hpp              # Topología básica (nodos + quads)
        MeshTopology.hpp        # Análisis topológico (edges, boundaries)
        
      geometry/
        Vec2.hpp                # Primitivas geométricas
        Polyline2D.hpp          # Operaciones geométricas puras
        
      boundary/                 # NUEVO módulo
        BoundaryLayer.hpp       # Manejo de capas de borde
        ProjectionStrategy.hpp  # Estrategias de proyección
        
      remesh/
        MartinsRemesher.hpp     # Orquestador principal
        TransitionZone.hpp      # Zona de transición (Delaunay)
        QuadSubdivision.hpp     # Subdivisión de quads
        
      smoothing/
        LaplacianSmoother.hpp
        
  src/
    (misma estructura)
    
    
    
1. Background Structure Grid
2. Corte por contorno
Create new Mesh2D inside_mesh = filter_nodes_inside_contour(mesh_bg, cone_contour);
Mesh2D band_mesh = remove_nodes_near_contour(inside_mesh, cone_contour, SL);

Mesh2Sub unify, temporarily, BandMesh & TransitionPatch MERGED quad (not all of them, fallback quads are not included, are used to mark edges to subdivide), 
