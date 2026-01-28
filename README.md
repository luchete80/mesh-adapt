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
