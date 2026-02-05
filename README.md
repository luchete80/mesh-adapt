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


Subdivision

Inside MeshToSub from patch.subdivided_edge_to_node is created the map subdivided_edge_to_global

Inside QuadRefiner
  edge_map_ is buit from meshtosub.subdivided_edge_to_global,
  initial refined edges (probably from fallback_quads) are mark with INITIAL cause flag 

  Then, in refine_to_conform()
   quad_patter is built from qad with classify() (which calls QuadClassifier)

      void classify_quads() {
          QuadClassifier qc(quads_, edge_map_);
          qc.classify(quad_patterns_, quad_rotations_);
      }


Initial edges are from fallback tris, which have 1 side external. 
is convenient to mark as refined the neighbour edge which has one internal one in 
order confine subdivision


