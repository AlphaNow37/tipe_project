Projet de Simon Palade pour son TIPE

# Objectif

On implémente ici des algorithme pour résoudre le problème du plus court chemin euclidien

Certains algorithmes (graphes de visibilité, polyanya) sont exacts, d'autres (RRT, RRT*, PRM, Theta*, grilles) sont des approximations.

# Codes intéressants
## Algorithmes
| Nom                                                                                                                                                                                                                                                                                      | Résumé                                                                       | Lien |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------|------|
| [Algorithme naïf](https://github.com/AlphaNow37/tipe_project/blob/b96f0af05bd3bd582f303d3ef394e76d9a5b1540/src/path_planning/visibility_graph.rs#L132)                                                                                                                                   | On trouve le graphe de visibilité en O(n^3)                                  |      |
| [Algorithme de Lee](https://github.com/AlphaNow37/tipe_project/blob/b96f0af05bd3bd582f303d3ef394e76d9a5b1540/src/path_planning/visibility_graph.rs#L198)                                                                                                                                 | On utilise un balayage pour trouver le graphe de visibilité en O(n^2 log )   |      |
| Algo naïf (GPU) [shader](https://github.com/AlphaNow37/tipe_project/blob/b96f0af05bd3bd582f303d3ef394e76d9a5b1540/src/parallel/shaders/naive_2d.wgsl#L1) [rust](https://github.com/AlphaNow37/tipe_project/blob/b96f0af05bd3bd582f303d3ef394e76d9a5b1540/src/parallel/vis_graphs.rs#L80) | On implémente l'algo naïf sur le GPU via wgpu                                |      |
| [Triangulation](https://github.com/AlphaNow37/tipe_project/blob/b96f0af05bd3bd582f303d3ef394e76d9a5b1540/src/triangulations/triangulation_line_sweep.rs#L400)                                                                                                                            | On trouve une triangulation en O(n log n)                                    |      |
| [Delaunay](https://github.com/AlphaNow37/tipe_project/blob/b96f0af05bd3bd582f303d3ef394e76d9a5b1540/src/triangulations/delaunay.rs#L53)                                                                                                                                                  | On transforme une triangulation en triangulation de Delaunay                 |      |
| [Polyanya](https://github.com/AlphaNow37/tipe_project/blob/b96f0af05bd3bd582f303d3ef394e76d9a5b1540/src/path_planning/polyanya.rs#L651)                                                                                                                                                  | On trouve le chemin le plus court par une sorte de Dijkstra continu          |      |
| [RRT](https://github.com/AlphaNow37/tipe_project/blob/b96f0af05bd3bd582f303d3ef394e76d9a5b1540/src/path_planning/graphs_heuristics.rs#L76)                                                                                                                                               | On trouve un chemin entre deux points de façon rapide mais pas trop mauvaise |      |
| [RRT*](https://github.com/AlphaNow37/tipe_project/blob/b96f0af05bd3bd582f303d3ef394e76d9a5b1540/src/path_planning/graphs_heuristics.rs#L143)                                                                                                                                             | On raffine l'arbre de RRT pour le rendre asymptotiquement optimal            |      |
| [PRM](https://github.com/AlphaNow37/tipe_project/blob/b96f0af05bd3bd582f303d3ef394e76d9a5b1540/src/path_planning/graphs_heuristics.rs#L304)                                                                                                                                              | On crée un graphe et non un arbre                                            |      |
| [Reeds Shepp](https://github.com/AlphaNow37/tipe_project/blob/b96f0af05bd3bd582f303d3ef394e76d9a5b1540/src/workspace/reeds_shepp/mod.rs#L155)                                                                                                                                            | On utilise la topologie pour un objet au rayon de braquage limité            |      |
| [Theta*](https://github.com/AlphaNow37/tipe_project/blob/b96f0af05bd3bd582f303d3ef394e76d9a5b1540/src/graphs/traits.rs#L109)                                                                                                                                                             | On affine le résultat de A* pour prendre des raccourcis                      |      |
| [Grille d'accessibilité](https://github.com/AlphaNow37/tipe_project/blob/b96f0af05bd3bd582f303d3ef394e76d9a5b1540/src/path_planning/accessibility_grid.rs#L9)                                                                                                                            | On se base sur une grille de 0 et 1 pour trouver un chemin                   |      |

## Structures de données
| Nom                                                                                                                                 | Résumé                                                                  |
|-------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------|
| [RTree](https://github.com/AlphaNow37/tipe_project/blob/b96f0af05bd3bd582f303d3ef394e76d9a5b1540/src/datastructures/r_tree.rs#L45)  | Un arbre de rectangles imbriqués pour avoir des collisions rapides         |
| [Arbre BSP](https://github.com/AlphaNow37/tipe_project/blob/b96f0af05bd3bd582f303d3ef394e76d9a5b1540/src/datastructures/bsp.rs#L92) | Un arbre servant à optimiser les requêtes R-voisins                     |
| [Skip Lists](https://github.com/AlphaNow37/tipe_project/blob/main/src/datastructures/skip_list.rs)                                                                                                                      | Une structure de donnée servant ici de liste chainée avec random access |

## Benchmarks

| Nom                                                                                              | Résumé                                           |
|--------------------------------------------------------------------------------------------------|--------------------------------------------------|
| [Benchmark](https://github.com/AlphaNow37/tipe_project/blob/main/src/tests/test_perf_path_2d.rs) | On mésure la vitesse des algorithmes en pratique |
| [Génération de cartes](https://github.com/AlphaNow37/tipe_project/blob/b96f0af05bd3bd582f303d3ef394e76d9a5b1540/src/geometry/polygon_map_generator.rs#L91)                                                                         | On génère les cartes pour le benchmark           |

# Lancer le code:

Def façon générale, les étapes sont:

- Installer rust, via les instructions du site web
- Aller dans tests/mod.rs, décommenter le test voulu
- Lancer `cargo run`, ou `cargo run --release` pour optimiser le binaire
- Pour faire des visualisations 3d, ajouter `-feature gpu_vis` et installer mon repo `space_animation` qui sert de moteur 3D
- Pour utiliser la librairie externe polyanya, ajouter `-feature polyanya`
- Pour les compute shaders, `-feature gpu`
