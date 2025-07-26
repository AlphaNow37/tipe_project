
Projet de Simon Palade pour son TIPE

# Objectifs
Ce TIPE explore la thématique du path planning: trouver le chemin le plus court allant d'un point A à un point B

## En 2D
On se donne une liste de polygones, ainsi que deux points

On renvoie une liste de positions, le chemin le plus court

On supposeras que:
- Les obstacles sont strictement disjoints
- Il n'y a pas trois points alignés
- Il n'y a pas deux points alignés avec les axes X et Y

## En 3D (et N-D de façon générale)
Le problème devient alors NP-difficile
On utilise alors des heuristiques intéressantes

On se donne une liste de boites alignées avec les axes (AABB) avec potentiellement des intersections

## Questions diverses
Que se passe il:
- Si l'objet à une taille non nulle ? On utilise des sommes de Minkovski pour modifier la map
- Sur une sphère ? Je pense qu'on peut adapter l'algo 2D avec les graphes de visibilités
- En partant/arrivant sur une ligne ou autre forme ? A voir
- Si les obstacles forment un seul polygone ? O(n) avec un algo plus optimisé mais frenchement compliqué
- Avec la distance de Manathan ? A voir
- Si les obstacles bougent ? NP-diff je crois
- En N dimensions ? Comme en 3D, utile pour des bras de robots, ..
- Si le rayon de courbure est non nul (voiture, ..) ? A voir
- En prenant en compte l'acceleration ? A voir

# Organisation pour les concours
## Tetraconcours
On considère un circuit de course de drone

On cherche alors à terminer la course le plus rapidement possible

On ferait alors:
- Présentation du problème de la course de drone
- Introduction de l'algo naïf en O(n^3), critique
- Introduction de l'algo en O(n^2 log n), preuve des invariants
- Critique de la modélisation, qui est trop stricte (suppose de connaitre les obstacles, être en 2D)
- Introduction de RRT et RRT*, preuve de l'optimalité asymptotique si il y a le temps

## ENS
On fait un peu tout: (tout ne sera pas fait faute de temps à l'oral)
- Présentation du problème
- Introduction de l'algo naïf en O(n^3), critique
- Introduction de l'algo en O(n^2 log n), preuve des invariants
- Introduction de l'algo en O(n log n), preuve si c'est pas trop long/chiant, si j'ai le temps de l'implémenter
- Comparaison des performances
- Critique de la modélisation, qui est trop stricte (suppose de connaitre les obstacles, être en 2D)
- Preuve de la NP-difficulté du problème en 3D et supérieur
- Introduction de l'algo se basant sur une grille, critique
- Introduction de RRT et RRT*, preuve de l'optimalité asymptotique
- Introduction de PRM ou RRT FN
- Comparaison des algos, de leur vitesse de convergence, de leur consommation mémoire
- Application à un bras robotique en haute dimension

# Algorithmes

## 2D naif
Pour chaque paire de sommet, ils sont visibles si aucune arête n'est entre les deux points

En O(n^3), pas génial

## 2D un peu optimisé
Pour chaque sommet, on fait tourner un rayon qui avec un tri et un arbre permet de faire moins de calculs

En O(n^2 log n), améliorable en O(n^2)

## 2D optimisé
En O(n log n), c'est un des buts du TIPE

## 3D RRT*
Algorithme:
On génère un arbre partant du point de départ
On tente pendant un certain temps:
- On prend un point au hasard
- On prend le sommet le plus proche, et on rapproche le nouveau point vers celui-ci
- On prend le sommet proche visible minimisant la distance avec la racine (on repart au début s'il n'existe pas)
- On prend des points proches et on change leur parent en ce nouveau point si cela diminue leur distance
Puis on voit si on a trouvé un chemin

Cela demande:
- Une structure pour trouver les voisins dans un rayon R variable, et trouver le sommet le plus proche

## 3D PRM (Probabilistic RoadMaps)
On ajoute des points au hasard, on détermine les sommets proches, on relie ceux qui sont visibles

Demande de pouvoir trouver les points proches rapidement

## Grille de résolution fixe
On génère un graphe en forme de grille

Est simple à coder mais très gourmande en mémoire surtout en haute dimensions, O(Largeur^Dimension)

## Grille de taille dynamique ?
L'idée serait de faire une grille de résolution plus élevée près des obstacles

# Structures de données

## RTree
Un arbre où un noeud est un rectangle contenant touts ses fils

En utilisant STR (Sort-Tile-Recursive) on peut construire rapidement un arbre avec peu d'overlaps et atteindre des complexités en O(log n) pour les tests de collisions (n est le nombre d'obstacles)

Utile pour les tests de collisions

## BSP (Binary Space Partition) (nom générique, j'ai rien trouvé de plus spécifique)
Un peu comme un Quadtree / Octree mais ne sépare l'espace qu'en 2, une dimension après l'autre

Utile pour trouver les R plus proches voisins et le plus proche voisin

# Lancer le code:
Def façon générale, les étapes sont:
- Installer rust, via les instructions du site web
- Aller dans tests/mod.rs, décommenter le test voulu
- Lancer `cargo run`, ou `cargo run --release` pour optimiser le binaire

Pour les algos 2d, que je ne change pour le moment pas, voir la branche pre_3d, qui est plus simple à installer

Pour la dernière versions!
- installer le repos space_animation
- le mettre dans le dossier à coté de tipe_project
- lancer tipe_project normalement

Contactez moi en cas de difficultée
