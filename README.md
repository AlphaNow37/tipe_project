
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

## En 3D
Le problème devient alors NP-difficile
On utilise alors des heuristiques intéressantes

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

On resterait probablement en 2D

## ENS
On fait un peu tout

# Algorithmes

## 2D naif
Pour chaque paire de sommet, ils sont visibles si aucune arête n'est entre les deux points
En O(n^3), pas génial

## 2D un peu optimisé
Pour chaque sommet, on fait tourner un rayon qui avec un tri et un arbre permet de faire moins de calculs
En O(n^2 log n), améliorable en O(n^2)

## 2D optimisé
En O(n log n), c'est un des buts du TIPE

## 3D
TODO

# Lancer le code:
- Installer rust, via les instructions du site web
- Aller dans tests/mod.rs, décommenter le test voulu
- Lancer `cargo run`, ou `cargo run --release` pour optimiser le binaire
