/// Ce dossier définit les espaces de travail
/// Un espace de travail définit:
/// - Quel type utiliser pour les positions, pour les géodésiques
/// - Quel type utiliser pour modéliser l'environnement, les obstacles
/// - Comment echantilloner un point au hasard
/// - Quel fonction de distance utiliser
/// - Quel type utiliser pour les requêtes type R-nearest neighbors


pub mod workspace;
pub mod obstacles;
pub mod geometrical_queries;
pub mod reeds_shepp;
pub mod cartesians;
// pub mod dubins;
