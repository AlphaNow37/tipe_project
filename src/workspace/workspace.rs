use core::f64;
use std::{fmt::Debug, hash::Hash};

use rand::Rng;

/// Un espace de travail définit:
/// - Quel type utiliser pour les positions, pour les géodésiques
/// - Quel type utiliser pour modéliser l'environnement, les obstacles
/// - Comment echantilloner un point au hasard
/// - Quel fonction de distance utiliser
/// - Quel type utiliser pour les requêtes type R-nearest neighbors

/// Représente la "forme" de l'espace de travail ainsi que le système de coordonnées
/// Ces noms sont sujets à changement
pub trait WorkspaceTopology: Clone {
    type Vertex: Copy + Eq + Hash + Debug;
    type Segment: Copy + Debug + Into<Self::Vertex>;
    const NB_DIMENSIONS: usize;
    
    /// Vrai lorsque la fonction distanc est symétrique
    fn is_distance_symetric(&self) -> bool;
    /// Retourne un segment reliant start à end
    fn segment(&self, start: Self::Vertex, end: Self::Vertex) -> Self::Segment;
    /// Premier point du segment
    fn segment_start(&self, segment: Self::Segment) -> Self::Vertex;
    /// Dernier point du segment
    fn segment_end(&self, segment: Self::Segment) -> Self::Vertex;
    /// Retourne un segment
    fn segment_reverse(&self, segment: Self::Segment) -> Self::Segment;
    /// Retourne la longueur d'un segment
    fn length(&self, segment: Self::Segment) -> f64;
    /// Retourne la distance entre deux points dans cet espace
    /// Doit satisfaire l'inégalité triangulaire mais peut être asymétrique
    fn distance(&self, start: Self::Vertex, end: Self::Vertex) -> f64 {
        self.length(self.segment(start, end))
    }
    /// Retourne un point res du segment [a; b] tel que dist(a, res) = time * dist(a, b)
    /// time doit être dans [0; 1]
    fn lerp(&self, segment: Self::Segment, time: f64) -> Self::Vertex;
    /// Retourne un point de l'espace choisie de façon aléatoire
    fn sample_random(&self, rng: &mut impl Rng) -> Self::Vertex;
    /// Tronque le segment pour que sa longueur soit inférieur à radius
    fn steer_in_disc(&self, segment: Self::Segment, radius: f64) -> Self::Segment;
}

pub fn path_length<W: WorkspaceTopology>(workspace: &W, path: &[W::Segment]) -> f64 {
    path.iter()
        .map(|s| workspace.length(*s))
        .sum::<f64>()
}
