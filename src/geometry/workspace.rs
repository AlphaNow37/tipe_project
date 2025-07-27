use core::f64;
use std::{fmt::Debug, hash::Hash};

use rand::Rng;

use crate::geometry::VecN;

/// Représente la "forme" de l'espace de travail ainsi que le système de coordonnées
/// Ces noms sont sujets à changement
pub trait WorkspaceTopology: Clone {
    type Vertex: Copy + Eq + Hash + Debug;

    /// Retourne la distance entre deux points dans cet espace
    /// Doit satisfaire les conditions habituelles d'une distance
    fn distance(&self, a: Self::Vertex, b: Self::Vertex) -> f64;
    /// Retourne un point res du segment [a; b] tel que dist(a, res) = time * dist(a, b)
    /// time doit être dans [0; 1]
    fn lerp(&self, a: Self::Vertex, b: Self::Vertex, time: f64) -> Self::Vertex;
    /// Retourne un point de l'espace choisie de façon aléatoire
    fn sample_random(&self, rng: &mut impl Rng) -> Self::Vertex;
    /// Retourne un point du segment [center; pt] tel que dist(center, res) <= radius
    fn steer_in_disc(&self, pt: Self::Vertex, center: Self::Vertex, radius: f64) -> Self::Vertex {
        let dist = self.distance(pt, center);
        if dist <= radius {
            pt
        } else {
            self.lerp(center, pt, radius / dist)
        }
    }
}

/// Une espace de travail constitué de N coordonnées, pouvant potentiellement boucler sur elle même, et dotée d'une distance D
/// Exemple: [0; 1] x [0; 1] x [0; 1], ou des angles, ..
#[derive(Clone, Copy, Debug)]
pub struct UniformTopology<const N: usize, D> {
    pub dist: D,
    pub offsets: VecN<N, f64>,
    pub sizes: VecN<N, f64>,
    pub is_torus: VecN<N, bool>,
}
impl<const N: usize, D> UniformTopology<N, D> {
    pub const fn new_borderless(dist: D) -> Self {
        Self {
            dist,
            offsets: VecN::splat(f64::NEG_INFINITY),
            sizes: VecN::splat(f64::INFINITY),
            is_torus: VecN::splat(false),
        }
    }
}
impl<const N: usize, D: Length<N>> WorkspaceTopology for UniformTopology<N, D> {
    type Vertex = VecN<N, f64>;

    fn distance(&self, a: Self::Vertex, b: Self::Vertex) -> f64 {
        self.dist.length(VecN::from_fn(|i| {
            let diff = (a[i] - b[i]).abs();
            if self.is_torus[i] && self.sizes[i] - diff < diff {
                self.sizes[i] - diff
            } else {
                diff
            }
        }))
    }
    fn lerp(&self, a: Self::Vertex, b: Self::Vertex, time: f64) -> Self::Vertex {
        debug_assert!(0. <= time && time <= 1.);
        VecN::from_fn(|i| {
            if self.is_torus[i] {
                let diff = (a[i] - b[i]).abs();
                if self.sizes[i] - diff < diff {
                    if a[i] < b[i] {
                        let mut result = a[i] + (-self.sizes[i] - a[i] + b[i]) * time;
                        if result < self.offsets[i] {
                            result += self.sizes[i];
                        }
                        return result;
                    } else {
                        let mut result = a[i] + (self.sizes[i] - a[i] + b[i]) * time;
                        if result >= self.offsets[i] + self.sizes[i] {
                            result -= self.sizes[i];
                        }
                        return result;
                    }
                }
            }
            (1. - time) * a[i] + time * b[i]
        })
    }
    fn sample_random(&self, rng: &mut impl Rng) -> Self::Vertex {
        VecN::from_fn(|i| rng.random_range(self.offsets[i]..(self.offsets[i] + self.sizes[i])))
    }
}

/// Représente une fonction de norme d'un vecteur.
/// Doit satisfaire les conditions habituelles d'une norme
pub trait Length<const N: usize>: Clone + Debug {
    fn length(&self, vector: VecN<N, f64>) -> f64;
}

/// La norme euclidienne, sqrt(sum x^2), L_2
#[derive(Clone, Copy, Debug)]
pub struct EuclidianDistance;
impl<const N: usize> Length<N> for EuclidianDistance {
    fn length(&self, vector: VecN<N, f64>) -> f64 {
        let mut sm = 0.0f64;
        for d in *vector {
            sm += d * d;
        }
        sm.sqrt()
    }
}

/// La norme de Manathann, sum |x|, L_1
#[derive(Clone, Copy, Debug)]
pub struct ManathannDistance;
impl<const N: usize> Length<N> for ManathannDistance {
    fn length(&self, vector: VecN<N, f64>) -> f64 {
        let mut sm = 0.0f64;
        for d in *vector {
            sm += d.abs();
        }
        sm
    }
}

/// La norme de Tchebychev, max |x|, L_inf
#[derive(Clone, Copy, Debug)]
pub struct TchebychevDistance;
impl<const N: usize> Length<N> for TchebychevDistance {
    fn length(&self, vector: VecN<N, f64>) -> f64 {
        let mut mx = 0.0f64;
        for d in *vector {
            mx = mx.max(d.abs());
        }
        mx
    }
}

pub fn path_length<W: WorkspaceTopology>(workspace: &W, path: &[W::Vertex]) -> f64 {
    (0..(path.len() - 1))
        .map(|i| workspace.distance(path[i], path[i + 1]))
        .sum::<f64>()
}
