use std::fmt::Debug;

use rand::Rng;

use crate::{
    geometry::{
        shapes::{Cube, Segment},
        VecN,
    },
    workspace::workspace::WorkspaceTopology,
};

/// Une espace de travail constitué de N coordonnées
/// Exemple: [0; 1] x [0; 1] x [0; 1]
#[derive(Clone, Copy, Debug)]
pub struct CartesianTopology<const N: usize, D> {
    pub dist: D,
    pub space: Cube<N>,
}
impl<const N: usize, D> CartesianTopology<N, D> {
    pub const fn new_borderless(dist: D) -> Self {
        Self {
            dist,
            space: Cube {
                start: VecN::splat(f64::NEG_INFINITY),
                end: VecN::splat(f64::INFINITY),
            },
        }
    }
}
impl<const N: usize, D: Length<N>> CartesianTopology<N, D> {
    pub fn distance_to_cube(&self, a: VecN<N, f64>, c: Cube<N>) -> f64 {
        self.dist.length(VecN::from_fn(|i| {
            if a[i] < c.start[i] {
                c.start[i] - a[i]
            } else if a[i] <= c.end[i] {
                0.
            } else {
                a[i] - c.end[i]
            }
        }))
    }
}
impl<const N: usize, D: Length<N>> WorkspaceTopology for CartesianTopology<N, D> {
    type Vertex = VecN<N, f64>;
    type Segment = Segment<N>;
    const NB_DIMENSIONS: usize = N;

    fn is_distance_symetric(&self) -> bool {
        true
    }
    fn segment(&self, start: Self::Vertex, end: Self::Vertex) -> Self::Segment {
        Segment { start, end }
    }
    fn segment_start(&self, s: Self::Segment) -> Self::Vertex {
        s.start
    }
    fn segment_end(&self, s: Self::Segment) -> Self::Vertex {
        s.end
    }
    fn segment_reverse(&self, segment: Self::Segment) -> Self::Segment {
        Segment {
            start: segment.end,
            end: segment.start,
        }
    }
    fn length(&self, s: Self::Segment) -> f64 {
        self.dist.length(s.end - s.start)
    }
    fn lerp(&self, s: Self::Segment, time: f64) -> Self::Vertex {
        debug_assert!(0. <= time && time <= 1.);
        s.start * (1. - time) + s.end * time
    }
    fn sample_random(&self, rng: &mut impl Rng) -> Self::Vertex {
        self.space.random_vertex(rng)
    }
    fn steer_in_disc(&self, mut s: Self::Segment, radius: f64) -> Self::Segment {
        let dist = self.distance(s.start, s.end);
        if dist > radius {
            s.end = self.lerp(s, radius / dist)
        }
        s
    }
}

/// Une espace de travail constitué de N coordonnées, pouvant potentiellement boucler sur elle même, et dotée d'une distance D
/// Exemple: [0; 1] x [0; 1] x [0; 1], ou des angles, ..
#[derive(Clone, Copy, Debug)]
pub struct LoopingCartesianTopology<const N: usize, D> {
    pub dist: D,
    pub offsets: VecN<N, f64>,
    pub sizes: VecN<N, f64>,
    pub is_torus: VecN<N, bool>,
}
impl<const N: usize, D: Length<N>> LoopingCartesianTopology<N, D> {
    /// Computes the (positive) distance between the two coordinates, considering only the dimension dim
    fn delta_pos(&self, coord_a: f64, coord_b: f64, dim: usize) -> f64 {
        let diff = (coord_a - coord_b).abs();
        if self.is_torus[dim] && self.sizes[dim] - diff < diff {
            self.sizes[dim] - diff
        } else {
            diff
        }
    }
    /// (minimum) distance between a and c. Assumes that c is "In one part", and does not go loop around any border
    pub fn distance_to_cube(&self, a: VecN<N, f64>, c: Cube<N>) -> f64 {
        self.dist.length(VecN::from_fn(|i| {
            if c.start[i] <= a[i] && a[i] <= c.end[i] {
                0.
            } else {
                self.delta_pos(a[i], c.start[i], i)
                    .min(self.delta_pos(a[i], c.end[i], i))
            }
        }))
    }
}
impl<const N: usize, D: Length<N>> WorkspaceTopology for LoopingCartesianTopology<N, D> {
    type Vertex = VecN<N, f64>;
    type Segment = (VecN<N, f64>, VecN<N, f64>);
    const NB_DIMENSIONS: usize = N;

    fn is_distance_symetric(&self) -> bool {
        true
    }
    fn segment(&self, start: Self::Vertex, end: Self::Vertex) -> Self::Segment {
        (start, end)
    }
    fn segment_start(&self, (start, _): Self::Segment) -> Self::Vertex {
        start
    }
    fn segment_end(&self, (_, end): Self::Segment) -> Self::Vertex {
        end
    }
    fn segment_reverse(&self, (start, end): Self::Segment) -> Self::Segment {
        (end, start)
    }
    fn length(&self, (start, end): Self::Segment) -> f64 {
        self.distance(start, end)
    }
    fn distance(&self, start: Self::Vertex, end: Self::Vertex) -> f64 {
        self.dist
            .length(VecN::from_fn(|i| self.delta_pos(start[i], end[i], i)))
    }
    fn lerp(&self, (start, end): Self::Segment, time: f64) -> Self::Vertex {
        debug_assert!(0. <= time && time <= 1.);
        VecN::from_fn(|i| {
            if self.is_torus[i] {
                let diff = (start[i] - end[i]).abs();
                if self.sizes[i] - diff < diff {
                    if start[i] < end[i] {
                        let mut result = start[i] + (-self.sizes[i] - start[i] + end[i]) * time;
                        if result < self.offsets[i] {
                            result += self.sizes[i];
                        }
                        return result;
                    } else {
                        let mut result = start[i] + (self.sizes[i] - start[i] + end[i]) * time;
                        if result >= self.offsets[i] + self.sizes[i] {
                            result -= self.sizes[i];
                        }
                        return result;
                    }
                }
            }
            (1. - time) * start[i] + time * end[i]
        })
    }
    fn sample_random(&self, rng: &mut impl Rng) -> Self::Vertex {
        VecN::from_fn(|i| rng.random_range(self.offsets[i]..(self.offsets[i] + self.sizes[i])))
    }
    fn steer_in_disc(&self, (start, end): Self::Segment, radius: f64) -> Self::Segment {
        let dist = self.distance(start, end);
        if dist <= radius {
            (start, end)
        } else {
            (start, self.lerp((start, end), radius / dist))
        }
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
