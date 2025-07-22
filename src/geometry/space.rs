use std::{
    f64::consts::TAU,
    ops::{Add, Range, Sub},
};

use rand::Rng;

use crate::{
    datastructures::bintree::BinTree,
    geometry::{angles::Angle, VecN},
    utils::numbers::{NotNanF64, Zero},
};

pub trait Space: Sized + Add<Output = Self> + Sub<Output = Self> + Copy + Zero {
    type BoxDelimParam: Clone;
    // type RNeighborsIterator<'a, T: Space + 'a>: Iterator<Item = &'a BinTree<T>>;

    fn distance(self, other: Self) -> f64;
    fn lerp(self, other: Self, time: f64) -> Self;
    fn sample(param: &Self::BoxDelimParam, rng: &mut impl Rng) -> Self;

    fn r_neighbors_map<S: Space>(
        self,
        tree: &BinTree<S>,
        center: S,
        radius: f64,
    ) -> impl Iterator<Item = S>;

    fn steer_in_disc(self, center: Self, width: f64) -> Self {
        let dist = self.distance(center);
        if dist <= width {
            self
        } else {
            center.lerp(self, width / dist)
        }
    }
}
impl Space for f64 {
    type BoxDelimParam = Range<f64>;

    fn distance(self, other: Self) -> f64 {
        (self - other).abs()
    }
    fn lerp(self, other: Self, time: f64) -> Self {
        (1. - time) * self + time * other
    }
    fn sample(param: &Self::BoxDelimParam, rng: &mut impl Rng) -> Self {
        rng.random_range(param.clone())
    }

    fn r_neighbors_map<S: Space>(
        self,
        tree: &BinTree<S>,
        center: S,
        radius: f64,
    ) -> impl Iterator<Item = S> {
        [center].into_iter()
    }
}
impl Space for Angle {
    type BoxDelimParam = Range<Angle>;

    fn distance(self, other: Self) -> f64 {
        let delta = (*self - *other).abs();
        delta.min(TAU - delta)
    }
    fn lerp(self, end: Self, time: f64) -> Self {
        self + (end - self) * time
    }
    fn sample(_param: &Self::BoxDelimParam, rng: &mut impl Rng) -> Self {
        Angle(NotNanF64::new_debug_checked(rng.random_range(0.0..TAU)))
    }

    fn r_neighbors_map<S: Space>(
        self,
        tree: &BinTree<S>,
        center: S,
        radius: f64,
    ) -> impl Iterator<Item = S> {
        [center].into_iter()
    }
}
impl<const N: usize, T: Space> Space for VecN<N, T> {
    type BoxDelimParam = VecN<N, T::BoxDelimParam>;

    fn distance(self, other: Self) -> f64 {
        let mut res = 0.;
        for i in 0..N {
            let d = self[i].distance(other[i]);
            res += d * d;
        }
        res.sqrt()
    }
    fn lerp(self, other: Self, time: f64) -> Self {
        Self::from_fn(|i| self[i].lerp(other[i], time))
    }
    fn sample(param: &Self::BoxDelimParam, rng: &mut impl Rng) -> Self {
        Self::from_fn(|i| T::sample(&param[i], rng))
    }

    fn r_neighbors_map<S: Space>(
        self,
        tree: &BinTree<S>,
        center: S,
        radius: f64,
    ) -> impl Iterator<Item = S> {
        struct StackIterator<const N: usize, I> {
            stacks: VecN<N, I>,
        }
        impl<S: Space, const N: usize, I: Iterator<Item=S>> Iterator for StackIterator<N, I> {
            type Item = S;
            fn next(&mut self) -> Option<Self::Item> {
                todo!()
            }
        }
        StackIterator
    }
}
