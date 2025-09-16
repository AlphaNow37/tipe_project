use std::{
    f64::consts::{PI, TAU},
    ops::{Add, Deref, Mul, Neg, Sub},
};

use rand::{
    distr::{Distribution, StandardUniform},
    Rng,
};

use crate::utils::numbers::{NotNanF64, Zero};

use super::VecN;

/// Angle dans le sens trigo
/// Invariant: est dans [0; 2pi[
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct Angle(pub NotNanF64);

impl Angle {
    pub const HALF: Self = Self(NotNanF64::new(PI));
    pub const QUARTER: Self = Self(NotNanF64::new(PI / 2.));

    pub fn new(radians: f64) -> Self {
        Self(NotNanF64::new(radians.rem_euclid(TAU)))
    }
    pub fn from_degrees(degs: f64) -> Self {
        Self::new(degs.to_radians())
    }
    /// Angle avec la demi-droite (Ox)
    pub fn from_point(p: VecN<2, f64>) -> Self {
        Self::new(p[1].atan2(p[0]))
    }
    /// Angle (a b c)
    pub fn from_points(a: VecN<2, f64>, b: VecN<2, f64>, c: VecN<2, f64>) -> Self {
        Self::from_point(c - b) - Self::from_point(a - b)
    }
    /// Dans l'arc (a b)
    pub fn is_between(self, a: Self, b: Self) -> bool {
        self - a <= b - a
    }
    pub fn abs(self) -> Self {
        if self.is_between(Self::ZERO, Self::HALF) {
            self
        } else {
            -self
        }
    }
    /// iter_to(10°, 90°, 50°) = (10°, 50°, 90°)
    /// Returns an iterator from self to dest included
    pub fn iter_to(self, dest: Self, max_resolution: Self) -> impl Iterator<Item = Self> {
        let delta = dest - self;
        let n = (*delta.0 / *max_resolution.0).ceil();
        (0..=(n as usize))
            .map(|i| i as f64)
            .map(move |i| self + delta * (i / n))
    }
    pub fn to_vec(self) -> VecN<2, f64> {
        let (s, c) = self.0.sin_cos();
        VecN([c, s])
    }
}

impl Deref for Angle {
    type Target = f64;
    fn deref(&self) -> &Self::Target {
        &*self.0
    }
}

impl Add for Angle {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(*self.0 + *rhs.0)
    }
}
impl Sub for Angle {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(*self.0 - *rhs.0)
    }
}
impl Neg for Angle {
    type Output = Self;
    fn neg(self) -> Self::Output {
        Self::ZERO - self
    }
}
impl Zero for Angle {
    const ZERO: Self = Self(NotNanF64::ZERO);
}
impl Mul<f64> for Angle {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self::Output {
        Self::new(*self.0 * rhs)
    }
}
impl Distribution<Angle> for StandardUniform {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> Angle {
        Angle(NotNanF64::new_debug_checked(rng.random_range(0.0..TAU)))
    }
}
