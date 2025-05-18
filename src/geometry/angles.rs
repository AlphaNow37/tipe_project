use std::{
    f64::consts::{PI, TAU},
    ops::{Add, Deref, Neg, Sub},
};

use crate::utils::numbers::{NotNanF64, Zero};

use super::VecN;

/// Angle dans le sens trigo, dans [0; 2pi[
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct Angle(pub NotNanF64);

impl Angle {
    pub const HALF: Self = Self(NotNanF64::new(PI));
    pub fn new(radians: f64) -> Self {
        Self(NotNanF64::new(radians.rem_euclid(TAU)))
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
