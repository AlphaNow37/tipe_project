use std::{
    f64::consts::TAU,
    ops::{Add, Sub},
};

use crate::{
    geometry::{angles::Angle, VecN},
    utils::numbers::Zero,
};

pub trait Space: Sized + Add<Output = Self> + Sub<Output = Self> + Copy + Zero {
    fn distance(self, other: Self) -> f64;
    fn lerp(self, other: Self, time: f64) -> Self;

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
    fn distance(self, other: Self) -> f64 {
        (self - other).abs()
    }
    fn lerp(self, other: Self, time: f64) -> Self {
        (1. - time) * self + time * other
    }
}
impl Space for Angle {
    fn distance(self, other: Self) -> f64 {
        let delta = (*self - *other).abs();
        delta.min(TAU - delta)
    }
    fn lerp(self, end: Self, time: f64) -> Self {
        self + (end - self) * time
    }
}
impl<const N: usize, T: Space> Space for VecN<N, T> {
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
}
