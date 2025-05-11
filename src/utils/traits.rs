use std::ops::{Add, Div, Sub};

use crate::{geometry::VecN, utils::numbers::Zero};

use super::macros::make_trait_alias;

make_trait_alias!(Weight = [Sized + Zero + Add<Output=Self> + Ord + Copy] {});

pub trait NormedSpace: Sized + Add<Output = Self> + Sub<Output = Self> + Copy {
    fn length(self) -> f64;
    fn distance(self, other: Self) -> f64 {
        (self - other).length()
    }
    fn normalize(self) -> Self
    where
        Self: Div<f64, Output = Self>,
    {
        self / self.length()
    }
    fn normalize_or_zero(self) -> Self
    where
        Self: Div<f64, Output = Self> + Zero,
    {
        let l = self.length();
        if l == 0. {
            Self::ZERO
        } else {
            self / l
        }
    }
}
impl NormedSpace for f64 {
    fn length(self) -> f64 {
        self.abs()
    }
}
impl<const N: usize, T: NormedSpace> NormedSpace for VecN<N, T> {
    fn length(self) -> f64 {
        let mut res = 0.;
        for i in 0..N {
            let d = self[i].length();
            res += d * d;
        }
        res.sqrt()
    }
}
