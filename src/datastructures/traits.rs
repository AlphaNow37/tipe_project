use crate::macros::make_trait_alias;

use crate::geometry::traits::Zero;
use std::{
    hash::Hash,
    ops::{Add, Deref, Mul, Sub},
};

make_trait_alias!(Weight = [Sized + Zero + Add<Output=Self> + Ord + Copy] {});

/// A Non-NaN, hashable float
#[derive(Default, Debug, PartialEq, PartialOrd, Clone, Copy)]
pub struct NotNanF64(f64);
impl NotNanF64 {
    pub const fn new(v: f64) -> Self {
        if v.is_nan() {
            panic!("Got a NaN float");
        }
        Self(v)
    }
}
impl Deref for NotNanF64 {
    type Target = f64;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl Eq for NotNanF64 {}
impl Ord for NotNanF64 {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0
            .partial_cmp(&other.0)
            .expect(&format!("Can't compare {} and {}", self.0, other.0))
    }
}
impl Hash for NotNanF64 {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        state.write_u64(self.0.to_bits());
    }
}
impl Add for NotNanF64 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self(*self + *rhs)
    }
}
impl Sub for NotNanF64 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self(*self - *rhs)
    }
}
impl Mul for NotNanF64 {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self::Output {
        Self(*self * *rhs)
    }
}
