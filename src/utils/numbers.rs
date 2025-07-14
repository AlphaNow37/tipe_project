use std::hash::Hash;
use std::ops::{Add, Deref, Mul, Sub};

/// A Non-NaN float (64 bits)
/// Can be hashed (there is no NaN)
#[derive(Default, Debug, PartialEq, PartialOrd, Clone, Copy)]
pub struct NotNanF64(f64);
impl NotNanF64 {
    pub const fn new(v: f64) -> Self {
        assert!(!v.is_nan());
        Self(v)
    }
    pub const fn new_debug_checked(v: f64) -> Self {
        debug_assert!(!v.is_nan());
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

/// small enough
pub const F64_EPSILON: f64 = 0.0000001;
pub fn f64_approx_zero(f: f64) -> bool {
    f.abs() < F64_EPSILON
}

/// To get the zero of a generic number
pub trait Zero {
    const ZERO: Self;
}
impl Zero for usize {
    const ZERO: Self = 0;
}
impl Zero for f64 {
    const ZERO: Self = 0.;
}
impl Zero for NotNanF64 {
    const ZERO: Self = Self::new(0.);
}

/// Utility for modular arithmetic
pub trait UsizeExt {
    fn add_rem(self, other: isize, rem: usize) -> usize;
}
impl UsizeExt for usize {
    fn add_rem(self, other: isize, rem: usize) -> usize {
        (self as isize + other).rem_euclid(rem as isize) as usize
    }
}
