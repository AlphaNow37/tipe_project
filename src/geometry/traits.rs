use std::ops::{Add, Sub};

pub trait Zero {
    const ZERO: Self;
}
impl Zero for usize {
    const ZERO: Self = 0;
}
impl Zero for f64 {
    const ZERO: Self = 0.;
}

pub trait RealLike {}

pub trait VectorSpace<K>: Add<Output = Self> + Sub<Output = Self> + Zero + Sized {}
