use std::{
    fmt::Debug,
    ops::{Add, Deref, Div, Mul, Sub},
};

use crate::utils::numbers::Zero;

#[derive(Clone, Copy, Eq, PartialEq)]
pub struct VecN<const N: usize, T>(pub [T; N]);

impl<const N: usize, T: Copy> VecN<N, T> {
    pub fn map_component<U>(self, f: impl FnMut(T) -> U) -> VecN<N, U> {
        VecN(self.0.map(f))
    }
    pub fn zip<U: Copy>(self, other: VecN<N, U>) -> VecN<N, (T, U)> {
        VecN(std::array::from_fn(|i| (self[i], other[i])))
    }
    pub fn dot(self, other: Self) -> T
    where
        T: Mul<Output = T> + Add<Output = T> + Zero,
    {
        self.zip(other)
            .iter()
            .fold(T::ZERO, |acc, (a, b)| acc + *a * *b)
    }
}

impl<const N: usize, T: Debug> Debug for VecN<N, T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "(")?;
        for i in 0..(N - 1) {
            write!(f, "{:.2?}, ", self[i])?;
        }
        write!(f, "{:.2?}", self[N - 1])?;
        write!(f, ")")?;
        Ok(())
    }
}
impl<const N: usize, T> Deref for VecN<N, T> {
    type Target = [T; N];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<const N: usize, T: Add<Output = T> + Clone> Add for VecN<N, T> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self(std::array::from_fn(|i| self[i].clone() + rhs[i].clone()))
    }
}
impl<const N: usize, T: Sub<Output = T> + Clone> Sub for VecN<N, T> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self(std::array::from_fn(|i| self[i].clone() - rhs[i].clone()))
    }
}
impl<const N: usize, T: Div<f64, Output = T> + Clone> Div<f64> for VecN<N, T> {
    type Output = Self;
    fn div(self, rhs: f64) -> Self::Output {
        Self(std::array::from_fn(|i| self[i].clone() / rhs))
    }
}
impl<const N: usize, T: Mul<f64, Output = T> + Clone> Mul<f64> for VecN<N, T> {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self::Output {
        Self(std::array::from_fn(|i| self[i].clone() * rhs))
    }
}
impl<const N: usize, T: Default> Default for VecN<N, T> {
    fn default() -> Self {
        Self(std::array::from_fn(|_| T::default()))
    }
}
impl<const N: usize, T: Zero> Zero for VecN<N, T> {
    const ZERO: Self = Self([T::ZERO; N]);
}
