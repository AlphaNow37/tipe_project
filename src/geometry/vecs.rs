use core::f64;
use std::{
    fmt::Debug,
    hash::Hash,
    ops::{Add, Deref, DerefMut, Div, Mul, Neg, Sub},
};
use crate::utils::numbers::{Zero, F64_EPSILON};

/// N-dimensions point/vector
#[derive(Clone, Copy, PartialEq, PartialOrd)]
pub struct VecN<const N: usize, T>(pub [T; N]);

impl<const N: usize, T: Copy> VecN<N, T> {
    pub fn from_fn(f: impl FnMut(usize) -> T) -> Self {
        Self(std::array::from_fn(f))
    }
    pub const fn splat(value: T) -> Self {
        Self([value; N])
    }
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
impl<T: Neg<Output = T>> VecN<2, T> {
    pub fn rotate_left(self) -> Self {
        let [x, y] = self.0;
        Self([-y, x])
    }
    pub fn rotate_right(self) -> Self {
        let [x, y] = self.0;
        Self([y, -x])
    }
}
impl<const N: usize> VecN<N, f64> {
    // self * ratio = other
    pub fn colinear_ratio(self, other: Self) -> Option<f64> {
        // None=on sait pas encore, Some None = pas colineo, Some Some r = ratio r
        let mut ratio = None;
        for i in 0..N {
            if self[i].abs() < F64_EPSILON {
                continue;
            }
            let rat = other[i] / self[i];
            ratio = match ratio {
                None => Some(Some(rat)),
                Some(None) => Some(None),
                Some(Some(r)) => {
                    if (r - rat).abs() < F64_EPSILON {
                        Some(Some(r))
                    } else {
                        Some(None)
                    }
                }
            }
        }

        ratio.unwrap_or(Some(f64::INFINITY))
    }

    pub fn max_component_index(self) -> usize {
        debug_assert!(N > 0);
        let mut mx_i = 0;
        for i in 1..N {
            if self[i] > self[mx_i] {
                mx_i = i;
            }
        }
        mx_i
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
impl<const N: usize, T> DerefMut for VecN<N, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
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
impl<const N: usize, T: Div<f64, Output = T>> Div<f64> for VecN<N, T> {
    type Output = Self;
    fn div(self, rhs: f64) -> Self::Output {
        Self(self.0.map(|elt| elt / rhs))
    }
}
impl<const N: usize, T: Mul<f64, Output = T>> Mul<f64> for VecN<N, T> {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self::Output {
        Self(self.0.map(|elt| elt * rhs))
    }
}
impl<const N: usize, T: Neg<Output = T>> Neg for VecN<N, T> {
    type Output = Self;
    fn neg(self) -> Self::Output {
        Self(self.0.map(|elt| -elt))
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

// Necessary in order to store them in hashmaps.. but not really sound on NaN, but i don't want to use NotNanF64 everywhere
impl<const N: usize> Hash for VecN<N, f64> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        // Faster than individual writes
        let len_u8 = 4 * N;
        let ptr = self.0.as_ptr() as *const u8;
        state.write(unsafe { std::slice::from_raw_parts(ptr, len_u8) })
    }
}
impl<const N: usize> Eq for VecN<N, f64> {}

#[cfg(feature = "gpu_vis")]
impl Into<lib_space_animation::math::Vec3> for VecN<3, f64> {
    fn into(self) -> lib_space_animation::math::Vec3 {
        lib_space_animation::math::Vec3::from_array(self.0.map(|c| c as f32))
    }
}

#[cfg(feature = "polyanya")]
impl Into<polyanya::geo::Coord<f32>> for VecN<2, f64> {
    fn into(self) -> polyanya::geo::Coord<f32> {
        polyanya::geo::Coord::from(self.0.map(|f| f as f32))
    }
}
#[cfg(feature = "polyanya")]
impl Into<polyanya::Coords> for VecN<2, f64> {
    fn into(self) -> polyanya::Coords {
        glam::f32::Vec2::from(self.0.map(|f| f as f32)).into()
    }
}
