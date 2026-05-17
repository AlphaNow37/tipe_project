use std::fmt::Debug;
use std::ops::Add;

use crate::utils::numbers::Zero;

use super::macros::make_trait_alias;

make_trait_alias!(Weight = [Sized + Zero + Add<Output=Self> + Ord + Copy + Debug] {});
