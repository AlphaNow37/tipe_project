#![feature(btree_cursors)]
#![feature(iter_map_windows)]
#![feature(iter_collect_into)]

pub mod datastructures;
pub mod geometry;
pub mod graphs;
pub mod path_planning;
pub mod svg;
mod tests;
pub mod utils;
#[cfg(feature = "gpu_vis")]
mod render_3d;
pub mod workspace;
pub mod parallel;
pub mod libs;
pub mod triangulations;

fn main() {
    tests::tests();
}
