pub mod datastructures;
pub mod geometry;
pub mod graphs;
pub mod path_planning;
pub mod svg;
mod tests;
pub mod utils;
mod render_3d;
pub mod workspace;

fn main() {
    tests::tests();
}
