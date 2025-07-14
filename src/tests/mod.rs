use rand::{rng, Rng};
use std::env::current_dir;
use std::path::PathBuf;

use crate::geometry::shapes::Polygon;

mod test_complex;
mod test_graph;
mod test_perfs;
mod test_simple;
mod background;
mod test_3d;
mod test_rtree;

pub fn out_dir() -> PathBuf {
    let here = current_dir()
        .expect("Expected a working directory")
        .join("out");
    here
}

const GIGGLE_INTENSITY: f64 = 0.001;

/// Modifies each coordinate by a tiny factor not to have aligned vertices (to ensure the map is valid)
pub fn giggle_coords(polys: &mut [Polygon]) {
    let mut rng = rng();
    for p in polys.iter_mut() {
        for coord in &mut p.0 {
            coord[0] += GIGGLE_INTENSITY * rng.random_range(-1.0..1.0);
            coord[1] += GIGGLE_INTENSITY * rng.random_range(-1.0..1.0);
        }
    }
}

pub fn tests() {
    // test_complex::test_square_map();
    // test_perfs::test_perf();
    // test_simple::test_pretty_simple();
    // background::generate_backgrounds();
    test_3d::test_3d()
    // test_rtree::test_rtree()
}
