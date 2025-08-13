use rand::{rng, Rng};
use std::env::current_dir;
use std::path::{Path, PathBuf};

use crate::geometry::shapes::Polygon;

mod background;
mod test_conv_path;
mod test_datastructures_2d;
mod test_datastructures_3d;
mod test_graph_2d;
mod test_path_3d;
mod test_path_complex_2d;
mod test_path_simple_2d;
mod test_perf_path_2d;

pub fn out_dir() -> PathBuf {
    let here = current_dir().expect("Expected a working directory");

    let root = here
        .ancestors()
        .find(|p| p.ends_with(Path::new("tipe_project")))
        .expect("There should be an ancestor named tipe_project");
    let out = root.join("out");
    dbg!(&out);
    out
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
    // test_perf_path_2d::test_perf();
    // test_path_simple_2d::test_pretty_simple();
    // background::generate_backgrounds();
    test_path_3d::test_3d()
    // test_rtree_3d::test_rtree()
    // test_datastructures_2d::test_rtree_2d()
    // test_datastructures_2d::test_grid_2d()
}
