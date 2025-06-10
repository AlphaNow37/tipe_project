use rand::{rng, Rng};

use crate::geometry::shapes::Polygon;

mod test_complex;
mod test_graph;
mod test_perfs;
mod test_simple;
mod background;

pub const OUT: &str = "/home/alpha_now/Desktop/progs/tipe_project/out";

const GIGGLE_INTENSITY: f64 = 0.001;

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
    test_complex::test_square_map();
    // test_perfs::test_perf();
    // test_simple::test_pretty_simple();
    // background::generate_backgrounds();
}
