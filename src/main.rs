use std::path::PathBuf;

use geometry::VecN;
use svg::object::Polygon;

pub mod datastructures;
pub mod geometry;
pub mod graphs;
pub mod macros;
pub mod svg;

fn main() {
    let out = PathBuf::from("/home/alpha_now/Desktop/progs/tipe_project/out");

    let mut svg = svg::SvgGroup::default();
    svg.push(
        Polygon {
            color: "red".to_string(),
            points: vec![
                VecN([10., 30.]),
                VecN([60., 100.]),
                VecN([20., -50.]),
                VecN([25., 10.]),
            ],
        },
        0.,
    );

    svg.write_to_file(&out.join("test.svg"));
}
