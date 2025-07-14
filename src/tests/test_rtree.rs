use crate::datastructures::r_tree::RTree;
use crate::geometry::shapes::Cube;
use crate::geometry::VecN;
use crate::render_3d::cubes::place_cubes;
use lib_space_animation::math::Transform;
use lib_space_animation::world::primitives::color::Color;
use lib_space_animation::world::world_builder::{WorldBuilder, WorldsBuilder};
use rand::{rng, Rng};
use std::array::from_fn;

const NCUBES: usize = 20;
const TOTAL_WIDTH: f64 = 20.0;
const MIN_WIDTH: f64 = 0.01;
const MAX_WIDTH: f64 = 3.;

pub fn test_rtree() {
    let mut rng = rng();

    let mut cubes = (0..NCUBES)
        .map(|_| {
            let p1: VecN<3, f64> = VecN(from_fn(|_| {
                rng.random_range(0.0..(TOTAL_WIDTH - MAX_WIDTH).powi(2)).sqrt()
            }));
            let p2: VecN<3, f64> = p1 + VecN(from_fn(|_| rng.random_range(MIN_WIDTH..MAX_WIDTH)));
            Cube::from_point(p1).with_point(p2)
        })
        .collect::<Vec<_>>();
    let rtree = RTree::bulk_load(&mut cubes);

    lib_space_animation::run(move || {
        let mut ws = WorldsBuilder::default();
        let mut w = ws.add_world(0);
        place_cubes(&mut w, &cubes, Color::RED, Transform::ID, true);

        fn place_rtree(rtree: &RTree<3, Cube<3>>, w: &mut WorldBuilder, d: usize) {
            match rtree {
                RTree::Leaf(c) => (),
                RTree::Node {
                    bounding_box,
                    children,
                } => {
                    place_cubes(
                        w,
                        &[*bounding_box],
                        [Color::BLUE, Color::GREEN, Color::YELLOW][d % 3],
                        Transform::ID,
                        false,
                    );
                    for c in children {
                        place_rtree(c, w, d + 1);
                    }
                }
            }
        }
        place_rtree(&rtree, &mut w, 0);

        w.finalize()
    });
}
