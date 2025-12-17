use crate::datastructures::bsp::Bsp;
use crate::datastructures::r_tree::RTree;
use crate::geometry::shapes::Cube;
use crate::geometry::VecN;
use crate::path_planning::graphs_heuristics::{
    prm, rrt_star, ContinueUntil, GraphHeuristicParameters,
};
use crate::render_3d::cubes::place_cubes;
use crate::render_3d::graphs::{place_graph, place_graph_populars};
use crate::workspace::cartesians::{CartesianTopology, EuclidianDistance};
use lib_space_animation::math::Transform;
use lib_space_animation::world::primitives::color::Color;
use lib_space_animation::world::world_builder::WorldsBuilder;
use std::marker::PhantomData;
use std::time::{Duration, Instant};

pub fn test_3d_illustration_shortest_euclidian_path() {
    let x0 = 0.1;
    let x1 = 0.7;
    let x2 = 0.7;
    let y0 = 1.;
    let mut cubes = vec![
        Cube::from_point(VecN([-x0, -x0, 0.])).with_point(VecN([x0, x0, 6. * y0])),
        Cube::from_point(VecN([-x0, -x1, 1. * y0])).with_point(VecN([x1, x1, 1. * y0 + 0.1])),
        Cube::from_point(VecN([-x2, -x0, 2. * y0])).with_point(VecN([x2, x2, 2. * y0 + 0.1])),
        Cube::from_point(VecN([-x2, -x2, 3. * y0])).with_point(VecN([x0, x2, 3. * y0 + 0.1])),
        Cube::from_point(VecN([-x2, -x2, 4. * y0])).with_point(VecN([x2, x0, 4. * y0 + 0.1])),
        Cube::from_point(VecN([-x0, -x1, 5. * y0])).with_point(VecN([x1, x1, 5. * y0 + 0.1])),
        // Cube::from_point(VecN([-x1-0.1, -x1-0.1, 0.])).with_point(VecN([x1+0.1, -x1+0.1, 6.])),
    ];
    let obstacles = RTree::bulk_load(&mut cubes);

    let start = VecN([x0 + 0.1, x0 + 0.1, 0.]);
    let end = VecN([x0 + 0.1, -x0 + 0.1, 6. * y0]);
    let workspace = CartesianTopology {
        dist: EuclidianDistance,
        space: Cube {
            start: VecN([-x1 * 1.5, -x1 * 1.5, -0.1]),
            end: VecN([x1 * 1.5, x1 * 1.5, 6. * y0 + 0.1]),
        },
    };

    lib_space_animation::run(move || {
        let mut ws = WorldsBuilder::default();
        let mut w = ws.add_world(0);

        let obstacles_tr = Transform::ID;
        place_cubes(&mut w, &cubes[..1], Color::RED, obstacles_tr, true);
        place_cubes(&mut w, &cubes[1..], Color::BLUE, obstacles_tr, true);

        let params = GraphHeuristicParameters {
            start,
            end,
            moving_radius: 1.,
            base_rewire_radius: 10.,
            obstacles: &obstacles,
            workspace,
            vertices: PhantomData::<(Bsp<3>, CartesianTopology<3, EuclidianDistance>)>,
            // execution_manager: ContinueUntil(Instant::now() + Duration::from_secs_f64(0.003)),
            execution_manager: ContinueUntil(Instant::now() + Duration::from_secs_f64(3.)),
        };

        let (path_opt, graph) = prm(params);

        // place_graph_populars(&mut w, &graph, |p| p, Color::GREEN, 0.01, obstacles_tr);
        // dbg!(graph.nb_links());

        match path_opt {
            None => println!("No path found !"),
            Some((path, _)) => {
                let mut pts: Vec<VecN<3, f64>> = path.iter().map(|s| s.start).collect();
                pts.push(path.last().unwrap().end);
                place_graph(
                    &mut w,
                    &(0..pts.len()),
                    |i| pts[i],
                    Color::YELLOW,
                    0.07,
                    obstacles_tr,
                );
            }
        }

        w.finalize()
    });
}
