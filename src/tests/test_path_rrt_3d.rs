use std::marker::PhantomData;

use crate::datastructures::r_tree::RTree;
use crate::geometry::obstacles::ObstaclesApprox;
use crate::geometry::shapes::Cube;
use crate::geometry::workspace::{EuclidianDistance, UniformTopology, WorkspaceTopology};
use crate::geometry::VecN;
use crate::path_planning::graphs_heuristics::{prm, rrt, GraphHeuristicParameters};
use crate::render_3d::cubes::place_cubes;
use crate::render_3d::graphs::place_graph;
use lib_space_animation::math::trans;
use lib_space_animation::world::primitives::color::Color;
use lib_space_animation::world::world_builder::WorldsBuilder;

pub fn test_rrt_3d() {
    let cubes = vec![
        Cube::from_point(VecN([-0.1, -0.1, -0.1])).with_point(VecN([4.1, 1.1, 0.1])),
        Cube::from_point(VecN([-0.1, -0.1, -0.1])).with_point(VecN([4.1, 0.1, 1.1])),
        Cube::from_point(VecN([-0.1, -0.1, -0.1])).with_point(VecN([0.1, 1.1, 1.1])),
        Cube::from_point(VecN([-0.1, 1.1, -0.1])).with_point(VecN([4.1, 0.9, 1.1])),
        Cube::from_point(VecN([4.1, -0.1, -0.1])).with_point(VecN([3.9, 1.1, 1.1])),
        Cube::from_point(VecN([0.95, 0., 0.])).with_point(VecN([1.05, 0.75, 1.0])),
        Cube::from_point(VecN([1.95, 0.25, 0.])).with_point(VecN([2.05, 1., 1.0])),
        Cube::from_point(VecN([2.95, 0., 0.35])).with_point(VecN([3.05, 1., 1.0])),
    ];

    lib_space_animation::run(move || {
        let worlds = WorldsBuilder::default();

        let mut world = worlds.add_world(0);
        let obstacles_tr = trans(-2., 0., -2.);

        place_cubes(&mut world, &cubes[..5], Color::BLUE, obstacles_tr, true);
        place_cubes(&mut world, &cubes[5..], Color::RED, obstacles_tr, true);

        let mut cubes2 = cubes.clone();
        let tree = RTree::bulk_load(&mut cubes2);

        type W = UniformTopology<3, EuclidianDistance>;
        let workspace: W = UniformTopology {
            dist: EuclidianDistance,
            offsets: VecN::splat(0.),
            sizes: VecN([4., 1., 1.]),
            is_torus: VecN::splat(false),
        };

        let obstacles = ObstaclesApprox {
            visible_resolution: 0.03,
            workspace,
            contains_func: Box::new(|v| tree.contains_point(v)),
        };

        let params = GraphHeuristicParameters {
            start: VecN([0.2, 0.4, 0.4]),
            goal: VecN([3.8, 0.4, 0.4]),
            moving_radius: 0.1,
            obstacles: &obstacles,
            workspace: workspace,
            vertices: PhantomData::<(Vec<<W as WorkspaceTopology>::Vertex>, W)>,
        };

        let (path, graph) = prm(params);

        match path {
            None => println!("No path found !"),
            Some((path, _)) => {
                place_graph(
                    &mut world,
                    &(0..path.len()),
                    |i| path[i],
                    Color::YELLOW,
                    0.02,
                    obstacles_tr,
                );
            }
        }

        place_graph(&mut world, &graph, |p| p, Color::GREEN, 0.01, obstacles_tr);

        let tr_axis = world.push(trans(0., 0., 2.));
        lib_space_animation::models::put_axis(&mut world, tr_axis);

        let worlds = world.finalize();
        worlds
    });
}
