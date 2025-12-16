use std::marker::PhantomData;
use std::time::{Duration, Instant};
use crate::datastructures::r_tree::RTree;
use crate::geometry::shapes::Cube;
use crate::geometry::VecN;
use crate::render_3d::cubes::place_cubes;
use crate::render_3d::graphs::place_graph;
use crate::workspace::cartesians::{LoopingCartesianTopology, TchebychevDistance};
use lib_space_animation::math::trans;
use lib_space_animation::world::primitives::color::Color;
use lib_space_animation::world::world_builder::WorldsBuilder;
use crate::datastructures::bsp::Bsp;
use crate::path_planning::graphs_heuristics::{rrt_star, ContinueUntil, GraphHeuristicParameters};
use crate::utils::numbers::Zero;
use crate::workspace::obstacles::ObstaclesApprox;

pub fn test_arms_simple() {
    let mut cubes = vec![
        Cube::from_point(VecN([-0.1, -0.1, -0.1])).with_point(VecN([4.1, 1.1, 0.1])),
        Cube::from_point(VecN([-0.1, -0.1, -0.1])).with_point(VecN([4.1, 0.1, 1.1])),
        Cube::from_point(VecN([-0.1, -0.1, -0.1])).with_point(VecN([0.1, 1.1, 1.1])),
        Cube::from_point(VecN([-0.1, 1.1, -0.1])).with_point(VecN([4.1, 0.9, 1.1])),
        Cube::from_point(VecN([4.1, -0.1, -0.1])).with_point(VecN([3.9, 1.1, 1.1])),
        Cube::from_point(VecN([0.95, 0., 0.])).with_point(VecN([1.05, 0.7, 1.0])),
        Cube::from_point(VecN([1.95, 0.3, 0.])).with_point(VecN([2.05, 1., 1.0])),
        Cube::from_point(VecN([2.95, 0., 0.35])).with_point(VecN([3.05, 1., 1.0])),
    ];

    lib_space_animation::run(move || {
        let worlds = WorldsBuilder::default();

        let mut world = worlds.add_world(0);
        let obstacles_tr = trans(-2., 0., -2.);
        let obstacles = RTree::bulk_load(&mut cubes);

        let center = VecN([0.3, 0.3, 0.3]);

        let start = VecN::splat(0.);
        let end = VecN([0.5, 0.5, 0.5]);

        let workspace = LoopingCartesianTopology::<3, _> {
            dist: TchebychevDistance,
            is_torus: VecN::splat(true),
            offsets: VecN::splat(0.),
            sizes: VecN::splat(1.),
        };

        place_cubes(&mut world, &cubes[..5], Color::BLUE, obstacles_tr, true);
        place_cubes(&mut world, &cubes[5..], Color::RED, obstacles_tr, true);

        let arms_lengths = VecN([2., 1.5, 1.]);
        let intermediate_positions = |angles: VecN<3, f64>| {
            let mut poss = [VecN::ZERO; 4];
            poss[0] = center;
            for i in 0..3 {
                let angle = angles[i];
                let 
            }
            poss
        };

        let is_in_obstacles = |_| false;

        match rrt_star(GraphHeuristicParameters {
            start,
            end,
            workspace,
            vertices: PhantomData::<(Bsp<3>, LoopingCartesianTopology<3, _>)>,
            execution_manager: ContinueUntil(Instant::now() + Duration::from_secs_f64(1.)),
            moving_radius: 0.2,
            base_rewire_radius: 0.5,
            obstacles: &ObstaclesApprox {
                workspace,
                contains_func: Box::new(is_in_obstacles),
                visible_resolution: 0.1,
            }
        }).0 {
            None => {
                println!("Aucun chemin trouvé !")
            },
            Some(_) => {
                println!("Un chemin a été trouvé !")
            }
        }

        let tr_axis = world.push(trans(0., 0., 2.));
        lib_space_animation::models::put_axis(&mut world, tr_axis);

        let worlds = world.finalize();
        worlds
    });
}
