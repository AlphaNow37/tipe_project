use crate::datastructures::bsp::Bsp;
use crate::datastructures::r_tree::RTree;
use crate::geometry::angles::Angle;
use crate::geometry::shapes::{Cube, Segment};
use crate::geometry::VecN;
use crate::path_planning::graphs_heuristics::{
    prm, rrt, rrt_star, ContinueUntil, GraphHeuristicParameters,
};
use crate::render_3d::cubes::place_cubes;
use crate::utils::numbers::Zero;
use crate::workspace::cartesians::{LoopingCartesianTopology, TchebychevDistance};
use crate::workspace::obstacles::ObstaclesApprox;
use crate::workspace::workspace::WorkspaceTopology;
use lib_space_animation::math::{rotate_x, rotate_y, rotate_z, scale, trans, Transform, Vec3};
use lib_space_animation::utils::Length;
use lib_space_animation::world::primitives::color::Color;
use lib_space_animation::world::visuals::Pipe;
use lib_space_animation::world::world::Worlds;
use lib_space_animation::world::world_builder::WorldsBuilder;
use std::array::from_fn;
use std::f64::consts::PI;
use std::marker::PhantomData;
use std::time::{Duration, Instant};

const NARMS: usize = 5;
const NDIM: usize = NARMS * 2;
const LENGTHS: [f64; NARMS] = [1., 1., 1., 1., 1.];
const CENTER: Transform = trans(1.75, 1.75, 1.75);
const SPEED: f64 = 0.5;

fn intermediate_position(angles: VecN<NDIM, f64>) -> VecN<{ NARMS + 1 }, VecN<3, f64>> {
    let mut res = VecN::splat(VecN::ZERO);
    let mut curr_transform = CENTER;
    res[0] = VecN(CENTER.trans().to_array().map(|c| c as f64));
    for i in 0..NARMS {
        curr_transform = curr_transform
            * rotate_z(lib_space_animation::math::Angle::from_rad(
                angles[2 * i] as f32,
            ))
            * rotate_y(lib_space_animation::math::Angle::from_rad(
                angles[2 * i + 1] as f32,
            ))
            * trans(0., 0., LENGTHS[i] as f32);
        res[i + 1] = VecN(curr_transform.trans().to_array().map(|c| c as f64));
    }
    res
}

pub fn test_arms_simple() {
    let mut cubes = vec![
        // Cube::from_point(VecN([0., 0., 0.])).with_point(VecN([1., 1., 1.])),
        // Cube::from_point(VecN([2., 0., 0.])).with_point(VecN([3., 1., 1.])),
        // Cube::from_point(VecN([0., 2., 0.])).with_point(VecN([1., 3., 1.])),
        // Cube::from_point(VecN([0., 0., 2.])).with_point(VecN([1., 1., 3.])),
        // Cube::from_point(VecN([1., 1., 1.])).with_point(VecN([1.5, 1.5, 1.5])),
        // Cube::from_point(VecN([0., 0., 0.])).with_point(VecN([2., 2., 2.])),
        // Cube::from_point(VecN([0., 0., 0.])).with_point(VecN([2., 2., 2.])),
        // Cube::from_point(VecN([0., 0., 0.])).with_point(VecN([2., 2., 2.])),
        // Cube::from_point(VecN([0., 0., 0.])).with_point(VecN([2., 2., 2.])),
    ];
    for i in 0..5 {
        for j in 0..5 {
            for k in 0..5 {
                if (i + j + k) % 2 == 0 {
                    cubes.push(
                        Cube::from_point(VecN([i as f64, j as f64, k as f64])).with_point(VecN([
                            i as f64 + 0.5,
                            j as f64 + 0.5,
                            k as f64 + 0.5,
                        ])),
                    )
                }
            }
        }
    }

    lib_space_animation::run(move || {
        let worlds = WorldsBuilder::default();

        let mut world = worlds.add_world(0);
        let obstacles_tr = trans(-2., 0., -2.);
        let mut cubes2 = cubes.clone();
        cubes2.clear();
        cubes2.push(Cube::from_point(VecN([10., 10., 10.])).with_point(VecN([11., 11., 11.])));
        let obstacles = RTree::bulk_load(&mut cubes);

        let start = VecN([0.; NDIM]);
        let end = VecN([PI; NDIM]);

        let workspace = LoopingCartesianTopology::<NDIM, _> {
            dist: TchebychevDistance,
            is_torus: VecN::splat(true),
            offsets: VecN::splat(0.),
            sizes: VecN::splat(2. * PI),
        };

        place_cubes(&mut world, &cubes, Color::BLUE, obstacles_tr, true);

        let is_in_obstacles = |angles| {
            let pos = intermediate_position(angles);
            for i in 0..NARMS {
                if obstacles.intersect_segment(Segment {
                    start: pos[i],
                    end: pos[i + 1],
                }) {
                    return true;
                }
            }
            false
        };

        match prm(GraphHeuristicParameters {
            start,
            end,
            workspace,
            vertices: PhantomData::<(Bsp<NDIM>, LoopingCartesianTopology<NDIM, _>)>,
            execution_manager: ContinueUntil(Instant::now() + Duration::from_secs_f64(20.)),
            moving_radius: 2.,
            base_rewire_radius: 2.,
            obstacles: &ObstaclesApprox {
                workspace,
                contains_func: Box::new(is_in_obstacles),
                visible_resolution: 0.1,
            },
        })
        .0
        {
            None => {
                println!("Aucun chemin trouvé !")
            }
            Some((path, _)) => {
                println!("Un chemin a été trouvé !");
                let lengths = path
                    .iter()
                    .map(|(a, b)| workspace.distance(*a, *b))
                    .collect::<Vec<_>>();
                let total_length = lengths.iter().sum::<f64>();
                let x_at_beginning = lengths
                    .iter()
                    .scan(0., |curr_x, l| {
                        *curr_x += l;
                        Some(*curr_x - l)
                    })
                    .collect::<Vec<_>>();
                let total_time = total_length / SPEED;
                let col = world.push(Color::RED);
                let tr = world.push(obstacles_tr);
                world.push_visual((col, tr));
                let trs = world.push_multi(move |w: &Worlds| {
                    let time = (total_time - w.settings.base_time as f64 % (2. * total_time)).abs();
                    let x = time * SPEED;
                    let i = x_at_beginning
                        .binary_search_by(|a| a.total_cmp(&x))
                        .unwrap_or_else(|i| i - 1);
                    let part = (x - x_at_beginning[i]) / lengths[i];
                    let angles = workspace.lerp(path[i], part);
                    let pos = intermediate_position(angles);
                    let pos_vec3 = pos
                        .0
                        .map(|p| Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32));
                    let trs: [Transform; NARMS] = from_fn(|i| {
                        Transform::from_transv(pos_vec3[i])
                            * Transform::from_z_looking_at(pos_vec3[i + 1] - pos_vec3[i])
                            * scale(0.05, 0.05, (pos_vec3[i + 1] - pos_vec3[i]).length())
                    });
                    trs
                });
                for i in 0..NARMS {
                    world.push_visual(Pipe(trs[i]));
                }
            }
        }

        let worlds = world.finalize();
        worlds
    });
}
