use std::marker::PhantomData;

use crate::datastructures::r_tree::RTree;
use crate::geometry::shapes::Cube;
use crate::geometry::workspace::{
    self, EuclidianDistance, Length, TchebychevDistance, UniformTopology, WorkspaceTopology
};
use crate::geometry::VecN;
use crate::path_planning::accessibility_grid::AccesibilityGrid;
use crate::path_planning::graphs_heuristics::{prm, rrt, GraphHeuristicParameters};
use crate::render_3d::cubes::place_cubes;
use crate::render_3d::graphs::place_graph;
use crate::render_3d::grid::place_grid;
use lib_space_animation::math::{trans, Transform};
use lib_space_animation::world::primitives::color::Color;
use lib_space_animation::world::world_builder::{WorldBuilder, WorldsBuilder};

const HEURISTIC: Heuristic = Heuristic::Prm;

#[derive(Eq, PartialEq)]
enum Heuristic {
    Grid,
    Prm,
    Rrt,
    RrtStar,
    RrtFn,
    RrtMarch,
}

fn using_grids(
    world: &mut WorldBuilder,
    obstacles: RTree<3, Cube<3>>,
    obstacles_tr: Transform,
    workspace: UniformTopology<3, impl Length<3>>,
    start: VecN<3, f64>,
    end: VecN<3, f64>,
) -> Option<(Vec<VecN<3, f64>>, f64)> {
    let grid = AccesibilityGrid::new_with_rtree(
        &obstacles,
        0.04,
        Cube {
            start: workspace.offsets,
            end: workspace.offsets + workspace.sizes,
        },
    );
    place_grid(world, &grid, obstacles_tr);

    grid.shortest_path(start, end, workspace)
}

fn using_graph_heuristic<W: WorkspaceTopology<Vertex = VecN<3, f64>>>(
    world: &mut WorldBuilder,
    obstacles: RTree<3, Cube<3>>,
    obstacles_tr: Transform,
    heuristic: Heuristic,
    workspace: W,
    start: VecN<3, f64>,
    end: VecN<3, f64>,
) -> Option<(Vec<VecN<3, f64>>, f64)> {
    let params = GraphHeuristicParameters {
        start: VecN([0.2, 0.4, 0.4]),
        end: VecN([3.8, 0.4, 0.4]),
        moving_radius: 0.1,
        obstacles: &obstacles,
        workspace,
        vertices: PhantomData::<(Vec<W::Vertex>, W)>,
    };

    let pos = |p| p;
    let color = Color::GREEN;
    let width = 0.01;

    let path = match heuristic {
        Heuristic::Prm => {
            let (path, graph) = prm(params);
            place_graph(world, &graph, pos, color, width, obstacles_tr);
            path
        }
        Heuristic::Rrt => {
            let (path, graph) = rrt(params);
            place_graph(world, &graph, pos, color, width, obstacles_tr);
            path
        }
        _ => unreachable!(),
    };

    path
}

pub fn test_3d() {
    let mut cubes = vec![
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
        let obstacles = RTree::bulk_load(&mut cubes);
        let start = VecN([0.2, 0.2, 0.4]);
        let end = VecN([3.8, 0.4, 0.4]);
        let workspace = UniformTopology {
            dist: TchebychevDistance,
            is_torus: VecN::splat(false),
            offsets: VecN::splat(0.),
            sizes: VecN([4., 1., 1.]),
        };

        place_cubes(&mut world, &cubes[..5], Color::BLUE, obstacles_tr, true);
        place_cubes(&mut world, &cubes[5..], Color::RED, obstacles_tr, true);

        let path_opt = if HEURISTIC == Heuristic::Grid {
            using_grids(&mut world, obstacles, obstacles_tr, workspace, start, end)
        } else {
            using_graph_heuristic(
                &mut world,
                obstacles,
                obstacles_tr,
                HEURISTIC,
                workspace,
                start,
                end,
            )
        };

        match path_opt {
            None => println!("No path found !"),
            Some((path, _)) => {
                place_graph(
                    &mut world,
                    &(0..path.len()),
                    |i| path[i],
                    Color::YELLOW,
                    0.05,
                    obstacles_tr,
                );
            }
        }

        let tr_axis = world.push(trans(0., 0., 2.));
        lib_space_animation::models::put_axis(&mut world, tr_axis);

        let worlds = world.finalize();
        worlds
    });
}
