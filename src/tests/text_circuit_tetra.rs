use super::{in_dir, out_dir};
use crate::datastructures::bsp::Bsp;
use crate::datastructures::r_tree::RTree;
use crate::geometry::shapes::{Cube, Polygon};
use crate::path_planning::graphs_heuristics::{ContinueUntil, Goal};
use crate::path_planning::shortcuts::shortcut;
use crate::svg::graph::put_graph;
use crate::svg::object::SvgObject;
use crate::utils::image_reader::read_image;
use crate::workspace::cartesians::{CartesianTopology, EuclidianDistance};
use crate::{
    geometry::{angles::Angle, VecN},
    graphs::{Graph, IterableGraph},
    path_planning::graphs_heuristics::{
        prm, rrt, rrt_star, GraphHeuristicParameters, SampleNTimes,
    },
    svg::{self, curves::put_reeds_shepp, object::Style},
    workspace::{
        obstacles::ObstaclesApprox,
        reeds_shepp::{OrientedCoord, ReedsSheppWorkspace},
    },
};
use std::iter::RepeatN;
use std::time::{Duration, Instant};
use std::{collections::HashSet, marker::PhantomData};

const INPUT_IMAGE_FILENAME: &'static str = "circuit_tetra_1.png";
const INPUT_GRID_FILENAME: &'static str = "circuit_tetra_1_arr.txt";
const STEERING_RADIUS: f64 = 1.8;

pub fn illustration_circuit_tetra() {
    let mut svg = svg::SvgGroup::default();
    let in_dir = in_dir();
    // let (obstacles, start_pos, end_pos) = read_image(&in_dir.clone().join(INPUT_GRID_FILENAME));

    let start_pos = VecN([0., 0.]);
    let end_pos = VecN([10., 9.]);
    let obstacles_vec = vec![
        Polygon::new(vec![
            VecN([-5., 1.]),
            VecN([2., 2.]),
            VecN([0., 7.]),
            VecN([-4., 6.]),
        ]),
        Polygon::new(vec![
            VecN([9., -1.]),
            VecN([15., 3.]),
            VecN([6., 9.]),
            VecN([5., 6.]),
        ]),
        Polygon::new(vec![
            VecN([-1., 9.]),
            VecN([5., 13.]),
            VecN([11., 10.]),
            VecN([13., 14.]),
            VecN([2., 17.]),
        ]),
    ];

    for p in obstacles_vec.iter() {
        svg.push(p.clone(), 0., Style::fill("#222222"));
    }

    let rtree = RTree::bulk_load(
        &mut obstacles_vec
            .iter()
            .map(|p| (p.clone(), p.collide_box()))
            .collect::<Vec<_>>(),
    );
    let space_bb = Cube {
        start: VecN([-6., -3.]),
        end: VecN([17., 18.]),
    };

    // dbg!(obstacles.accessible.iter().filter(|a| !**a).count());

    let workspace_straight_lines = CartesianTopology {
        dist: EuclidianDistance,
        space: space_bb,
    };

    let params = GraphHeuristicParameters {
        start: start_pos,
        end: Goal::Vertex(end_pos),
        base_rewire_radius: 7.,
        // execution_manager: ContinueUntil(Instant::now() + Duration::from_secs_f64(0.5)),
        execution_manager: SampleNTimes(2000),
        moving_radius: 4.,
        obstacles: &ObstaclesApprox {
            contains_func: Box::new(|p: VecN<2, f64>| rtree.contains_point(p)),
            visible_resolution: 0.025,
            workspace: workspace_straight_lines,
        },
        vertices: PhantomData::<(Bsp<2>, CartesianTopology<2, EuclidianDistance>)>,
        workspace: workspace_straight_lines,
    };

    let (out, graph) = rrt_star(params);

    dbg!(graph.iter().count());
    assert!(!rtree.contains_point(start_pos));
    assert!(!rtree.contains_point(end_pos));

    put_graph(
        &mut svg,
        &graph,
        |p| p,
        0.,
        Style::stroke("#222222", 0.05).with_fill("none"),
    );

    match out {
        None => println!("No path found !"),
        Some((path1, length)) => {
            println!("Path found of length: {length}");
            let path2 = shortcut(&workspace_straight_lines, path1.clone(), &rtree, 5000);
            for (path, color) in [(path1, "blue"), (path2, "green")] {
                for part in path {
                    svg.push(part, 20., Style::stroke(color, 0.2).with_fill("none"))
                }
            }
        }
    }

    svg.write_to_file(&out_dir().join("illustration_tetra_straight2.svg"));

    let mut svg = svg::SvgGroup::default();

    for p in obstacles_vec.iter() {
        svg.push(p.clone(), 0., Style::fill("#555555"));
    }

    let workspace_reeds_shepp = ReedsSheppWorkspace {
        physical_space: space_bb,
        steering_radius: STEERING_RADIUS,
        forward_only: true,
    };

    let obstacles_reeds_shepp = ObstaclesApprox {
        contains_func: Box::new(|p: OrientedCoord| rtree.contains_point(p.0)),
        visible_resolution: 0.02,
        workspace: workspace_reeds_shepp,
    };
    let params = GraphHeuristicParameters {
        start: (start_pos, Angle::from_degrees(0.)),
        end: Goal::Vertex((end_pos, Angle::from_degrees(0.))),
        base_rewire_radius: 4.,
        execution_manager: ContinueUntil(Instant::now() + Duration::from_secs_f64(1000.)),
        moving_radius: 2.,
        obstacles: &obstacles_reeds_shepp,
        vertices: PhantomData::<(Vec<OrientedCoord>, ReedsSheppWorkspace)>,
        workspace: workspace_reeds_shepp,
    };

    let (out, graph) = rrt_star(params);

    dbg!(graph.iter().count());

    // let mut seen = HashSet::new();
    // for start in graph.iter() {
    //     for end in graph.neighbors(start) {
    //         if seen.contains(&(start, end.start)) {
    //             continue;
    //         }
    //         seen.insert((end.start, start));
    //         put_reeds_shepp(
    //             &mut svg,
    //             Style::stroke("gray", 0.02).with_fill("none"),
    //             end,
    //             0.,
    //         );
    //     }
    // }

    match out {
        None => println!("No path found !"),
        Some((path1, length)) => {
            let path2 = shortcut(
                &workspace_reeds_shepp,
                path1.clone(),
                &obstacles_reeds_shepp,
                500,
            );

            println!("Path found of length: {length}");
            for (path, color) in [(path1, "blue"), (path2, "green")] {
                for part in path {
                    put_reeds_shepp(
                        &mut svg,
                        Style::stroke(color, 0.2).with_fill("none"),
                        part,
                        1.,
                    );
                }
            }
        }
    }

    // svg.push(
    //     Image {
    //         url: in_dir
    //             .join(INPUT_IMAGE_FILENAME)
    //             .to_str()
    //             .unwrap()
    //             .to_string(),
    //         positions: obstacles.bounding_box,
    //     },
    //     -10.,
    //     Style::default(),
    // );
    svg.write_to_file(&out_dir().join("illustration_tetra_reeds_shepp2.svg"));
}
