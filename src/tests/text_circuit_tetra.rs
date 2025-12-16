use super::{in_dir, out_dir};
use crate::datastructures::bsp::Bsp;
use crate::path_planning::graphs_heuristics::ContinueUntil;
use crate::svg::graph::put_graph;
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
use std::time::{Duration, Instant};
use std::{collections::HashSet, marker::PhantomData};

const INPUT_IMAGE_FILENAME: &'static str = "circuit_tetra_1.png";
const INPUT_GRID_FILENAME: &'static str = "circuit_tetra_1_arr.txt";
const STEERING_RADIUS: f64 = 50.;

pub fn illustration_circuit_tetra() {
    let mut svg = svg::SvgGroup::default();
    let in_dir = in_dir();
    let (obstacles, start_pos, end_pos) = read_image(&in_dir.clone().join(INPUT_GRID_FILENAME));

    dbg!(obstacles.accessible.iter().filter(|a| !**a).count());

    let workspace_straight_lines = CartesianTopology {
        dist: EuclidianDistance,
        space: obstacles.bounding_box,
    };

    let params = GraphHeuristicParameters {
        start: start_pos,
        end: end_pos,
        base_rewire_radius: 400.,
        execution_manager: ContinueUntil(Instant::now() + Duration::from_secs_f64(0.5)),
        moving_radius: 200.,
        obstacles: &ObstaclesApprox {
            contains_func: Box::new(|p: VecN<2, f64>| obstacles.contains_point(p)),
            visible_resolution: 0.5,
            workspace: workspace_straight_lines,
        },
        vertices: PhantomData::<(Bsp<2>, CartesianTopology<2, EuclidianDistance>)>,
        workspace: workspace_straight_lines,
    };

    let (out, graph) = rrt_star(params);

    dbg!(graph.iter().count());
    assert!(!obstacles.contains_point(start_pos));
    assert!(!obstacles.contains_point(end_pos));

    put_graph(
        &mut svg,
        &graph,
        |p| p,
        0.,
        Style::stroke("gray", 1.).with_fill("none"),
    );

    match out {
        None => println!("No path found !"),
        Some((path, length)) => {
            println!("Path found of length: {length}");
            for part in path {
                svg.push(
                    part,
                    20.,
                    Style::stroke("blue", 5.).with_fill("none")
                )
            }
        }
    }

    svg.write_to_file(&out_dir().join("illustration_tetra_straight.svg"));

    let mut svg = svg::SvgGroup::default();

    let workspace_reeds_shepp = ReedsSheppWorkspace {
        physical_space: obstacles.bounding_box,
        steering_radius: STEERING_RADIUS,
        forward_only: true,
    };

    let params = GraphHeuristicParameters {
        start: (start_pos, Angle::from_degrees(-90.)),
        end: (end_pos, Angle::from_degrees(-90.)),
        base_rewire_radius: 300.,
        execution_manager: ContinueUntil(Instant::now() + Duration::from_secs_f64(20.)),
        moving_radius: 100.,
        obstacles: &ObstaclesApprox {
            contains_func: Box::new(|p: OrientedCoord| obstacles.contains_point(p.0)),
            visible_resolution: 0.1,
            workspace: workspace_reeds_shepp,
        },
        vertices: PhantomData::<(Vec<OrientedCoord>, ReedsSheppWorkspace)>,
        workspace: workspace_reeds_shepp,
    };

    let (out, graph) = rrt_star(params);

    dbg!(graph.iter().count());

    let mut seen = HashSet::new();
    for start in graph.iter() {
        for end in graph.neighbors(start) {
            if seen.contains(&(start, end.start)) {
                continue;
            }
            seen.insert((end.start, start));
            put_reeds_shepp(
                &mut svg,
                Style::stroke("gray", 1.).with_fill("none"),
                end,
                0.,
            );
        }
    }

    match out {
        None => println!("No path found !"),
        Some((path, length)) => {
            println!("Path found of length: {length}");
            for part in path {
                put_reeds_shepp(
                    &mut svg,
                    Style::stroke("blue", 10.).with_fill("none"),
                    part,
                    1.,
                );
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
    svg.write_to_file(&out_dir().join("illustration_tetra_reeds_shepp.svg"));
}
