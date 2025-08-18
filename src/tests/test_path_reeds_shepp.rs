use std::{array::from_fn, collections::HashSet, marker::PhantomData};

use rand::{rng, Rng};

/// A very simple test for svg
use crate::{
    datastructures::r_tree::RTree,
    geometry::{angles::Angle, shapes::Cube, VecN},
    graphs::{Graph, IterableGraph},
    path_planning::graphs_heuristics::{
        prm, rrt, rrt_star, GraphHeuristicParameters, SampleNTimes,
    },
    svg::{self, curves::put_reeds_shepp, object::Style},
    workspace::{
        obstacles::ObstaclesApprox,
        reeds_shepp::{OrientedCoord, ReedsSheppWorkspace},
        workspace::WorkspaceTopology,
    },
};

use super::out_dir;

pub fn test_path_reeds_shepp() {
    let mut svg = svg::SvgGroup::default();

    let mut rng = rng();

    let mut obstacles_array = from_fn::<_, 200, _>(|_| {
        let start = VecN([rng.random_range(0.0..10.0), rng.random_range(0.0..10.0)]);
        Cube {
            start,
            end: start + VecN([rng.random_range(0.1..0.4), rng.random_range(0.1..0.4)]),
        }
    })
    .to_vec();
    obstacles_array.push(Cube {
        start: VecN::splat(-1.),
        end: VecN([0., 11.]),
    });
    obstacles_array.push(Cube {
        start: VecN::splat(-1.),
        end: VecN([11., 0.]),
    });
    obstacles_array.push(Cube {
        start: VecN([0., 10.]),
        end: VecN::splat(11.),
    });
    obstacles_array.push(Cube {
        start: VecN([10., 0.]),
        end: VecN::splat(11.),
    });
    let obstacles_tree = RTree::bulk_load(&mut obstacles_array);

    let workspace = ReedsSheppWorkspace {
        physical_space: Cube {
            start: VecN([-1., -1.]),
            end: VecN([11., 11.]),
        },
        steering_radius: 1.,
    };

    let params = GraphHeuristicParameters {
        start: (VecN([0.1, 0.1]), Angle::from_degrees(0.)),
        end: (VecN([9.9, 9.9]), Angle::from_degrees(90.)),
        base_rewire_radius: 10.,
        execution_manager: SampleNTimes(200),
        moving_radius: 4.,
        obstacles: &ObstaclesApprox {
            contains_func: Box::new(|p: OrientedCoord| obstacles_tree.contains_point(p.0)),
            visible_resolution: 0.1,
            workspace,
        },
        vertices: PhantomData::<(Vec<OrientedCoord>, ReedsSheppWorkspace)>,
        workspace,
    };

    let (out, graph) = prm(params);

    for c in obstacles_array {
        svg.push(c, -1., Style::fill("red"));
    }

    let mut seen = HashSet::new();
    for start in graph.iter() {
        for end in graph.neighbors(start) {
            if seen.contains(&(start, end)) {
                continue;
            }
            seen.insert((end, start));
            let segment = workspace.segment(start, end);
            put_reeds_shepp(
                &mut svg,
                Style::stroke("gray", 0.01).with_fill("none"),
                segment,
                0.,
            );
        }
    }

    match out {
        None => println!("No path found !"),
        Some((path, length)) => {
            println!("Path found of length: {length}");
            for i in 0..(path.len() - 1) {
                let segment = workspace.segment(path[i], path[i + 1]);
                put_reeds_shepp(
                    &mut svg,
                    Style::stroke("blue", 0.03).with_fill("none"),
                    segment,
                    1.,
                );
            }
        }
    }

    svg.write_to_file(&out_dir().join("test_path_reeds_shepp.svg"));
}
