use std::path::Path;

use geometry::{
    polygon_map_generator::{gen_pol_map_luck, gen_pol_map_square},
    shapes::Polygon,
    VecN,
};
use graphs::{Graph, LinkGraph};
use path_planning::visibility_graph::compute_vis_graph_naive;
use rand::{distr::Distribution, rng, Rng};
use svg::{graph::put_graph, object::Style};

pub mod datastructures;
pub mod geometry;
pub mod graphs;
pub mod path_planning;
pub mod svg;
pub mod utils;

const OUT: &str = "/home/alpha_now/Desktop/progs/tipe_project/out";

fn test_1() {
    let mut g = LinkGraph::default();
    for (start, end) in [
        (0, 1),
        (0, 2),
        (2, 4),
        (3, 7),
        (4, 5),
        (4, 0),
        (4, 1),
        (4, 2),
        (5, 3),
        (7, 0),
        (7, 5),
    ] {
        g.add_link(start, end);
    }

    let poss = [
        VecN([0., 0.]),
        VecN([1., 3.]),
        VecN([4., 0.]),
        VecN([6., 2.]),
        VecN([5., 5.]),
        VecN([9., 5.]),
        VecN([1., 1.]),
        VecN([7., -3.]),
    ];

    let mut svg = svg::SvgGroup::default();
    put_graph(&mut svg, &g, |i| poss[i], 0., Style::stroke("red", 0.1));
    svg.write_to_file(&(Path::new(OUT)).join("test.svg"));
}

fn test_2() {
    let p1 = Polygon(vec![
        VecN([0., 0.]),
        VecN([1., 1.]),
        VecN([3., -1.5]),
        VecN([4., -2.]),
        VecN([-0.75, -1.9]),
        VecN([-0.5, 0.7]),
    ]);
    let p2 = Polygon(vec![
        VecN([0., 0.3]),
        VecN([1., 2.]),
        VecN([4., 0.]),
        VecN([3.5, 2.]),
        VecN([0.2, 2.5]),
    ]);
    let p3 = Polygon(vec![
        VecN([-1., 0.]),
        VecN([-1.5, -2.]),
        VecN([0.5, -3.]),
        VecN([0.7, -5.]),
        VecN([-4., -2.]),
    ]);

    let mut svg = svg::SvgGroup::default();
    let obstacles = vec![p1.clone(), p2.clone(), p3.clone()];

    let vis = compute_vis_graph_naive(&obstacles);
    put_graph(
        &mut svg,
        &vis,
        |(i, j)| obstacles[i].0[j],
        1.,
        Style::stroke("white", 0.01),
    );
    svg.push(p1, 0., Style::fill("#550000"));
    svg.push(p2, 0., Style::fill("#005500"));
    svg.push(p3, 0., Style::fill("#000055"));

    if let Some((path, _)) = vis.a_star_with((1, 1), (2, 3), |(i, j)| obstacles[i].0[j]) {
        svg.push(
            path.iter()
                .map(|(i, j)| obstacles[*i].0[*j])
                .collect::<Vec<_>>(),
            2.,
            Style::stroke("red", 0.05).with_fill("none"),
        );
    }

    svg.write_to_file(&(Path::new(OUT)).join("test2.svg"));
}

fn test_3() {
    let mut rng = rng();

    println!("Computing the map..");
    let obstacles = gen_pol_map_square(12, 100.0, 100);
    dbg!(obstacles.len());

    let mut svg = svg::SvgGroup::default();

    println!("Computing the visibility graph");
    let vis = compute_vis_graph_naive(&obstacles);

    println!("Writing the visibility graph");
    put_graph(
        &mut svg,
        &vis,
        |(i, j)| obstacles[i].0[j],
        1.,
        Style::stroke("white", 0.01),
    );

    println!("Writing obstacles");
    for p in &obstacles {
        svg.push(
            p.clone(),
            0.,
            Style::fill(format!("#{:0x}", rng.random_range(0..256 * 256 * 256))),
        );
    }

    println!("Finding path");
    let distr =
        rand::distr::weighted::WeightedIndex::new(obstacles.iter().map(|p| p.0.len().pow(2)))
            .unwrap();
    let start_i = distr.sample(&mut rng);
    let start_j = rng.random_range(0..obstacles[start_i].0.len());
    let end_i = distr.sample(&mut rng);
    let end_j = rng.random_range(0..obstacles[end_i].0.len());
    if let Some((path, _)) = vis.a_star_with((start_i, start_j), (end_i, end_j), |(i, j)| {
        obstacles[i].0[j]
    }) {
        println!("Writing path");
        svg.push(
            path.iter()
                .map(|(i, j)| obstacles[*i].0[*j])
                .collect::<Vec<_>>(),
            2.,
            Style::stroke("red", 1.).with_fill("none"),
        );
    }

    println!("Saving file");
    svg.write_to_file(&(Path::new(OUT)).join("test3.svg"));
}

fn main() {
    // test_2();
    test_3();
}
