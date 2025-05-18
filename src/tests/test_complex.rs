use std::path::Path;

use crate::{
    geometry::polygon_map_generator::gen_pol_map_square,
    graphs::Graph,
    path_planning::visibility_graph::compute_vis_graph,
    svg::{self, graph::put_graph, object::Style},
    tests::giggle_coords,
};
use rand::{distr::Distribution, rng, Rng};

use super::OUT;

pub fn test_square_map() {
    let mut rng = rng();

    println!("Computing the map..");
    let mut obstacles = gen_pol_map_square(40, 500.0, 2000);

    println!("Giggling");
    giggle_coords(&mut obstacles);

    dbg!(obstacles.len());

    let mut svg = svg::SvgGroup::default();

    println!("Computing the visibility graph");
    let vis = compute_vis_graph(&obstacles);

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
    svg.write_to_file(&(Path::new(OUT)).join("test_square_map.svg"));
}
