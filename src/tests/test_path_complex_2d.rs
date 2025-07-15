/// Generates a large map and test the algorithms

use crate::{
    geometry::{ polygon_map_generator::gen_pol_map_square},
    graphs::Graph,
    path_planning::visibility_graph::{compute_vis_graph_fullmap, vis_graph_opt1},
    svg::{self, graph::put_graph, object::Style},
    tests::giggle_coords,
};
use rand::{distr::Distribution, rng, Rng};
use crate::tests::out_dir;

pub fn test_square_map() {
    let mut rng = rng();

    println!("Computing the map..");
    let obstacles = gen_pol_map_square(10, 500.0, 250);

    // println!("Computing margins");
    // let mut obstacles2 = obstacles
    //     .iter()
    //     .map(|p| p.add_margin(Angle::from_degrees(45.), 1.0))
    //     .collect::<Vec<_>>();

    println!("Computing margins");
    let mut obstacles2 = obstacles
        .iter()
        .cloned()
        // .map(|p| p.add_rough_margin(0.5))
        .collect::<Vec<_>>();

    println!("Giggling");
    giggle_coords(&mut obstacles2);

    dbg!(obstacles.len());
    dbg!(obstacles2.len());

    let mut svg = svg::SvgGroup::default();

    println!("Computing the visibility graph");
    let vis = compute_vis_graph_fullmap(&obstacles2, vis_graph_opt1);

    println!("Writing the visibility graph");
    put_graph(
        &mut svg,
        &vis,
        |(i, j)| obstacles2[i].0[j],
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

    println!("Writing obstacles with margin");
    for p in &obstacles2 {
        svg.push(p.clone(), -1., Style::fill("#00440077"));
    }

    println!("Finding path");
    let distr =
        rand::distr::weighted::WeightedIndex::new(obstacles2.iter().map(|p| p.0.len().pow(2)))
            .unwrap();
    let start_i = distr.sample(&mut rng);
    let start_j = rng.random_range(0..obstacles2[start_i].0.len());
    let end_i = distr.sample(&mut rng);
    let end_j = rng.random_range(0..obstacles2[end_i].0.len());
    if let Some((path, _)) = vis.a_star_with((start_i, start_j), (end_i, end_j), |(i, j)| {
        obstacles2[i].0[j]
    }) {
        println!("Writing path");
        svg.push(
            path.iter()
                .map(|(i, j)| obstacles2[*i].0[*j])
                .collect::<Vec<_>>(),
            2.,
            Style::stroke("red", 1.).with_fill("none"),
        );
    }

    println!("Saving file");
    svg.write_to_file(&out_dir().join("test_square_map.svg"));
}
