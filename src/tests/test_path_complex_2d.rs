use std::collections::HashMap;
// use crate::libs::l_polyanya::shortest_path_polyanya_lib;
use crate::parallel::{compute_vis_graph_gpu_adjacencymatrix, compute_vis_graph_gpu_edgelist};
use crate::path_planning::polyanya::shortest_path_polyanya;
use crate::svg::polyanya_interval_map::put_map;
use crate::tests::out_dir;
use crate::workspace::cartesians::{CartesianTopology, EuclidianDistance};
/// Generates a large map and test the algorithms
use crate::{
    geometry::polygon_map_generator::gen_pol_map_square,
    graphs::Graph,
    path_planning::visibility_graph::{compute_vis_graph_fullmap, vis_graph_opt1},
    svg::{self, graph::put_graph, object::Style},
    tests::giggle_coords,
};
use rand::{distr::Distribution, rng, Rng};

pub fn test_square_map() {
    let workspace = CartesianTopology::new_borderless(EuclidianDistance);

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
    // let vis = compute_vis_graph_fullmap(&obstacles2, vis_graph_opt1);
    // let vis = compute_vis_graph_gpu_adjacencymatrix(&obstacles2);
    let vis = compute_vis_graph_gpu_edgelist(&obstacles2, 1000000);

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
    if let Some((path, _)) = vis.a_star_with(
        (start_i, start_j),
        (end_i, end_j),
        |(i, j)| obstacles2[i].0[j],
        &workspace,
    ) {
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

pub fn test_square_map_polyanya() {
    let mut rng = rng();

    println!("Computing the map..");
    let mut obstacles = gen_pol_map_square(10, 500.0, 120);

    // println!("Computing margins");
    // let mut obstacles2 = obstacles
    //     .iter()
    //     .map(|p| p.add_margin(Angle::from_degrees(45.), 1.0))
    //     .collect::<Vec<_>>();

    println!("Giggling");
    giggle_coords(&mut obstacles);

    dbg!(obstacles.len());

    let mut svg = svg::SvgGroup::default();

    println!("Finding path");
    let distr =
        rand::distr::weighted::WeightedIndex::new(obstacles.iter().map(|p| p.0.len().pow(2)))
            .unwrap();
    let start_i = distr.sample(&mut rng);
    let start_j = rng.random_range(0..obstacles[start_i].0.len());
    let end_i = distr.sample(&mut rng);
    let end_j = rng.random_range(0..obstacles[end_i].0.len());

    let (tri, opt, map) = shortest_path_polyanya(
        &obstacles,
        // obstacles[start_i].0[start_j],
        // obstacles[end_i].0[end_j],
        (start_i, start_j),
        (end_i, end_j),
    );
    put_graph(
        &mut svg,
        &tri.to_vertex_graph(),
        |i| tri.vertex_poss[i],
        1.,
        Style::stroke("white", 0.15),
    );
    put_graph(
        &mut svg,
        &tri.to_triangle_graph(),
        |i| tri.get_tri_center(i),
        0.7,
        Style::stroke("green", 0.05),
    );

    let mut colors = HashMap::new();
    put_map(&mut svg, &map, 0.5, |src| {
        colors.entry(src).or_insert_with(|| {
            format!(
                "#{:0x}{:0x}{:0x}FF",
                rng.random_range(150..220),
                rng.random_range(150..220),
                rng.random_range(150..220)
            )
        }).clone()
    });

    println!("Writing obstacles");
    for p in &obstacles {
        svg.push(
            p.clone(),
            0.,
            Style::fill(format!(
                "#{:02x}{:02x}{:02x}CC",
                rng.random_range(50..100),
                rng.random_range(50..100),
                rng.random_range(50..100)
            )),
        );
    }

    if let Some((path, _)) = opt {
        println!("Writing path");
        svg.push(path, 2., Style::stroke("red", 1.).with_fill("none"));
    } else {
        println!("No path found !")
    }

    println!("Saving file");
    svg.write_to_file(&out_dir().join("test_square_map_polyanya.svg"));
}
