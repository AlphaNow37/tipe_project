use std::collections::HashMap;
// use crate::libs::l_polyanya::shortest_path_polyanya_lib;
use crate::geometry::polygon_map_generator::gen_pol_map_global;
use crate::parallel::{compute_vis_graph_gpu_adjacencymatrix, compute_vis_graph_gpu_edgelist};
use crate::path_planning::polyanya::{find_start_goal_idx, shortest_path_polyanya, PolyanyaMode};
use crate::svg::polyanya_interval_map::put_map;
use crate::tests::out_dir;
use crate::triangulations::delaunay::make_delaynay;
use crate::triangulations::triangulation_line_sweep::triangulate_line_sweep;
use crate::workspace::cartesians::{
    CartesianTopology, DiscreteCartesianTopology, EuclidianDistance,
};
/// Generates a large map and test the algorithms
use crate::{
    geometry::polygon_map_generator::gen_pol_map_square,
    graphs::Graph,
    path_planning::visibility_graph::{compute_vis_graph_fullmap, vis_graph_opt1},
    svg::{self, graph::put_graph, object::Style},
    tests::giggle_coords,
};
use rand::{distr::Distribution, rng, Rng};
use rand::prelude::SliceRandom;

pub fn test_square_map() {
    let workspace = CartesianTopology::new_borderless(EuclidianDistance);

    let mut rng = rng();

    println!("Computing the map..");
    let obstacles = gen_pol_map_square(7, 50.0, 60);

    // let obstacles  = gen_pol_map_global(30, 20.);

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
    svg.set_background("white".to_string());

    println!("Computing the visibility graph");
    let vis = compute_vis_graph_fullmap(&obstacles2, vis_graph_opt1);
    // let vis = compute_vis_graph_gpu_adjacencymatrix(&obstacles2);
    // let vis = compute_vis_graph_gpu_edgelist(&obstacles2, 1000000);

    println!("Writing the visibility graph");
    put_graph(
        &mut svg,
        &vis,
        |(i, j)| obstacles2[i].0[j],
        1.,
        Style::stroke("black", 0.06),
    );

    println!("Writing obstacles");
    for p in &obstacles {
        svg.push(
            p.clone(),
            0.,
            Style::fill(format!(
                "#{:0x}{:0x}{:0x}FF",
                rng.random_range(20..170),
                rng.random_range(20..170),
                rng.random_range(20..170)
            )),
        );
    }

    // println!("Writing obstacles with margin");
    // for p in &obstacles2 {
    //     svg.push(p.clone(), -1., Style::fill("#00440077"));
    // }

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
            Style::stroke("red", 0.75).with_fill("none"),
        );
    }

    println!("Saving file");
    svg.write_to_file(&out_dir().join("test_gvis.svg"));
}

pub fn test_square_map_polyanya() {
    let mut rng = rng();

    println!("Computing the map..");
    let mut obstacles = gen_pol_map_square(10, 500.0, 100);

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
        PolyanyaMode::AStar,
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
        colors
            .entry(src)
            .or_insert_with(|| {
                format!(
                    "#{:0x}{:0x}{:0x}FF",
                    rng.random_range(150..220),
                    rng.random_range(150..220),
                    rng.random_range(150..220)
                )
            })
            .clone()
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

pub fn test_square_map_theta_star() {
    let mut rng = rng();

    println!("Computing the map..");
    let mut obstacles = gen_pol_map_square(20, 500.0, 200);

    println!("Giggling");
    giggle_coords(&mut obstacles);

    dbg!(obstacles.len());

    let mut svg = svg::SvgGroup::default();

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

    println!("Creating the triangulation");
    let mut tri = triangulate_line_sweep(&obstacles, 20.);
    // println!("Making it delaunay");
    // dbg!(make_delaynay(&mut tri));
    tri.build_vertex_to_adj_tris();

    let (new_start, new_end) =
        find_start_goal_idx((start_i, start_j), (end_i, end_j), &obstacles, &tri);

    dbg!(new_start, new_end);

    println!("Writing the triangulation");
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

    println!("Computing the path using theta star");

    let g = tri.to_vertex_graph();
    let opt = g.theta_star_with(
        new_start,
        new_end,
        |i| i,
        &DiscreteCartesianTopology {
            positions: &tri.vertex_poss,
            dist: EuclidianDistance,
        },
        &tri,
    );

    if let Some((path, _)) = opt {
        println!("Writing path");
        svg.push(
            path.iter().map(|i| tri.vertex_poss[*i]).collect::<Vec<_>>(),
            2.,
            Style::stroke("red", 1.).with_fill("none"),
        );
    } else {
        println!("No path found !");
    }

    println!("Saving file");
    svg.write_to_file(&out_dir().join("test_square_map_theta_star.svg"));
}

pub fn test_square_map_duo() {
    let workspace = CartesianTopology::new_borderless(EuclidianDistance);

    let mut rng = rng();

    println!("Computing the map..");
    let obstacles = gen_pol_map_square(4, 1000.0, 15);

    // let obstacles  = gen_pol_map_global(30, 20.);

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

    let mut svgvis = svg::SvgGroup::default();
    let mut svgpol = svg::SvgGroup::default();

    println!("Computing the visibility graph");
    let vis = compute_vis_graph_fullmap(&obstacles2, vis_graph_opt1);
    // let vis = compute_vis_graph_gpu_adjacencymatrix(&obstacles2);
    // let vis = compute_vis_graph_gpu_edgelist(&obstacles2, 1000000);

    println!("Writing the visibility graph");
    put_graph(
        &mut svgvis,
        &vis,
        |(i, j)| obstacles2[i].0[j],
        1.,
        Style::stroke("black", 2.),
    );

    println!("Writing obstacles");
    for p in &obstacles {
        let style = Style::fill(format!(
            "#{:02x}{:02x}{:02x}FF",
            rng.random_range(0..40),
            rng.random_range(0..40),
            rng.random_range(0..40)
        ));
        svgvis.push(p.clone(), 0., style.clone());
        svgpol.push(p.clone(), 0., style);
    }

    println!("Finding path (vis)");
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
        println!("Writing path for vis");
        svgvis.push(
            path.iter()
                .map(|(i, j)| obstacles2[*i].0[*j])
                .collect::<Vec<_>>(),
            2.,
            Style::stroke("red", 10.).with_fill("none"),
        );
    } else {
        println!("No path found for vis")
    }

    println!("Finding a path (polyanya)");
    let (tri, opt, map) = shortest_path_polyanya(
        &obstacles,
        // obstacles[start_i].0[start_j],
        // obstacles[end_i].0[end_j],
        (start_i, start_j),
        (end_i, end_j),
        PolyanyaMode::AStar,
    );
    put_graph(
        &mut svgpol,
        &tri.to_vertex_graph(),
        |i| tri.vertex_poss[i],
        1.,
        Style::stroke("black", 2.),
    );
    // put_graph(
    //     &mut svgpol,
    //     &tri.to_triangle_graph(),
    //     |i| tri.get_tri_center(i),
    //     0.7,
    //     Style::stroke("green", 2.),
    // );

    let mut colors = HashMap::new();
    put_map(&mut svgpol, &map, 0.5, |src| {
        colors
            .entry(src)
            .or_insert_with(|| {
                let mut cols = [
                    rng.random_range(200..=255),
                    rng.random_range(20..=100),
                    rng.random_range(20..=255),
                ];
                cols.shuffle(&mut rng);
                format!(
                    "#{:02X}{:02X}{:02X}FF",
                    cols[0],
                    cols[1],
                    cols[2]
                )
            })
            .clone()
    });

    if let Some((path, _)) = opt {
        println!("Writing path polyanya");
        svgpol.push(path, 2., Style::stroke("red", 10.).with_fill("none"));
    } else {
        println!("No path found for polyanya !")
    }

    println!("Saving files");
    svgvis.write_to_file(&out_dir().join("test_duo_gvis.svg"));
    svgpol.write_to_file(&out_dir().join("test_duo_pol.svg"));
}
