use std::collections::HashMap;
use rand::{rng, Rng};
use crate::geometry::shapes::Polygon;
use crate::geometry::VecN;
use crate::path_planning::polyanya::{polyanya, PolyanyaMode, shortest_path_polyanya};
use crate::svg;
use crate::svg::graph::put_graph;
use crate::svg::object::Style;
use crate::svg::polyanya_interval_map::put_map;
use crate::tests::{giggle_coords, out_dir};
use crate::triangulations::delaunay::make_delaynay;
use crate::triangulations::triangulation_lineaire::triangulate_linear;

pub fn test_polyanya_simple() {
    let mut rng = rng();

    let p1 = Polygon::new(vec![
        // VecN([0., 0.]),
        // VecN([1., -1.]),
        // VecN([2., -0.5]),

        VecN([0., 0.]),
        VecN([1., 1.]),
        VecN([3., -1.5]),
        VecN([5.9, -4.]),
        VecN([-0.75, -1.9]),
        VecN([-0.5, 0.7]),

        // VecN([1., 0.]),
        // VecN([0.8, 0.8]),
        // VecN([0., 1.]),
        // VecN([-0.8, 0.6]),
        // VecN([-1., 0.]),
        // VecN([-0.6, -0.8]),
        // VecN([0., -1.]),
        // VecN([0.6, -0.6]),
    ]);
    let p2 = Polygon::new(vec![
        // VecN([0.9, 2.]),
        // VecN([1.6, 1.3]),
        // VecN([1.05, -0.1]),

        VecN([0.1, 0.3]),
        VecN([1.1, 1.3]),
        VecN([8., 0.]),
        VecN([6.5, 2.]),
        VecN([-2.2, 2.5]),
    ]);
    let p3 = Polygon::new(vec![
        VecN([-1., 0.]),
        VecN([-1.5, -2.]),
        VecN([0.5, -3.]),
        VecN([-4.2, -5.]),
        VecN([-7., -2.]),
    ]);

    let mut svg = svg::SvgGroup::default();
    let mut obstacles = vec![p1, p2, p3];
    giggle_coords(&mut obstacles);

    for (p, col) in obstacles.iter().zip(["#FF4444", "#44FF44", "#4444FF"]) {
        svg.push(p.clone(), 0., Style::fill(col));
    }

    // let mut tri = triangulate_linear(&obstacles, 100.);
    // make_delaynay(&mut tri);
    //
    // put_graph(
    //     &mut svg,
    //     &tri.to_vertex_graph(),
    //     |i| tri.vertex_poss[i],
    //     1.,
    //     Style::stroke("white", 0.04),
    // );
    // put_graph(
    //     &mut svg,
    //     &tri.to_triangle_graph(),
    //     |i| tri.get_tri_center(i),
    //     0.7,
    //     Style::stroke("green", 0.04),
    // );

    let (tri, opt, map) = shortest_path_polyanya(&obstacles, (0, 0), (0, 4), PolyanyaMode::DijkstraExhaustive);
    put_graph(
        &mut svg,
        &tri.to_vertex_graph(),
        |i| tri.vertex_poss[i],
        1.,
        Style::stroke("white", 0.04),
    );
    put_graph(
        &mut svg,
        &tri.to_triangle_graph(),
        |i| tri.get_tri_center(i),
        0.7,
        Style::stroke("green", 0.04),
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

    if let Some((path, length)) = opt {
        println!("A path has been found !");
        svg.push(
            path,
            2.,
            Style::stroke("red", 0.1).with_fill("none"),
        );
    } else {
        println!("No path found !");
    }

    svg.write_to_file(&out_dir().join("test_polyanya_simple2.svg"));
}

// pub fn test_polyanya_complex() {
//
// }
