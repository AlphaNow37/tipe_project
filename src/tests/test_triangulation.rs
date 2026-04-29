use rand::{rng, Rng};
use crate::geometry::polygon_map_generator::gen_pol_map_square;
use crate::geometry::shapes::Polygon;
use crate::geometry::VecN;
use crate::svg;
use crate::svg::graph::put_graph;
use crate::svg::object::Style;
use crate::tests::{giggle_coords, out_dir};
use crate::triangulations::delaunay::{check_is_delaynay, make_delaynay};
use crate::triangulations::triangulation::{TriAdjacentEdge, Triangulation};
use crate::triangulations::triangulation_lineaire::triangulate_linear;

pub fn test_triangulation_simple() {

    let p1 = Polygon::new(vec![
        VecN([0., 0.]),
        VecN([1., 1.]),
        VecN([3., -1.5]),
        VecN([5.9, -4.]),
        VecN([-0.75, -1.9]),
        VecN([-0.5, 0.7]),
    ]);
    let p2 = Polygon::new(vec![
        VecN([0., 0.3]),
        VecN([1., 1.3]),
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
    // let p4 = Polygon(vec![
    //     VecN([-20., 0.]),
    //     VecN([10., 10.]),
    //     VecN([11., -10.]),
    // ]);

    let mut svg = svg::SvgGroup::default();
    let mut obstacles = vec![p1, p2, p3];
    giggle_coords(&mut obstacles);

    // svg.push(p4.clone(), -1., Style::stroke("red", 0.1));

    for (p, col) in obstacles[..3].iter().zip(["#FF4444", "#44FF44", "#4444FF"]) {
        svg.push(p.clone(), 0., Style::fill(col));
    }

    let t = triangulate_linear(&obstacles, 10.);

    put_graph(
        &mut svg,
        &t.to_vertex_graph(),
        |i| t.vertex_poss[i],
        1.,
        Style::stroke("white", 0.1)
    );

    svg.write_to_file(&out_dir().join("test_triangulatione.svg"));
}


pub fn test_triangulation_hard() {

    let mut rng = rng();

    let polys = gen_pol_map_square(
        100,
        1000.,
        5000,
    );

    let mut svg = svg::SvgGroup::default();

    // svg.push(p4.clone(), -1., Style::stroke("red", 0.1));

    for p in polys.iter() {
        svg.push(p.clone(), 0., Style::fill(format!("#{:0x}", rng.random_range(0..256 * 256 * 256))),);
    }

    let mut t = triangulate_linear(&polys, 10.);

    let c1 = make_delaynay(&mut t);
    debug_assert!(t.verify_invariants() == ());
    // let c2 = make_delaynay(&mut t);
    // debug_assert!(t.verify_invariants() == ());
    // println!("Count 1: {c1}, count 2: {c2}");
    println!("Count: {c1}");
    println!("Nb vertices: {}", t.vertex_poss.len());

    debug_assert!(check_is_delaynay(&t));

    put_graph(
        &mut svg,
        &t.to_vertex_graph(),
        |i| t.vertex_poss[i],
        1.,
        Style::stroke("white", 0.1)
    );

    put_graph(
        &mut svg,
        &t.to_triangle_graph(),
        |i| t.get_tri_center(i),
        0.5,
        Style::stroke("red", 0.1),
    );

    svg.write_to_file(&out_dir().join("test_triangulatione_hard_delaunay.svg"));
}


pub fn test_tri_manual() {
    let mut t = Triangulation::new(vec![
        VecN([-5., 2.]),
        VecN([1.5, 5.]),
        VecN([9., 3.]),
        VecN([1., 0.]),
    ]);
    t.triangles = vec![
        [
            TriAdjacentEdge {
                verts: [1, 0],
                other_tri: None,
            },
            TriAdjacentEdge {
                verts: [0, 2],
                other_tri: Some(1),
            },
            TriAdjacentEdge {
                verts: [2, 1],
                other_tri: None,
            },
        ],
        [
            TriAdjacentEdge {
                verts: [0, 3],
                other_tri: None,
            },
            TriAdjacentEdge {
                verts: [3, 2],
                other_tri: None,
            },
            TriAdjacentEdge {
                verts: [2, 0],
                other_tri: Some(0),
            },
        ]
    ];

    let mut svg = svg::SvgGroup::default();

    dbg!(make_delaynay(&mut t));

    put_graph(
        &mut svg,
        &t.to_vertex_graph(),
        |i| t.vertex_poss[i],
        1.,
        Style::stroke("white", 0.1)
    );

    svg.write_to_file(&out_dir().join("test_tri_manual.svg"));
}
