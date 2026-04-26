use crate::geometry::shapes::Polygon;
use crate::geometry::VecN;
use crate::svg;
use crate::svg::graph::put_graph;
use crate::svg::object::Style;
use crate::tests::{giggle_coords, out_dir};
use crate::triangulations::triangulation_lineaire::triangulate_linear;

pub fn test_triangulation() {

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
    let p4 = Polygon(vec![
        VecN([-20., 0.]),
        VecN([10., 10.]),
        VecN([11., -10.]),
    ]);

    let mut svg = svg::SvgGroup::default();
    let mut obstacles = vec![p1, p2, p3, p4.clone()];
    giggle_coords(&mut obstacles);

    svg.push(p4.clone(), -1., Style::stroke("red", 0.1));

    for (p, col) in obstacles[..3].iter().zip(["#FF4444", "#44FF44", "#4444FF"]) {
        svg.push(p.clone(), 0., Style::fill(col));
    }

    let t = triangulate_linear(&obstacles);

    put_graph(
        &mut svg,
        &t.to_vertex_graph(),
        |i| t.vertex_poss[i],
        1.,
        Style::stroke("white", 0.1)
    );

    svg.write_to_file(&out_dir().join("test_triangulatione.svg"));
}
