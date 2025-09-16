use crate::geometry::angles::Angle;
use crate::geometry::shapes::Polygon;
use crate::geometry::VecN;
use crate::path_planning::visibility_graph::{compute_vis_graph_fullmap, vis_graph_naive};
use crate::svg;
use crate::svg::graph::put_graph;
use crate::svg::object::Style;
use crate::tests::{giggle_coords, out_dir};

pub fn illustration_lower_bound_visibility() {

    let mut pts: Vec<VecN<2, f64>> = (Angle::from_degrees(15.).iter_to(Angle::from_degrees(346.), Angle::from_degrees(30.)))
        .map(|a| a.to_vec())
        .collect();
    dbg!(&pts);
    pts.reverse();
    pts.push(VecN([1.5, 0.1]));
    pts.push(VecN([1.5, 1.5]));
    pts.push(VecN([-1.5, 1.5]));
    pts.push(VecN([-1.5, -1.5]));
    pts.push(VecN([1.5, -1.5]));
    pts.push(VecN([1.5, -0.1]));

    let mut svg = svg::SvgGroup::default();
    svg.set_background("#FFFFFF".to_string());

    let mut obstacles = vec![Polygon::new(pts)];
    giggle_coords(&mut obstacles);

    for p in obstacles.iter() {
        svg.push(p.clone(), 0., Style::fill("#555555"));
    }

    let vis = compute_vis_graph_fullmap(&obstacles, vis_graph_naive);
    put_graph(
        &mut svg,
        &vis,
        |(i, j)| obstacles[i].0[j],
        1.,
        Style::stroke("#000000", 0.025),
    );

    svg.write_to_file(&out_dir().join("illustration_lower_bound_visibility.svg"));
}
