use crate::{
    geometry::{shapes::Polygon, VecN},
    graphs::Graph,
<<<<<<< HEAD
    path_planning::visibility_graph::compute_vis_graph,
=======
    path_planning::visibility_graph::{compute_vis_graph_fullmap, vis_graph_opt1},
>>>>>>> aed8dcc (Changed the interface of visibility graphs for more modularity. Added a cached function graph)
    svg::{self, graph::put_graph, object::Style},
};

use super::{giggle_coords, out_dir};

pub fn test_pretty_simple() {
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

    let mut svg = svg::SvgGroup::default();
    let mut obstacles = vec![p1, p2, p3];
    giggle_coords(&mut obstacles);

    let vis = compute_vis_graph_fullmap(&obstacles, vis_graph_opt1);
    put_graph(
        &mut svg,
        &vis,
        |(i, j)| obstacles[i].0[j],
        1.,
        Style::stroke("white", 0.01),
    );

    for (p, col) in obstacles.iter().zip(["#FF4444", "#44FF44", "#4444FF"]) {
        svg.push(p.clone(), 0., Style::fill(col));
    }

    if let Some((path, _)) = vis.a_star_with((1, 2), (2, 3), |(i, j)| obstacles[i].0[j]) {
        svg.push(
            path.iter()
                .map(|(i, j)| obstacles[*i].0[*j])
                .collect::<Vec<_>>(),
            2.,
            Style::stroke("red", 0.05).with_fill("none"),
        );
    }

    svg.write_to_file(&out_dir().join("test_p_simple.svg"));
}

pub fn test_very_simple() {
    let p1 = Polygon::new(vec![VecN([0., 0.]), VecN([1., 0.2]), VecN([0.5, 1.])]);
    let p2 = Polygon::new(vec![VecN([2., 3.]), VecN([3.1, 1.1]), VecN([2.5, 5.9])]);

    let mut svg = svg::SvgGroup::default();
    let obstacles = vec![p1.clone(), p2.clone()];

    let vis = compute_vis_graph_fullmap(&obstacles, vis_graph_opt1);
    put_graph(
        &mut svg,
        &vis,
        |(i, j)| obstacles[i].0[j],
        1.,
        Style::stroke("white", 0.01),
    );
    svg.push(p1, 0., Style::fill("#550000"));
    svg.push(p2, 0., Style::fill("#005500"));

    svg.write_to_file(&out_dir().join("test_v_simple.svg"));
}
