use super::{giggle_coords, out_dir};
use crate::geometry::shapes::Cube;
use crate::svg::object::Text;
/// hardcoded, simple tests for debugging purposes
use crate::{
    geometry::{shapes::Polygon, VecN},
    graphs::Graph,
    path_planning::visibility_graph::{compute_vis_graph_fullmap, vis_graph_opt1},
    svg::{self, graph::put_graph, object::Style},
    workspace::cartesians::{CartesianTopology, EuclidianDistance},
};
use crate::graphs::IterableGraph;
use crate::path_planning::visibility_graph::vis_graph_naive;

pub fn test_pretty_simple() {
    let workspace = CartesianTopology::new_borderless(EuclidianDistance);

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

    if let Some((path, _)) = vis.a_star_with((1, 2), (2, 3), |(i, j)| obstacles[i].0[j], &workspace)
    {
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

pub fn illustration_presentation() {
    let workspace = CartesianTopology::new_borderless(EuclidianDistance);

    let p1 = Polygon::new(vec![
        VecN([0., 0.]),
        VecN([0., 2.]),
        VecN([2., 2.]),
        VecN([2., 0.]),
        VecN([1., 1.]),
    ]);
    let p2 = Polygon::new(vec![
        VecN([-0.25, -0.5]),
        VecN([0.75, 0.5]),
        VecN([1.75, -0.5]),
        VecN([0.75, -1.5]),
    ]);
    let p3 = Polygon::new(vec![
        // VecN([2.25, -0.75]),
        // VecN([2.25, 0.75]),
        // VecN([3.75, 0.]),
    ]);
    let start = VecN([-0.75, 0.25]);
    let end = VecN([2.5, -0.75]);

    let pstart = Polygon::new(vec![start]);
    let pend = Polygon::new(vec![end]);

    let mut svg = svg::SvgGroup::default();
    svg.set_background("#FFFFFF".to_string());

    let mut obstacles = vec![p1, p2, p3, pstart, pend];
    giggle_coords(&mut obstacles);

    for p in obstacles.iter() {
        svg.push(p.clone(), 0., Style::fill("#555555"));
    }

    for pos in [start, end] {
        svg.push(
            Cube::from_point(pos + VecN([-0.02, -0.02])).with_point(pos + VecN([0.02, 0.02])),
            20.,
            Style::fill("black"),
        );
    }

    svg.push(
        Text {
            content: "start".into(),
            position: start + VecN([-0.4, 0.]),
            font_size: 0.2,
        },
        20.,
        Style::fill("black"),
    );
    svg.push(
        Text {
            content: "goal".into(),
            position: end + VecN([0.05, 0.]),
            font_size: 0.2,
        },
        20.,
        Style::fill("black"),
    );
    // test_path_simple_2d::test_pretty_simple();

    let vis = compute_vis_graph_fullmap(&obstacles, vis_graph_naive);

    if let Some((path, _)) = vis.a_star_with((3, 0), (4, 0), |(i, j)| obstacles[i].0[j], &workspace)
    {
        svg.push(
            path.iter()
                .map(|(i, j)| obstacles[*i].0[*j])
                .collect::<Vec<_>>(),
            2.,
            Style::stroke("red", 0.05).with_fill("none"),
        );
    } else {
        println!("No path found !");
    }

    svg.write_to_file(&out_dir().join("illustration_presentation.svg"));

    put_graph(
        &mut svg,
        &vis,
        |(i, j)| obstacles[i].0[j],
        1.,
        Style::stroke("#000000", 0.025),
    );

    svg.write_to_file(&out_dir().join("illustration_visibility_graph.svg"));
}
