/// Used to generate dynamic backgrounds on my computer
///
use std::path::Path;

use crate::{
    geometry::{shapes::Polygon, VecN},
    graphs::Graph,
    path_planning::visibility_graph::{compute_vis_graph_fullmap, vis_graph_opt1},
    svg::{self, graph::put_graph, object::Style},
    workspace::workspace::{EuclidianDistance, UniformTopology},
};

use super::giggle_coords;

pub fn generate_backgrounds() {
    let workspace = UniformTopology::new_borderless(EuclidianDistance);

    let p1 = Polygon::new(vec![
        VecN([0., 0.]),
        VecN([1., 1.]),
        VecN([3., -1.4]),
        VecN([5.9, -4.]),
        VecN([-0.75, -1.9]),
        VecN([-0.5, 0.7]),
    ]);
    let p2 = Polygon::new(vec![
        VecN([0., 0.3]),
        VecN([1., 1.3]),
        VecN([8., 0.]),
        VecN([6.5, 2.]),
        VecN([2., 3.]),
        VecN([-2.2, 2.5]),
    ]);
    let p3 = Polygon::new(vec![
        VecN([-1., 0.]),
        VecN([-1.5, -2.]),
        VecN([0.5, -3.]),
        VecN([-4., -5.1]),
        VecN([-4.5, -4.8]),
        VecN([-7., -2.]),
    ]);

    let mut obstacles = vec![p1, p2, p3];
    giggle_coords(&mut obstacles);

    let vis = compute_vis_graph_fullmap(&obstacles, vis_graph_opt1);

    let mut idx = 0;
    for (i1, i2) in [(0, 1), (1, 2), (2, 1)] {
        for j1 in 0..6 {
            for j2 in 0..6 {
                let mut svg = svg::SvgGroup::default();
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

                if let Some((path, _)) =
                    vis.a_star_with((i1, j1), (i2, j2), |(i, j)| obstacles[i].0[j], &workspace)
                {
                    svg.push(
                        path.iter()
                            .map(|(i, j)| obstacles[*i].0[*j])
                            .collect::<Vec<_>>(),
                        2.,
                        Style::stroke("red", 0.05).with_fill("none"),
                    );
                }

                svg.write_to_file(
                    &(Path::new("/home/alpha_now/Documents/backgrounds"))
                        .join(format!("p{idx}.svg")),
                );
                idx += 1;
            }
        }
    }
}
