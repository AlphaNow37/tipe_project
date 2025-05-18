use std::path::Path;

use crate::{
    geometry::VecN,
    graphs::LinkGraph,
    svg::{self, graph::put_graph, object::Style},
};

use super::OUT;

pub fn test_graph() {
    let mut g = LinkGraph::default();
    for (start, end) in [
        (0, 1),
        (0, 2),
        (2, 4),
        (3, 7),
        (4, 5),
        (4, 0),
        (4, 1),
        (4, 2),
        (5, 3),
        (7, 0),
        (7, 5),
    ] {
        g.add_link(start, end);
    }

    let poss = [
        VecN([0., 0.]),
        VecN([1., 3.]),
        VecN([4., 0.]),
        VecN([6., 2.]),
        VecN([5., 5.]),
        VecN([9., 5.]),
        VecN([1., 1.]),
        VecN([7., -3.]),
    ];

    let mut svg = svg::SvgGroup::default();
    put_graph(&mut svg, &g, |i| poss[i], 0., Style::stroke("red", 0.1));
    svg.write_to_file(&(Path::new(OUT)).join("test_graph.svg"));
}
