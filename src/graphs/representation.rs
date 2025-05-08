use std::{collections::HashSet, hash::Hash};

use crate::{
    geometry::{shapes::Segment, VecN},
    svg::{object::Style, SvgGroup},
};

use super::IterableGraph;

pub fn put_graph<V: Hash + Eq + Copy, G: IterableGraph<V>>(
    svg: &mut SvgGroup,
    graph: &G,
    pos: impl Fn(V) -> VecN<2, f64>,
    height: f64,
    style: Style,
) {
    let mut placed = HashSet::new();
    for v in graph.iter() {
        let pos_v = pos(v);
        for n in graph.neighbors(v) {
            if placed.contains(&(n, v)) {
                continue;
            }
            let pos_n = pos(n);
            placed.insert((v, n));
            svg.push(
                Segment {
                    start: pos_v,
                    end: pos_n,
                },
                height,
                style.clone(),
            );
        }
    }
}
