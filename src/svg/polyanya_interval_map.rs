use std::collections::HashMap;
use std::hash::Hash;
use crate::geometry::shapes::Polygon;
use crate::path_planning::polyanya::Intervals;
use crate::svg::object::Style;
use crate::svg::SvgGroup;

/// Ajoute une carte polyanya sur le svg
pub fn put_map(
    svg: &mut SvgGroup,
    map: &HashMap<[usize; 2], Intervals>,
    height: f64,
    mut color: impl FnMut(usize)->String
) {
    for (_, intervals) in map.iter() {
        for int in &intervals.intervals {
            let col = color(int.source);
            let p1 = intervals.segment.to_line().point_at_time(int.times[0]);
            let p2 = intervals.segment.to_line().point_at_time(int.times[1]);
            svg.push(
                Polygon::new(vec![p1, p2, int.source_pos]),
                height,
                Style::fill(col)
            )
        }
    }
}
