use std::cell::Cell;
use std::cmp::Ordering;
use std::collections::BTreeSet;

use crate::geometry::angles::Angle;
use crate::geometry::shapes::{Ray, Segment};
use crate::graphs::MapGraph;

use crate::geometry::{shapes::Polygon, VecN};
use crate::utils::numbers::UsizeExt;

#[derive(Clone, Copy, Debug)]
struct PolyVertex {
    pos: VecN<2, f64>,
    // L'interieur du polygone est entre les deux voisins
    nexts: [VecN<2, f64>; 2],
    coords: (usize, usize),
}

struct SweepingTreeSegment<'a> {
    segment: Segment<2>,
    ray: &'a Cell<Ray<2>>,
}
impl<'a> Ord for SweepingTreeSegment<'a> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        if self.segment == other.segment {
            return Ordering::Equal;
        }
        let t1 = self
            .segment
            .to_line()
            .intersection_time(self.ray.get().to_line())
            .expect("Invalid segment in the tree")
            .0;
        let t2 = other
            .segment
            .to_line()
            .intersection_time(self.ray.get().to_line())
            .expect("Invalid segment in the tree")
            .0;
        t1.total_cmp(&t2)
    }
}
impl<'a> PartialOrd for SweepingTreeSegment<'a> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl<'a> PartialEq for SweepingTreeSegment<'a> {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other).is_eq()
    }
}
impl<'a> Eq for SweepingTreeSegment<'a> {}

fn to_vertice_vec(obstacles: &[Polygon]) -> Vec<PolyVertex> {
    obstacles
        .iter()
        .enumerate()
        .flat_map(|(poly_i, poly)| {
            let (off1, off2) = if poly.is_counter_clockwise() {
                (1, -1)
            } else {
                (-1, 1)
            };

            let n = poly.0.len();
            (0..poly.0.len()).map(move |i| PolyVertex {
                pos: poly.0[i],
                nexts: [poly.0[i.add_rem(off1, n)], poly.0[i.add_rem(off2, n)]],
                coords: (poly_i, i),
            })
        })
        .collect()
}
fn coords_iterator<'a>(obstacles: &'a [Polygon]) -> impl Iterator<Item = (usize, usize)> + 'a {
    obstacles
        .iter()
        .enumerate()
        .flat_map(|(i, poly)| (0..poly.0.len()).map(move |j| (i, j)))
}

pub fn compute_vis_graph_naive(obstacles: &[Polygon]) -> MapGraph<(usize, usize)> {
    let mut verteces = to_vertice_vec(obstacles);

    let visibles_from_naives = move |coords: (usize, usize)| {
        let vertex_i = verteces
            .iter()
            .position(|v| v.coords == coords)
            .expect("Invalid coords");
        let vertex = verteces.remove(vertex_i);

        let invisible_part = if obstacles[coords.0].0.len() == 0 {
            None
        } else {
            Some((
                Angle::from_point(vertex.nexts[0] - vertex.pos),
                Angle::from_point(vertex.nexts[1] - vertex.pos),
            ))
        };

        let mut visibles = Vec::new();
        'a: for &v in &verteces {
            if let Some((a1, a2)) = invisible_part {
                if Angle::from_point(v.pos - vertex.pos).is_between(a1, a2)
                    && !vertex.nexts.contains(&v.pos)
                {
                    continue 'a;
                }
            }
            let segment = Segment {
                start: vertex.pos,
                end: v.pos,
            };
            for &v2 in &verteces {
                if v2.coords == v.coords {
                    continue;
                }
                for next in v2.nexts {
                    if next != vertex.pos
                        && next != v.pos
                        && segment.intersect_segment(Segment {
                            start: v2.pos,
                            end: next,
                        })
                    {
                        continue 'a;
                    }
                }
            }
            visibles.push(v.coords);
        }

        verteces.push(vertex);
        visibles
    };
    MapGraph::from_fn(coords_iterator(obstacles), visibles_from_naives)
}

pub fn compute_vis_graph(obstacles: &[Polygon]) -> MapGraph<(usize, usize)> {
    let mut verteces = to_vertice_vec(obstacles);
    let n = verteces.len();
    let visibles_from = move |coords: (usize, usize)| {
        let pos = obstacles[coords.0].0[coords.1];
        verteces.sort_by_key(|v| Angle::from_point(v.pos - pos));

        let vertex_i = verteces
            .iter()
            .position(|v| v.coords == coords)
            .expect("Invalid coords");
        let vertex = verteces.remove(vertex_i);

        let mut visibles = Vec::new();
        let mut ray = Cell::new(Ray {
            start: vertex.pos,
            end: vertex.pos + VecN([1., 0.]),
        });

        let mut tree = BTreeSet::new();
        for &v in &verteces {
            let s = Segment {
                start: v.pos,
                end: v.nexts[0],
            };
            if ray.get().intersect_segment(s) {
                tree.insert(SweepingTreeSegment {
                    ray: &ray,
                    segment: s,
                });
            }
        }

        for v in &verteces {
            // tree.remove(&SweepingTreeSegment { segment: todo!(), ray: () });
        }

        verteces.push(vertex);
        visibles
    };
    MapGraph::from_fn(coords_iterator(obstacles), visibles_from)
}
