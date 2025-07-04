/// This one file uses visibility graphs
use std::cell::Cell;
use std::cmp::Ordering;
use std::collections::BTreeSet;

use crate::geometry::angles::Angle;
use crate::geometry::shapes::{Ray, Segment};
use crate::graphs::{CachedFuncGraph, MapGraph};

use crate::geometry::{shapes::Polygon, VecN};
use crate::utils::numbers::UsizeExt;

/// An intermediate representation of a vertex
#[derive(Clone, Copy, Debug)]
pub struct PolyVertex {
    // Position on the place
    pos: VecN<2, f64>,
    // Neighbors
    // L'interieur du polygone est entre les deux voisins
    nexts: [VecN<2, f64>; 2],
    // Index of the polygon/Index inside the polygon
    coords: (usize, usize),
}

/// A segment inside the balanced tree used by the sweeping line algorithm
#[derive(Clone, Copy)]
struct SweepingTreeSegment<'a> {
    segment: Segment<2>,
    ray: &'a Cell<Ray<2>>,
}
// Compute which one of self and other is closer to the origin of the ray
impl<'a> Ord for SweepingTreeSegment<'a> {
    fn cmp(&self, other: &Self) -> Ordering {
        let mut other_seg = other.segment;
        let mut self_seg = self.segment;
        // Normalise the inputs
        if self_seg.start == other_seg.end {
            other_seg = other_seg.reverse();
        } else if other_seg.start == self_seg.end {
            self_seg = self_seg.reverse();
        }
        if self_seg == other_seg {
            return Ordering::Equal;
        }
        let ray = self.ray.get();
        // Special case: the ray and the two segments intersects at a single point
        if self_seg.start == other_seg.start && other_seg.start == ray.end {
            // We can suppose they are both on the same side
            if ray.is_on_left_side(self_seg.end) {
                debug_assert!(
                    ray.is_on_left_side(other_seg.end),
                    "ray: {ray:?}, self_seg: {self_seg:?}, other_seg: {other_seg:?}"
                );
                if self_seg.to_ray().is_on_left_side(other_seg.end) {
                    Ordering::Greater
                } else {
                    Ordering::Less
                }
            } else {
                debug_assert!(
                    !ray.is_on_left_side(other_seg.end),
                    "ray: {ray:?}, self_seg: {self_seg:?}, other_seg: {other_seg:?}"
                );
                if self_seg.to_ray().is_on_left_side(other_seg.end) {
                    Ordering::Less
                } else {
                    Ordering::Greater
                }
            }
        } else {
            // Find the closest one
            let t1 = self_seg
                .to_line()
                .intersection_time(ray.to_line())
                .ok_or_else(|| format!("Segment: {self_seg:?}, ray: {ray:?}"))
                .expect("Invalid segment in the tree")
                .1;
            let t2 = other_seg
                .to_line()
                .intersection_time(ray.to_line())
                .ok_or_else(|| format!("Segment: {other_seg:?}, ray: {ray:?}"))
                .expect("Invalid segment in the tree")
                .1;
            debug_assert_ne!(t1, t2, "Logical error");
            t1.total_cmp(&t2)
        }
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

/// Computes the intermediate representation
fn to_vertice_vec(obstacles: &[Polygon]) -> Vec<PolyVertex> {
    obstacles
        .iter()
        .enumerate()
        .flat_map(|(poly_i, poly)| {
            let n = poly.len();
            (0..n).map(move |i| PolyVertex {
                pos: poly.points()[i],
                nexts: [
                    poly.points()[i.add_rem(1, n)],
                    poly.points()[i.add_rem(-1, n)],
                ],
                coords: (poly_i, i),
            })
        })
        .collect()
}

/// Returns an iterator of all coords
fn coords_iterator<'a>(obstacles: &'a [Polygon]) -> impl Iterator<Item = (usize, usize)> + 'a {
    obstacles
        .iter()
        .enumerate()
        .flat_map(|(i, poly)| (0..poly.len()).map(move |j| (i, j)))
}

pub fn vis_graph_naive(
    verteces: &mut Vec<PolyVertex>,
    coords: (usize, usize),
    _obstacles: &[Polygon],
) -> Vec<(usize, usize)> {
    let vertex_i = verteces
        .iter()
        .position(|v| v.coords == coords)
        .expect("Invalid coords");
    let vertex = verteces.swap_remove(vertex_i);

    let invisible_part = if vertex.nexts[0] == vertex.pos {
        None
    } else {
        Some((
            Angle::from_point(vertex.nexts[0] - vertex.pos),
            Angle::from_point(vertex.nexts[1] - vertex.pos),
        ))
    };

    let mut visibles = Vec::new();
    'a: for &v in verteces.iter() {
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
        for &v2 in verteces.iter() {
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
}

const DBG: bool = false;

pub fn vis_graph_opt1(
    verteces: &mut Vec<PolyVertex>,
    coords: (usize, usize),
    obstacles: &[Polygon],
) -> Vec<(usize, usize)> {
    let pos = obstacles[coords.0].points()[coords.1];
    verteces.sort_by_key(|v| Angle::from_point(v.pos - pos));

    if DBG {
        dbg!(pos);
    }

    let vertex_i = verteces
        .iter()
        .position(|v| v.coords == coords)
        .expect("Invalid coords");
    let vertex = verteces.remove(vertex_i);

    let mut visibles = Vec::new();
    let ray = Cell::new(Ray {
        start: vertex.pos,
        end: vertex.pos + VecN([1., 0.]),
    });

    let invisible_part = if obstacles[coords.0].len() == 0 {
        None
    } else {
        Some((
            Angle::from_point(vertex.nexts[0] - vertex.pos),
            Angle::from_point(vertex.nexts[1] - vertex.pos),
        ))
    };

    let mut tree = BTreeSet::new();
    for &v in verteces.iter() {
        for npos in v.nexts {
            if npos == vertex.pos {
                continue;
            }
            let s = Segment {
                start: v.pos,
                end: npos,
            };
            if ray.get().intersect_segment(s) {
                tree.insert(SweepingTreeSegment {
                    ray: &ray,
                    segment: s,
                });
            }
        }
    }

    for v in verteces.iter() {
        if DBG {
            println!("Checking: {:?}", v.pos);
            println!(
                "Tree: {:?}",
                &tree
                    .iter()
                    .map(|s| s.segment)
                    .map(|s| format!(" {:?}-{:?} ", s.start, s.end))
                    .collect::<String>()
            );
        }

        let mut visible = false;

        // Check if its an extremum of the nearest segment
        if tree
            .first()
            .map_or(false, |s| s.segment.has_extremum(v.pos))
        {
            visible = true;
        }

        // Removes segments going to v from tree
        let new_ray = Ray {
            start: vertex.pos,
            end: v.pos,
        };
        for npos in v.nexts {
            if npos != vertex.pos && !new_ray.is_on_left_side(npos) {
                if DBG {
                    println!("Removed: {:?}-{:?}", v.pos, npos);
                }
                let was_present = tree.remove(&SweepingTreeSegment {
                    segment: Segment {
                        start: v.pos,
                        end: npos,
                    },
                    ray: &ray,
                });
                debug_assert!(was_present);
            }
        }

        // Sweep
        ray.set(new_ray);

        // Adds new segments coming from v
        for npos in v.nexts {
            // Don't add segments going to vertex
            if npos == vertex.pos {
                visible = true;
            } else if new_ray.is_on_left_side(npos) {
                if DBG {
                    println!("Added: {:?}-{:?}", v.pos, npos);
                }
                let was_inserted = tree.insert(SweepingTreeSegment {
                    segment: Segment {
                        start: v.pos,
                        end: npos,
                    },
                    ray: &ray,
                });
                debug_assert!(
                    Angle::from_point(v.pos - vertex.pos).abs() < Angle::new(0.05) || was_inserted
                );
            }
        }

        // Check if its an extremum of the nearest segment
        if tree
            .first()
            .map_or(false, |s| s.segment.has_extremum(v.pos))
        {
            visible = true;
        }

        if visible {
            if let Some((a1, a2)) = invisible_part {
                if Angle::from_point(v.pos - vertex.pos).is_between(a1, a2)
                    && !vertex.nexts.contains(&v.pos)
                {
                    continue;
                }
            }
            visibles.push(v.coords);
        }
        // tree.remove(&SweepingTreeSegment { segment: todo!(), ray: () });
    }

    verteces.push(vertex);
    visibles
}

pub fn compute_vis_graph_fullmap(
    obstacles: &[Polygon],
    method: fn(&mut Vec<PolyVertex>, (usize, usize), &[Polygon]) -> Vec<(usize, usize)>,
) -> MapGraph<(usize, usize)> {
    let mut verteces = to_vertice_vec(obstacles);

    let visibles_from = move |coords: (usize, usize)| (method)(&mut verteces, coords, obstacles);
    MapGraph::from_fn(coords_iterator(obstacles), visibles_from)
}
pub fn compute_vis_graph_cachemap<'a>(
    obstacles: &'a [Polygon],
    method: fn(&mut Vec<PolyVertex>, (usize, usize), &[Polygon]) -> Vec<(usize, usize)>,
) -> CachedFuncGraph<impl FnMut((usize, usize)) -> Vec<(usize, usize)> + 'a, (usize, usize)> {
    let mut verteces = to_vertice_vec(obstacles);

    let visibles_from = move |coords: (usize, usize)| (method)(&mut verteces, coords, obstacles);
    CachedFuncGraph::new(visibles_from)
}

#[test]
fn test_sweep() {
    let ray = Cell::new(Ray {
        start: VecN([0., 0.]),
        end: VecN([0., 1.]),
    });

    let mut s1 = SweepingTreeSegment {
        ray: &ray,
        segment: Segment {
            start: VecN([0.5, 0.5]),
            end: VecN([-0.5, 0.5]),
        },
    };
    let mut s2 = SweepingTreeSegment {
        ray: &ray,
        segment: Segment {
            start: VecN([0.5, 1.]),
            end: VecN([-0.5, 1.]),
        },
    };
    assert!(s1 < s2);

    s1.segment.start[0] = 5.;
    s1.segment.end[0] = -5.;
    assert!(s1 < s2);

    s1.segment.start[0] = 0.1;
    s1.segment.end[0] = -0.1;
    assert!(s1 < s2);

    s1.segment.end = VecN([-0.5, 1.]);
    assert!(s1 < s2);

    s1.segment.end = VecN([-0.5, 0.5]);
    s2.segment.end = VecN([-0.5, 0.5]);
    assert!(s1 < s2);

    s1.segment.start = VecN([0.5, 1.]);
    assert!(s1 == s2);

    s1.segment.start = VecN([-0.5, 0.5]);
    s1.segment.end = VecN([0.5, 1.]);
    assert!(s1 == s2);
}
