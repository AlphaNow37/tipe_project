use crate::datastructures::skip_list::{Cursor, Interval, IntervalSkipLists, SkipListAccess};
use crate::geometry::shapes::{are_counter_clockwise, Cube, Polygon};
use crate::geometry::VecN;
use crate::triangulations::triangulation::{TriAdjacentEdge, Triangulation};
use crate::utils::numbers::{UsizeExt, F64_EPSILON};
use std::cmp::Ordering;
use std::collections::HashMap;
// convention: up (algo) = left (skiplist)

#[derive(Copy, Clone, Debug)]
enum VertexKind {
    WallTop(VecN<2, f64>),
    WallBot(VecN<2, f64>),
    Source([VecN<2, f64>; 2]), // top-down
    Sink,
    Merge,
    Split([VecN<2, f64>; 2]), // top-down
}

#[derive(Copy, Clone, Debug)]
struct Vertex {
    pos: VecN<2, f64>,
    kind: VertexKind,
}

#[derive(Clone, Copy, Debug)]
enum AreaCover {
    NoneTop,            // the face faces left-top
    NoneBot,            // the face faces left-bot
    Interval(f64, f64), // down-top
}

impl AreaCover {
    /// p1, p2 are counterclockwise from the point of vue of the empty space
    fn from_poss(p1: VecN<2, f64>, p2: VecN<2, f64>) -> Self {
        if p1[1] >= p2[1] {
            AreaCover::Interval(p2[1], p1[1])
        } else if p1[0] <= p2[0] {
            AreaCover::NoneTop
        } else {
            AreaCover::NoneBot
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct AreaInterval {
    vertices: [usize; 2], // counterclockwise from the point of vue of the empty space
    cover: AreaCover,
}

impl Interval for AreaInterval {
    type Value = f64;
    fn cmp_value(&self, value: &Self::Value) -> Ordering {
        // returns greater iff the value is lower down
        match self.cover {
            AreaCover::NoneTop => Ordering::Greater,
            AreaCover::NoneBot => Ordering::Less,
            AreaCover::Interval(low, high) => {
                if high < *value {
                    Ordering::Less
                } else if low <= *value {
                    Ordering::Equal
                } else {
                    Ordering::Greater
                }
            }
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum FrontContent {
    SingleVertex(usize),
    SegmentFront(SkipListAccess),
}

#[derive(Copy, Clone, Debug)]
struct FrontInterval {
    top: [VecN<2, f64>; 2], // left-right
    bot: [VecN<2, f64>; 2], // left-right
    content: FrontContent,
}
impl Interval for FrontInterval {
    type Value = VecN<2, f64>;
    fn cmp_value(&self, value: &Self::Value) -> Ordering {
        // returns greater iff the value is lower down
        if *value == self.top[1] || *value == self.bot[1] {
            return Ordering::Equal;
        }
        let dir1 = self.top[1] - self.top[0];
        let delta1 = *value - self.top[0];
        let dir2 = self.bot[1] - self.bot[0];
        let delta2 = *value - self.bot[0];
        if dir1.rotate_right().dot(delta1) < 0. {
            return Ordering::Less;
        }
        if dir2.rotate_left().dot(delta2) < 0. {
            return Ordering::Greater;
        }
        Ordering::Equal
    }
}

/// Start and end up on the area right on top of the new vertex
fn propagate_up(
    mut cursor: &mut Cursor<AreaInterval>,
    triangulation: &mut Triangulation,
    edge_to_tri: &mut HashMap<[usize; 2], usize>,
    new_i: usize,
) {
    let pos = triangulation.vertex_poss[new_i];
    loop {
        debug_assert!(!cursor.get_interval().is_none());
        let &AreaInterval {
            vertices: [a, b], ..
        } = cursor
            .get_interval()
            .expect("The cursor was not on an area !");

        debug_assert!(b == new_i);

        cursor.move_left();
        if cursor.is_full_left() {
            cursor.move_right();
            return;
        }
        cursor.move_left();
        let &AreaInterval {
            vertices: [v1, v2],
            ..
        } = cursor
            .get_interval()
            .expect("The cursor is not on the left");

        debug_assert!(a == v2);

        let p1 = triangulation.vertex_poss[v1];
        let p2 = triangulation.vertex_poss[v2];
        if (p2 - p1).rotate_left().dot(pos - p1) < 0. {
            cursor.move_right();
            cursor.move_right();
            return;
        }
        cursor.remove_interval();
        cursor.move_right();
        cursor.remove_interval();

        debug_assert!(are_counter_clockwise(
            &[p1, p2, pos]
        ));

        let new_t_i = triangulation.add_triangle([
            TriAdjacentEdge {
                verts: [v1, v2],
                other_tri: edge_to_tri.remove(&[v1, v2]),
            },
            TriAdjacentEdge {
                verts: [v2, new_i],
                other_tri: edge_to_tri.remove(&[v2, new_i]),
            },
            TriAdjacentEdge {
                verts: [new_i, v1],
                other_tri: None,
            },
        ]);

        edge_to_tri.insert([v1, new_i], new_t_i);
        edge_to_tri.insert([new_i, v2], new_t_i);

        cursor.push_interval(AreaInterval {
            cover: AreaCover::from_poss(p1, pos),
            vertices: [v1, new_i],
        })
    }
}

// Start (and end) on the area right below the new vertex
fn propagate_down(
    mut cursor: &mut Cursor<AreaInterval>,
    triangulation: &mut Triangulation,
    edge_to_tri: &mut HashMap<[usize; 2], usize>,
    new_i: usize,
) {
    let pos = triangulation.vertex_poss[new_i];
    loop {
        debug_assert!(!cursor.get_interval().is_none());
        let &AreaInterval {
            vertices: [a, b], ..
        } = cursor
            .get_interval()
            .expect("The cursor was not on an area !");

        debug_assert!(a == new_i);

        cursor.move_right();
        if cursor.is_full_right() {
            cursor.move_left();
            return;
        }
        cursor.move_right();
        let &AreaInterval {
            vertices: [v1, v2],
            ..
        } = cursor
            .get_interval()
            .expect("The cursor is not on the left");

        debug_assert!(b == v1);

        let p1 = triangulation.vertex_poss[v1];
        let p2 = triangulation.vertex_poss[v2];
        if (p2 - p1).rotate_left().dot(pos - p1) < 0. {
            cursor.move_left();
            cursor.move_left();
            debug_assert!(!cursor.get_interval().is_none());
            return;
        }
        cursor.remove_interval();
        cursor.move_left();
        cursor.remove_interval();

        debug_assert!(are_counter_clockwise(
            &[p1, p2, pos]
        ));

        let new_t_i = triangulation.add_triangle([
            TriAdjacentEdge {
                verts: [v1, v2],
                other_tri: edge_to_tri.remove(&[v1, v2]),
            },
            TriAdjacentEdge {
                verts: [v2, new_i],
                other_tri: None,
            },
            TriAdjacentEdge {
                verts: [new_i, v1],
                other_tri: edge_to_tri.remove(&[new_i, v1]),
            },
        ]);

        edge_to_tri.insert([new_i, v2], new_t_i);
        edge_to_tri.insert([v1, new_i], new_t_i);

        cursor.push_interval(AreaInterval {
            cover: AreaCover::from_poss(pos, p2),
            vertices: [new_i, v2],
        })
    }
}

/// the returned cursor is on the position of the vertex, not on an interval
fn insert_vertex_front<'a>(
    interval: &'a mut FrontContent,
    new_i: usize,
    areas: &'a mut IntervalSkipLists<AreaInterval>,
    triangulation: &mut Triangulation,
    edge_to_tri: &mut HashMap<[usize; 2], usize>,
    add_area_top: bool,
    add_area_bot: bool,
) -> Cursor<'a, AreaInterval> {
    let curr_pos = triangulation.vertex_poss[new_i];
    match interval {
        FrontContent::SingleVertex(i) => {
            let i = *i;
            let acc = SkipListAccess::new();
            *interval = FrontContent::SegmentFront(acc);
            let FrontContent::SegmentFront(acc) = interval else {unreachable!()};
            let mut cursor = areas.cursor_left(acc);
            let pos = triangulation.vertex_poss[i];

            if add_area_bot {
                cursor.push_interval(AreaInterval {
                    vertices: [new_i, i],
                    cover: AreaCover::from_poss(curr_pos, pos),
                });
                cursor.move_left();
            }
            if add_area_top {
                cursor.push_interval(AreaInterval {
                    vertices: [i, new_i],
                    cover: AreaCover::from_poss(pos, curr_pos),
                });
                cursor.move_right();
            }

            cursor
        }
        FrontContent::SegmentFront(area_access) => {
            let mut cursor = areas.cursor(area_access, curr_pos[1]);

            // adjacent of the cursor at the end of the block
            let mut hooked_vertex;
            if cursor.is_full_left() {
                cursor.move_right();
                let AreaInterval {
                    vertices: [a, _], ..
                } = cursor
                    .get_interval()
                    .expect("There should be an interval here");
                hooked_vertex = *a;
                cursor.move_left();
            } else if let Some(AreaInterval {
                vertices: [a, _], ..
            }) = cursor.get_interval()
            {
                hooked_vertex = *a;
                cursor.move_left();
            } else {
                cursor.move_left();
                let AreaInterval {
                    vertices: [_, a], ..
                } = cursor
                    .get_interval()
                    .expect("There should be an interval here");
                hooked_vertex = *a;
                cursor.move_right();
            }
            let hook_pos = triangulation.vertex_poss[hooked_vertex];

            debug_assert!(cursor.get_interval().is_none());

            cursor.push_interval(AreaInterval {
                cover: AreaCover::from_poss(curr_pos, hook_pos),
                vertices: [new_i, hooked_vertex],
            });
            propagate_down(&mut cursor, triangulation, edge_to_tri, new_i);

            debug_assert!(!cursor.get_interval().is_none());

            if !add_area_bot {
                cursor.remove_interval()
            } else {
                cursor.move_left();
            }

            debug_assert!(cursor.get_interval().is_none());

            cursor.push_interval(AreaInterval {
                cover: AreaCover::from_poss(hook_pos, curr_pos),
                vertices: [hooked_vertex, new_i],
            });

            debug_assert!(!cursor.get_interval().is_none());

            propagate_up(&mut cursor, triangulation, edge_to_tri, new_i);

            debug_assert!(!cursor.get_interval().is_none());

            if !add_area_top {
                cursor.remove_interval()
            } else {
                cursor.move_right();
            }

            debug_assert!(cursor.get_interval().is_none());

            cursor
        }
    }
}

fn add_polygon(vertices: &mut Vec<Vertex>, poly: &Polygon) {
    for i in 0..poly.len() {
        let prev = poly.0[i.add_rem(-1, poly.len())];
        let next = poly.0[i.add_rem(1, poly.len())];
        let curr = poly.0[i];
        let delta_prev = prev - curr;
        let delta_next = next - curr;
        vertices.push(Vertex {
            pos: curr,
            kind: match (
                delta_prev[0] > 0.,
                delta_next[0] > 0.,
                delta_prev.dot(delta_next.rotate_left()) > 0.,
            ) {
                (false, false, false) => VertexKind::Sink,
                (true, true, false) => VertexKind::Source([next, prev]),
                (true, true, true) => VertexKind::Split([prev, next]),
                (false, false, true) => VertexKind::Merge,
                (true, false, _) => VertexKind::WallBot(prev),
                (false, true, _) => VertexKind::WallTop(next),
            },
        });
    }
}

// Assume that no two vertices have the same x or y coord
pub fn triangulate_linear(polygons: &[Polygon], margin: f64) -> Triangulation {
    let mut vertices = Vec::new();

    let mut enveloppe = None;

    for poly in polygons {
        for i in 0..poly.len() {
            enveloppe = match enveloppe {
                None => Some(Cube::from_point(poly.0[i])),
                Some(c) => Some(c.with_point(poly.0[i]))
            }
        }
    }
    let enveloppe = enveloppe.unwrap_or(Cube::default());

    for poly in polygons {
        add_polygon(&mut vertices, &poly);
    }
    add_polygon(&mut vertices, &Polygon(vec![
        enveloppe.topleft() + VecN([-margin-F64_EPSILON, margin-F64_EPSILON]),
        enveloppe.topright() + VecN([margin-F64_EPSILON, margin+F64_EPSILON]),
        enveloppe.botright() + VecN([margin+F64_EPSILON, -margin-F64_EPSILON]),
        enveloppe.botleft() + VecN([-margin+F64_EPSILON, -margin+F64_EPSILON]),
    ]));

    vertices.sort_by(|v1, v2| v1.pos[0].total_cmp(&v2.pos[0]));
    let poss = vertices.iter().map(|v| v.pos).collect::<Vec<_>>();

    let mut lists_fronts = IntervalSkipLists::new();
    let mut lists_areas = IntervalSkipLists::new();
    let mut front_access = SkipListAccess::new();

    // counterclockwise from the point of vue of the empty space
    let mut edge_to_tri: HashMap<[usize; 2], usize> = HashMap::new();

    let mut triangulation = Triangulation::new(poss);

    for (i, &v) in vertices.iter().enumerate() {
        let mut cursor_fronts = lists_fronts.cursor(&mut front_access, v.pos);

        match v.kind {
            VertexKind::Source([top, bot]) => {
                debug_assert!(cursor_fronts.get_interval().is_none());
                cursor_fronts.push_interval(FrontInterval {
                    bot: [v.pos, bot],
                    top: [v.pos, top],
                    content: FrontContent::SingleVertex(i),
                });
            }

            VertexKind::WallTop(out_pos) => {
                let front_interval = cursor_fronts
                    .get_interval_mut()
                    .expect("Expected to be in a front");
                debug_assert_eq!(front_interval.top[1], v.pos);
                insert_vertex_front(
                    &mut front_interval.content,
                    i,
                    &mut lists_areas,
                    &mut triangulation,
                    &mut edge_to_tri,
                    false,
                    true,
                );
                front_interval.top = [v.pos, out_pos];
            }
            VertexKind::WallBot(out_pos) => {
                let front_interval = cursor_fronts
                    .get_interval_mut()
                    .expect("Expected to be in a front");
                debug_assert_eq!(front_interval.bot[1], v.pos);
                insert_vertex_front(
                    &mut front_interval.content,
                    i,
                    &mut lists_areas,
                    &mut triangulation,
                    &mut edge_to_tri,
                    true,
                    false,
                );
                front_interval.bot = [v.pos, out_pos];
            }
            VertexKind::Sink => {
                let front_interval = cursor_fronts
                    .get_interval_mut()
                    .expect("Expected to be in a front");
                debug_assert_eq!(front_interval.bot[1], v.pos);
                debug_assert_eq!(front_interval.top[1], v.pos);
                let c = insert_vertex_front(
                    &mut front_interval.content,
                    i,
                    &mut lists_areas,
                    &mut triangulation,
                    &mut edge_to_tri,
                    false,
                    false,
                );
                debug_assert!(c.is_full_left());
                debug_assert!(c.is_full_right());
                cursor_fronts.remove_interval();
            }
            VertexKind::Split([out_top, out_bot]) => {
                let front_interval = cursor_fronts
                    .get_interval_mut()
                    .expect("Expected to be in a front");
                let bot2 = front_interval.bot;
                let mut c = insert_vertex_front(
                    &mut front_interval.content,
                    i,
                    &mut lists_areas,
                    &mut triangulation,
                    &mut edge_to_tri,
                    true,
                    true,
                );
                let acc_bot = c.split_list();
                front_interval.bot = [v.pos, out_top];
                cursor_fronts.move_right();
                cursor_fronts.push_interval(FrontInterval {
                    bot: bot2,
                    top: [v.pos, out_bot],
                    content: FrontContent::SegmentFront(acc_bot),
                });
            }
            VertexKind::Merge => {
                cursor_fronts.move_right();
                cursor_fronts.move_right();
                let front_2 = cursor_fronts
                    .get_interval_mut()
                    .expect("Expected to be in a front");
                insert_vertex_front(
                    &mut front_2.content,
                    i,
                    &mut lists_areas,
                    &mut triangulation,
                    &mut edge_to_tri,
                    false,
                    true,
                );
                debug_assert_eq!(front_2.top[1], v.pos);
                let bot2 = front_2.bot;
                let FrontContent::SegmentFront(acc2) = front_2.content else {panic!("Not possible after adding a vertex");};

                cursor_fronts.remove_interval();
                cursor_fronts.move_left();
                let mut front_1 = cursor_fronts
                    .get_interval_mut()
                    .expect("Expected to be in a front");
                let mut cursor1 = insert_vertex_front(
                    &mut front_1.content,
                    i,
                    &mut lists_areas,
                    &mut triangulation,
                    &mut edge_to_tri,
                    true,
                    false,
                );
                debug_assert!(cursor1.get_interval().is_none());
                debug_assert_eq!(front_1.bot[1], v.pos);
                debug_assert!(cursor1.is_full_right());

                cursor1.concat_list(acc2);

                front_1.bot = bot2;
            }
        }
    }

    debug_assert!(triangulation.verify_invariants() == ());

    triangulation
}
