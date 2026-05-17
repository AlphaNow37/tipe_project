use crate::datastructures::priority_queue::PriorityQueue;
use crate::geometry::shapes::{InfiniteLine, Polygon, Ray, Segment};
use crate::geometry::VecN;
use crate::graphs::ParentTree;
use crate::triangulations::delaunay::make_delaynay;
use crate::triangulations::triangulation::Triangulation;
use crate::triangulations::triangulation_lineaire::triangulate_linear;
use crate::utils::numbers::{NotNanF64, Zero, F64_EPSILON};
use crate::workspace::cartesians::{EuclidianDistance, Length};
use std::collections::HashMap;
use std::sync::atomic::{AtomicUsize, Ordering};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PolyanyaMode {
    DijkstraExhaustive,
    Dijkstra,
    AStar,
}

fn sort([a, b]: [usize; 2]) -> [usize; 2] {
    if a < b {
        [a, b]
    } else {
        [b, a]
    }
}

type Edge = [usize; 2];
// interval id, to tri, edge
type Pqueue = PriorityQueue<NotNanF64, (usize, Option<usize>, [usize; 2])>;

static NEXT_ID: AtomicUsize = AtomicUsize::new(0);
pub fn generate_id() -> usize {
    NEXT_ID.fetch_add(1, Ordering::Relaxed)
}

struct Context<'a> {
    pqueue: Pqueue,
    dist_fun: Box<dyn Fn(InfiniteLine<2>, [f64; 2], VecN<2, f64>) -> f64 + 'a>,
    vposs: &'a [VecN<2, f64>],
}

#[derive(Clone, Copy, Debug)]
pub struct Interval {
    extrs_vertex_i: [Option<usize>; 2], // when the interval touches an extremity of the interval
    pub times: [f64; 2],                // times on the edge: times[0] < times[1]
    pub source: usize,
    pub source_pos: VecN<2, f64>,
    lag: f64,  // the distance between the global start of polyanya and the source
    id: usize, // unique id for each interval
}
impl Interval {
    // splits the segment into 3 pieces: the projection at the center, and two sub intervals
    fn project_onto(
        self,
        intervals: &mut Intervals,
        edge_segment: Segment<2>,
        ctx: &mut Context,
        from_tri: usize,
    ) {
        debug_assert!(self.check_invariants(None) == ());

        if self.source_pos == edge_segment.start || self.source_pos == edge_segment.end {
            intervals.insert_interval(
                Interval {
                    id: generate_id(),
                    lag: self.lag,
                    extrs_vertex_i: intervals.extrs_vertex_i.map(|i| Some(i)),
                    source_pos: self.source_pos,
                    source: self.source,
                    times: [0., 1.],
                },
                ctx,
                from_tri,
            );
            return;
        }
        debug_assert!(
            EuclidianDistance.length(self.source_pos - edge_segment.start) >= F64_EPSILON
        );
        debug_assert!(EuclidianDistance.length(self.source_pos - edge_segment.end) >= F64_EPSILON);

        let i_segment = Segment {
            start: if self.times[0] == 0. {
                edge_segment.start
            } else {
                edge_segment.to_line().point_at_time(self.times[0])
            },
            end: if self.times[1] == 1. {
                edge_segment.end
            } else {
                edge_segment.to_line().point_at_time(self.times[1])
            },
        };
        let ray_1 = Ray {
            start: self.source_pos,
            end: i_segment.start,
        };
        let ray_2 = Ray {
            start: self.source_pos,
            end: i_segment.end,
        };
        let d = intervals.segment.to_line();

        let [t1, t2]: [f64; 2] = [ray_1, ray_2].map(|r| {
            if r.end == d.start {
                return 0.;
            }
            if r.end == d.end {
                return 1.;
            }

            match d.intersection_time(r.to_line()) {
                Some((t, tr)) => {
                    if tr >= 0. {
                        t.clamp(0., 1.)
                    } else if (r.end - r.start).dot(d.end - d.start) > 0. {
                        1.
                    } else {
                        0.
                    }
                }
                None => {
                    println!("Parallels encountered !");
                    // parallels
                    if (r.end - r.start).dot(d.end - d.start) > 0. {
                        1.
                    } else {
                        0.
                    }
                }
            }
        });

        if (t1 == 0. && t2 == 0.) || (t1 == 1. && t2 == 1.) {
            let side = (t1 == 1.) as usize;
            let common_pos = if self.extrs_vertex_i[0] == Some(intervals.extrs_vertex_i[side]) {
                edge_segment.start
            } else if self.extrs_vertex_i[1] == Some(intervals.extrs_vertex_i[side]) {
                edge_segment.end
            } else {
                return;
            };
            intervals.insert_interval(
                Interval {
                    id: generate_id(),
                    lag: self.lag + EuclidianDistance.length(common_pos - self.source_pos),
                    extrs_vertex_i: [
                        Some(intervals.extrs_vertex_i[0]),
                        Some(intervals.extrs_vertex_i[1]),
                    ],
                    source_pos: common_pos,
                    source: intervals.extrs_vertex_i[side],
                    times: [0., 1.],
                },
                ctx,
                from_tri,
            );
            return;
        }

        debug_assert_ne!(t1, t2, "{:?}", self);
        debug_assert!(
            (t1 - t2).abs() >= F64_EPSILON,
            "{self:?}, {t1}, {t2} {edge_segment:?} {:?}",
            intervals.segment
        );

        // reversing if necessary
        let (t1, t2, extr1, extr2, p1, p2) = if t1 < t2 {
            (
                t1,
                t2,
                self.extrs_vertex_i[0],
                self.extrs_vertex_i[1],
                i_segment.start,
                i_segment.end,
            )
        } else {
            (
                t2,
                t1,
                self.extrs_vertex_i[1],
                self.extrs_vertex_i[0],
                i_segment.end,
                i_segment.start,
            )
        };

        debug_assert!(0. <= t1);
        debug_assert!(t1 + F64_EPSILON < t2);
        debug_assert!(t2 <= 1.);
        debug_assert!(t1 < 1.);
        debug_assert!(0. < t2);

        let proj_vertex_i_1 = (t1 == 0.).then_some(intervals.extrs_vertex_i[0]);
        let proj_vertex_i_2 = (t2 == 1.).then_some(intervals.extrs_vertex_i[1]);

        if t1 > 0. && extr1.is_some() {
            intervals.insert_interval(
                Interval {
                    id: generate_id(),
                    lag: self.lag + EuclidianDistance.length(p1 - self.source_pos),
                    extrs_vertex_i: [
                        Some(intervals.extrs_vertex_i[0]),
                        // (t1 == 1.).then_some(intervals.extrs_vertex_i[1]),
                        None,
                    ],
                    source_pos: p1,
                    source: extr1.unwrap(),
                    times: [0., t1],
                },
                ctx,
                from_tri,
            )
        }
        intervals.insert_interval(
            Interval {
                id: generate_id(),
                lag: self.lag,
                extrs_vertex_i: [proj_vertex_i_1, proj_vertex_i_2],
                source_pos: self.source_pos,
                source: self.source,
                times: [t1, t2],
            },
            ctx,
            from_tri,
        );
        if t2 < 1. && extr2.is_some() {
            intervals.insert_interval(
                Interval {
                    id: generate_id(),
                    lag: self.lag + EuclidianDistance.length(p2 - self.source_pos),
                    extrs_vertex_i: [
                        None,
                        // (t2 == 0.).then_some(intervals.extrs_vertex_i[0]),
                        Some(intervals.extrs_vertex_i[1]),
                    ],
                    source_pos: p2,
                    source: extr2.unwrap(),
                    times: [t2, 1.],
                },
                ctx,
                from_tri,
            )
        }
    }
    // fn split(self, t: f64) -> [Self; 2] {
    //     debug_assert!(self.times[0] < t && t < self.times[1]);
    //     [
    //         Interval {
    //             times: [self.times[0], t],
    //             extrs_vertex_i: [self.extrs_vertex_i[0], None],
    //             ..self
    //         },
    //         Interval {
    //             times: [t, self.times[1]],
    //             extrs_vertex_i: [None, self.extrs_vertex_i[1]],
    //             ..self
    //         },
    //     ]
    // }
    fn min_dist(self, edge_segment: Segment<2>, ctx: &Context) -> f64 {
        self.lag + (ctx.dist_fun)(edge_segment.to_line(), self.times, self.source_pos)

        // let t = edge_segment
        //     .to_line()
        //     .project_time(self.source_pos)
        //     .clamp(0., 1.);
        // EuclidianDistance.length(edge_segment.to_line().point_at_time(t) - self.source_pos)
        //     + self.lag
    }
    fn check_invariants(&self, vposs: Option<&[VecN<2, f64>]>) {
        debug_assert!(self.lag >= 0.);
        debug_assert_eq!(
            self.extrs_vertex_i[0].is_some(),
            self.times[0] == 0.,
            "{:?} {:?}",
            self.times,
            self.extrs_vertex_i
        );
        debug_assert_eq!(
            self.extrs_vertex_i[1].is_some(),
            self.times[1] == 1.,
            "{:?} {:?}",
            self.times,
            self.extrs_vertex_i
        );
        debug_assert!(self.times[0] < self.times[1], "{:?}", self.times);
        debug_assert!(self.times[0] < self.times[1] - F64_EPSILON, "{:?}", self);
        debug_assert!(0. <= self.times[0]);
        debug_assert!(self.times[1] <= 1.);
        if self.times[1] != 0. {
            debug_assert!(self.times[1] >= F64_EPSILON);
        }
        if self.times[1] != 1. {
            debug_assert!(self.times[1] <= 1. - F64_EPSILON);
        }
        if let Some(vposs) = vposs {
            debug_assert_eq!(vposs[self.source], self.source_pos);
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum BattleResult {
    /// The first interval wins completely
    W1,
    /// The second interval wins completely
    W2,
    /// There is a split at t
    W1W2(f64),
    W2W1(f64),
    /// There are two splits
    W2W1W2(f64, f64),
    W1W2W1(f64, f64),
}
impl BattleResult {
    fn p1_win_after(self, t: f64) -> (bool, f64) {
        match self {
            Self::W1 => (true, f64::INFINITY),
            Self::W2 => (false, f64::INFINITY),
            Self::W1W2(t2) => {
                if t < t2 {
                    (true, t2)
                } else {
                    (false, f64::INFINITY)
                }
            }
            Self::W2W1(t2) => {
                if t < t2 {
                    (false, t2)
                } else {
                    (true, f64::INFINITY)
                }
            }
            Self::W1W2W1(t2, t3) => {
                if t < t2 {
                    (true, t2)
                } else if t < t3 {
                    (false, t3)
                } else {
                    (true, f64::INFINITY)
                }
            }
            Self::W2W1W2(t2, t3) => {
                if t < t2 {
                    (false, t2)
                } else if t < t3 {
                    (true, t3)
                } else {
                    (false, f64::INFINITY)
                }
            }
        }
    }
}
fn interval_battle(i1: Interval, i2: Interval, segment: Segment<2>) -> BattleResult {
    if i1.source == i2.source {
        return if i1.lag <= i2.lag {
            BattleResult::W1
        } else {
            BattleResult::W2
        };
    }

    let k = i2.lag - i1.lag;
    let q1 = i1.source_pos;
    let q2 = i2.source_pos;
    let p0 = segment.start;
    let p1 = segment.end;

    let u = p1 - p0;
    let d1 = q1 - p0;
    let d2 = q2 - p0;

    let a12 = u[0] * u[0] + u[1] * u[1]; // a12 > 0
    let b1 = -2. * (u[0] * d1[0] + u[1] * d1[1]);
    let b2 = -2. * (u[0] * d2[0] + u[1] * d2[1]);
    let c1 = d1[0] * d1[0] + d1[1] * d1[1];
    let c2 = d2[0] * d2[0] + d2[1] * d2[1];

    let c = c1 - c2 - k * k;
    let b = b1 - b2;

    let f = 4. * k * k;
    let af = b * b - f * a12;
    let bf = 2. * b * c - f * b2; // here (b1->b2)
    let cf = c * c - f * c2; // here (c1->c2)

    let disc = bf * bf - 4. * af * cf;

    let winner_at_is_1 = |t: f64| {
        let dist_1: f64 = i1.lag + (t * t * a12 + t * b1 + c1).sqrt();
        let dist_2: f64 = i2.lag + (t * t * a12 + t * b2 + c2).sqrt();
        dist_1 < dist_2
    };

    if disc < -1.0e-9 {
        // There are no roots !
        if winner_at_is_1(0.) {
            BattleResult::W1
        } else {
            BattleResult::W2
        }
    } else if disc < 1.0e-9 {
        // Only one root ! this case happens everytime k=||q1-q0||
        let t = -bf / (2. * af);
        match (winner_at_is_1(t - 1.), winner_at_is_1(t + 1.)) {
            (true, true) => BattleResult::W1,
            (false, false) => BattleResult::W2,
            (true, false) => BattleResult::W1W2(t),
            (false, true) => BattleResult::W2W1(t),
        }
    } else {
        // There are two roots ! test between the roots
        let mut t1 = (-bf - disc.sqrt()) / (2. * af);
        let mut t2 = (-bf + disc.sqrt()) / (2. * af);

        if t2 < t1 {
            (t1, t2) = (t2, t1)
        }

        match (
            winner_at_is_1(t1 - 1.),
            winner_at_is_1((t1 + t2) / 2.),
            winner_at_is_1(t2 + 1.),
        ) {
            (true, true, true) => BattleResult::W1,
            (false, false, false) => BattleResult::W2,
            (true, true, false) => BattleResult::W1W2(t2),
            (true, false, false) => BattleResult::W1W2(t1),
            (false, false, true) => BattleResult::W2W1(t2),
            (false, true, true) => BattleResult::W2W1(t1),
            (true, false, true) => BattleResult::W1W2W1(t1, t2),
            (false, true, false) => BattleResult::W2W1W2(t1, t2),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Intervals {
    pub intervals: Vec<Interval>, // Invariant: sorted, int[i+1].start = int[i].end
    pub segment: Segment<2>,
    extrs_vertex_i: [usize; 2],
    pub adj_tris: [Option<usize>; 2],
}
impl Intervals {
    fn new(segment: Segment<2>, extrs_vertex_i: [usize; 2], adj_tris: [Option<usize>; 2]) -> Self {
        Self {
            intervals: Vec::new(),
            segment,
            extrs_vertex_i,
            adj_tris,
        }
    }
    fn add_interval_to_pqueue(&self, interval: Interval, ctx: &mut Context, from_tri: usize) {
        debug_assert!(self.adj_tris.contains(&Some(from_tri)));
        debug_assert!(interval.check_invariants(Some(ctx.vposs)) == ());
        ctx.pqueue.push(
            NotNanF64::new_debug_checked(interval.min_dist(self.segment, ctx)),
            (
                interval.id,
                if self.adj_tris[0] == Some(from_tri) {
                    self.adj_tris[1]
                } else {
                    self.adj_tris[0]
                },
                sort(self.extrs_vertex_i),
            ),
        )
    }
    // add an interval onto the list, merging them
    fn insert_interval(&mut self, mut interval: Interval, ctx: &mut Context, from_tri: usize) {
        debug_assert!(interval.check_invariants(Some(ctx.vposs)) == ());
        debug_assert!(self.check_invariants(Some(ctx.vposs)) == ());

        // dbg!(&self, &interval);

        let mut i = 0;
        loop {
            if interval.times[1] <= interval.times[0] + F64_EPSILON {
                return;
            }
            if i >= self.intervals.len() {
                self.intervals.push(interval);
                self.add_interval_to_pqueue(interval, ctx, from_tri);
                return;
            }
            let mut curr = self.intervals[i];
            if curr.times[0] + F64_EPSILON >= interval.times[1] {
                if curr.times[0] < interval.times[1] {
                    interval.times[1] = curr.times[0]
                }
                debug_assert!(interval.check_invariants(None) == ());
                self.intervals.insert(i, interval);
                self.add_interval_to_pqueue(interval, ctx, from_tri);
                return;
            }
            if curr.times[1] <= interval.times[0] + F64_EPSILON {
                if curr.times[1] > interval.times[0] {
                    interval.times[0] = curr.times[1]
                }
                i += 1;
                continue;
            }

            self.intervals.remove(i);

            debug_assert!(curr.times[0] < interval.times[1]);
            debug_assert!(curr.times[1] > interval.times[0]);

            let result = interval_battle(curr, interval, self.segment);

            let mut curr_winning = curr.times[0] < interval.times[0];
            let mut t = curr.times[0].max(interval.times[0]);
            let max_t = curr.times[1].min(interval.times[1]);
            while t < max_t {
                let (win1, t1) = result.p1_win_after(t);
                if !win1 && curr_winning {
                    if t - curr.times[0] >= F64_EPSILON {
                        let int = Interval {
                            times: [curr.times[0], t],
                            extrs_vertex_i: [curr.extrs_vertex_i[0], None],
                            ..curr
                        };
                        debug_assert!(int.check_invariants(None) == ());
                        self.intervals.insert(i, int);
                        i += 1;
                        interval.times[0] = t;
                        if t != 0. {
                            debug_assert!(t > 0.);
                            interval.extrs_vertex_i[0] = None;
                        }
                    } else {
                        interval.times[0] = curr.times[0];
                        if curr.times[0] != 0. {
                            debug_assert!(t > 0.);
                            interval.extrs_vertex_i[0] = None;
                        }
                    }
                } else if win1 && !curr_winning {
                    if t - interval.times[0] >= F64_EPSILON {
                        let int = Interval {
                            times: [interval.times[0], t],
                            extrs_vertex_i: [interval.extrs_vertex_i[0], None],
                            id: generate_id(),
                            ..interval
                        };
                        self.add_interval_to_pqueue(int, ctx, from_tri);
                        self.intervals.insert(i, int);
                        i += 1;
                        curr.times[0] = t;
                        if t != 0. {
                            debug_assert!(t > 0.);
                            curr.extrs_vertex_i[0] = None;
                        }
                    } else {
                        curr.times[0] = interval.times[0];
                        if interval.times[0] != 0. {
                            debug_assert!(t > 0.);
                            curr.extrs_vertex_i[0] = None;
                        }
                    }
                }
                curr_winning = win1;
                t = t1;
            }
            if curr_winning {
                if curr.times[1] - curr.times[0] >= F64_EPSILON {
                    self.intervals.insert(i, curr);
                    i += 1;
                    interval.times[0] = curr.times[1];
                    interval.extrs_vertex_i[0] = None;
                } else {
                    interval.times[0] = curr.times[0];
                    if curr.times[0] != 0. {
                        interval.extrs_vertex_i[0] = None;
                    }
                }
                if curr.times[1] >= interval.times[1] {
                    return;
                }
            }
        }
    }
    fn check_invariants(&self, vposs: Option<&[VecN<2, f64>]>) {
        if let Some(vposs) = vposs {
            debug_assert_eq!(self.segment.start, vposs[self.extrs_vertex_i[0]]);
            debug_assert_eq!(self.segment.end, vposs[self.extrs_vertex_i[1]]);
        }
        for i in 0..self.intervals.len() {
            self.intervals[i].check_invariants(vposs);
            if let Some(j) = self.intervals[i].extrs_vertex_i[0] {
                debug_assert_eq!(j, self.extrs_vertex_i[0]);
            }
            if let Some(j) = self.intervals[i].extrs_vertex_i[1] {
                debug_assert_eq!(j, self.extrs_vertex_i[1]);
            }
        }
        if self.intervals.len() != 0 {
            for i in 0..(self.intervals.len() - 1) {
                debug_assert!(self.intervals[i].times[1] <= self.intervals[i + 1].times[0]);
            }
        }
        debug_assert_ne!(self.adj_tris[0], self.adj_tris[1]);
    }
}

fn check_map_invariants(map: &HashMap<Edge, Intervals>, vposs: &[VecN<2, f64>]) {
    for (edge, ints) in map.iter() {
        debug_assert_eq!(ints.extrs_vertex_i, *edge);
        ints.check_invariants(Some(vposs));
    }
}

fn build_parent_tree(
    interval_map: &HashMap<Edge, Intervals>,
    goal: usize,
    vposs: &[VecN<2, f64>],
) -> (ParentTree<usize, usize>, f64) {
    let mut ptree = ParentTree::new();
    let mut best_goal_dist = f64::INFINITY;

    for ints in interval_map.values() {
        if ints.intervals.len() == 0 {
            continue;
        }
        for (int, j) in [
            (ints.intervals[0], 0),
            (ints.intervals[ints.intervals.len() - 1], 1),
        ] {
            if let Some(i) = int.extrs_vertex_i[j] {
                let p = int.source;
                if i == goal && p != goal {
                    let d = int.lag + EuclidianDistance.length(vposs[goal] - int.source_pos);
                    if d < best_goal_dist {
                        ptree.set_parent(goal, p);
                        best_goal_dist = d;
                    }
                } else if i != p {
                    ptree.set_parent(i, p)
                }
            }
        }
    }
    // dbg!(&ptree);
    (ptree, best_goal_dist)
}

pub fn polyanya(
    t: &Triangulation,
    start: usize,
    goal: usize,
    mode: PolyanyaMode,
) -> (Option<(Vec<usize>, f64)>, HashMap<Edge, Intervals>) {
    // 1: create the hashmap for intervals, a min heap for node ids, the interval map, a buffer, the context

    // (usize, usize, usize): (interval id, tri to expand to, edge. the weight is the closest distance on the interval
    let mut pqueue: Pqueue = PriorityQueue::default();
    // edge ([usize; 2] sorted) to intervals
    let mut intervals_map: HashMap<Edge, Intervals> = HashMap::new();
    // A pre-allocated buffer
    let mut ints_with_right_id = Vec::with_capacity(4);

    let goal_pos = t.vertex_poss[goal];
    let mut context = Context {
        dist_fun: match mode {
            PolyanyaMode::Dijkstra | PolyanyaMode::DijkstraExhaustive => {
                Box::new(|line, times, source_pos| {
                    let time = line.project_time(source_pos).clamp(times[0], times[1]);
                    EuclidianDistance.length(line.point_at_time(time) - source_pos)
                })
            }
            PolyanyaMode::AStar => Box::new(|line, times, source_pos| {
                let t1 = line.project_time(source_pos);
                let t2 = line.project_time(goal_pos);
                let p1 = line.point_at_time(t1);
                let p2 = line.point_at_time(t2);
                let d1 = EuclidianDistance.length(p1 - source_pos);
                let d2 = EuclidianDistance.length(p2 - goal_pos);
                if d1 + d2 < F64_EPSILON {
                    return 0.;
                }
                let time = ((t1 * d2 + t2 * d1) / (d1 + d2)).clamp(times[0], times[1]);
                EuclidianDistance.length(line.point_at_time(time) - source_pos)
                    + EuclidianDistance.length(line.point_at_time(time) - goal_pos)
            }),
        },
        pqueue,
        vposs: &t.vertex_poss,
    };

    // 2: find a first interval to add
    'a: {
        for i in 0..t.triangles.len() {
            let tri = t.triangles[i];
            for k in 0..3 {
                if tri[k].verts[0] == start {
                    let verts = sort(tri[k].verts);
                    let id = generate_id();
                    let ints = Intervals {
                        adj_tris: [Some(i), tri[k].other_tri],
                        segment: Segment {
                            start: t.vertex_poss[verts[0]],
                            end: t.vertex_poss[verts[1]],
                        },
                        extrs_vertex_i: verts,
                        intervals: vec![Interval {
                            extrs_vertex_i: [Some(verts[0]), Some(verts[1])],
                            lag: 0.,
                            id,
                            source_pos: t.vertex_poss[start],
                            source: start,
                            times: [0., 1.],
                        }],
                    };
                    intervals_map.insert(sort(verts), ints);
                    context
                        .pqueue
                        .push(NotNanF64::ZERO, (id, Some(i), sort(verts)));
                    context
                        .pqueue
                        .push(NotNanF64::ZERO, (id, tri[k].other_tri, sort(verts)));
                    break 'a;
                }
            }
        }
        panic!("Could not find a starting interval !");
    }

    // 3: loop around

    let mut curr_best = f64::INFINITY;

    'a: while let Some((min_dist, (id, to_tri_i, edge))) = context.pqueue.pop_min() {
        debug_assert!(check_map_invariants(&intervals_map, &t.vertex_poss) == ());

        // dbg!(min_dist);

        if *min_dist > curr_best && mode != PolyanyaMode::DijkstraExhaustive {
            // dbg!(&intervals_map);

            // dbg!("Building the parent tree !");
            let (ptree, dist) = build_parent_tree(&intervals_map, goal, &t.vertex_poss);
            // dbg!("Finding a path !");
            let mut path = ptree.path_to(goal);
            path.push(goal);
            // dbg!(dist, curr_best);
            return (Some((path, curr_best)), intervals_map);
        }

        debug_assert_eq!(edge, sort(edge));

        let ints = intervals_map
            .get(&edge)
            .expect("This edge should be in the map if it is popped !");
        let segment = ints.segment;
        debug_assert_eq!(edge, ints.extrs_vertex_i);

        ints_with_right_id.clear();
        ints.intervals
            .iter()
            .filter(|int| int.id == id)
            .copied()
            .collect_into(&mut ints_with_right_id);
        for &int in &ints_with_right_id {
            if int.extrs_vertex_i[0] == Some(goal) || int.extrs_vertex_i[1] == Some(goal) {
                let new_dist =
                    int.lag + EuclidianDistance.length(int.source_pos - t.vertex_poss[goal]);
                if new_dist < curr_best {
                    curr_best = new_dist;
                }
            }

            let Some(to_tri_i) = to_tri_i else {
                continue 'a;
            };
            let tri = t.triangles[to_tri_i];

            for k in 0..3 {
                let new_edge = sort(tri[k].verts);
                if new_edge != edge {
                    let mut entry = intervals_map.entry(new_edge);
                    let new_ints = entry.or_insert_with(|| {
                        let segment = Segment {
                            start: t.vertex_poss[new_edge[0]],
                            end: t.vertex_poss[new_edge[1]],
                        };
                        Intervals::new(segment, new_edge, [Some(to_tri_i), tri[k].other_tri])
                    });
                    int.project_onto(new_ints, segment, &mut context, to_tri_i);
                }
            }
        }
    }

    if curr_best == f64::INFINITY {
        (None, intervals_map)
    } else {
        // dbg!("Building the parent tree !");
        let (ptree, dist) = build_parent_tree(&intervals_map, goal, &t.vertex_poss);
        // dbg!("Finding a path !");
        let mut path = ptree.path_to(goal);
        path.push(goal);
        // dbg!(dist, curr_best);
        return (Some((path, curr_best)), intervals_map);
    }
}

pub fn find_start_goal_idx(
    start: (usize, usize),
    goal: (usize, usize),
    obstacles: &[Polygon],
    tri: &Triangulation,
) -> (usize, usize) {
    let mut new_start = None;
    let mut new_goal = None;
    for j in 0..tri.vertex_poss.len() {
        if tri.vertex_poss[j] == obstacles[start.0].0[start.1] {
            new_start = Some(j);
        }
        if tri.vertex_poss[j] == obstacles[goal.0].0[goal.1] {
            new_goal = Some(j);
        }
    }
    (new_start.unwrap(), new_goal.unwrap())
}

pub fn shortest_path_polyanya(
    obstacles: &[Polygon],
    start: (usize, usize),
    goal: (usize, usize),
    mode: PolyanyaMode,
) -> (
    Triangulation,
    Option<(Vec<VecN<2, f64>>, f64)>,
    HashMap<Edge, Intervals>,
) {
    let mut tri = triangulate_linear(obstacles, 100.);
    let (new_start, new_goal) = find_start_goal_idx(start, goal, obstacles, &tri);
    make_delaynay(&mut tri);
    let (opt, map) = polyanya(&tri, new_start, new_goal, mode);
    let opt2 = opt.map(|(path, length)| {
        let path2 = path.iter().map(|i| tri.vertex_poss[*i]).collect::<Vec<_>>();
        // debug_assert!(
        //     (path2
        //         .iter()
        //         .map_windows(|[a, b]| EuclidianDistance.length(**b - **a))
        //         .sum::<f64>()
        //         - length)
        //         .abs()
        //         < F64_EPSILON
        // );
        // dbg!(path2
        //     .iter()
        //     .map_windows(|[a, b]| EuclidianDistance.length(**b - **a))
        //     .sum::<f64>());
        (path2, length)
    });
    (tri, opt2, map)
}
