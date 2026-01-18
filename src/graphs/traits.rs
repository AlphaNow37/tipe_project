use std::collections::{hash_map::Entry, HashMap};
use std::fmt::Debug;
use std::hash::Hash;
use std::ops::Range;
use std::sync::Arc;

use crate::datastructures::priority_queue::PriorityQueue;
use crate::graphs::SubGraph;
use crate::utils::numbers::NotNanF64;
use crate::utils::traits::Weight;
use crate::workspace::obstacles::ObstaclesEnv;
use crate::workspace::workspace::WorkspaceTopology;

fn dist_heuristic<W: WorkspaceTopology>(
    workspace: &W,
    end: W::Vertex,
    a: W::Vertex,
    b: W::Vertex,
) -> NotNanF64 {
    NotNanF64::new(
        workspace.distance(a, b) + workspace.distance(b, end) - workspace.distance(a, end),
    )
}

/// A graph interface
pub trait Graph<Vertex, Info = Vertex> {
    fn neighbors(&self, vertex: Vertex) -> impl Iterator<Item = Info>;
    fn dijkstra_with<W: Weight>(
        &self,
        start: Vertex,
        end_predicate: impl Fn(Vertex) -> bool,
        dist_fn: impl Fn(Vertex, Vertex) -> W,
        should_shortcut: impl Fn(Vertex, Vertex) -> bool,
    ) -> Option<(Vec<Vertex>, W)>
    where
        Vertex: Hash + Eq + Copy,
        Info: Into<Vertex>,
    {
        let mut parent_cost = HashMap::new();
        let mut queue = PriorityQueue::default();
        parent_cost.insert(start, (start, W::ZERO));
        queue.push(W::ZERO, start);
        loop {
            let Some((cost, vertex)) = queue.pop_min() else {
                return None;
            };
            let (parent, best_cost) = parent_cost[&vertex];
            if best_cost < cost {
                continue;
            }
            if end_predicate(vertex) {
                let mut path = vec![vertex];
                let mut v = vertex;
                while v != start {
                    v = parent_cost[&v].0;
                    path.push(v);
                }
                path.reverse();
                return Some((path, cost));
            }
            for child_info in self.neighbors(vertex) {
                let child = child_info.into();
                let (new_parent, new_cost, pcost) = if should_shortcut(parent, child) {
                    let pcost = parent_cost[&parent].1;
                    (parent, pcost + dist_fn(parent, child), pcost)
                } else {
                    (vertex, cost + dist_fn(vertex, child), cost)
                };
                debug_assert!(new_cost >= pcost);
                match parent_cost.entry(child) {
                    Entry::Vacant(e) => {
                        e.insert((new_parent, new_cost));
                        queue.push(new_cost, child);
                    }
                    Entry::Occupied(mut e) => {
                        if e.get().1 <= new_cost {
                            continue;
                        } else {
                            e.insert((new_parent, new_cost));
                            queue.push(new_cost, child);
                        }
                    }
                }
            }
        }
    }
    fn a_star_with<W: WorkspaceTopology>(
        &self,
        start: Vertex,
        end: Vertex,
        pos_fn: impl Fn(Vertex) -> W::Vertex,
        workspace: &W,
    ) -> Option<(Vec<Vertex>, f64)>
    where
        Vertex: Hash + Eq + Copy,
        Info: Into<Vertex>,
    {
        let pos_end = pos_fn(end);
        self.dijkstra_with(
            start,
            |pt| pt == end,
            |a, b| dist_heuristic(workspace, pos_end, pos_fn(a), pos_fn(b)),
            |_, _| false,
        )
        .map(|(path, weight)| (path, *weight + workspace.distance(pos_fn(start), pos_end)))
    }

    fn theta_star_with<W: WorkspaceTopology>(
        &self,
        start: Vertex,
        end: Vertex,
        pos_fn: impl Fn(Vertex) -> W::Vertex,
        workspace: &W,
        obstacles: &impl ObstaclesEnv<W>,
    ) -> Option<(Vec<Vertex>, f64)>
    where
        Vertex: Hash + Eq + Copy,
        Info: Into<Vertex>,
    {
        let pos_end = pos_fn(end);
        self.dijkstra_with(
            start,
            |pt| pt == end,
            |a, b| dist_heuristic(workspace, pos_end, pos_fn(a), pos_fn(b)),
            |a, b| !obstacles.collide_segment(workspace.segment(pos_fn(a), pos_fn(b))),
        )
        .map(|(path, weight)| (path, *weight + workspace.distance(pos_fn(start), pos_end)))
    }

    fn theta_with<W: WorkspaceTopology>(
        &self,
        start: Vertex,
        end_predicate: impl Fn(Vertex) -> bool,
        pos_fn: impl Fn(Vertex) -> W::Vertex,
        workspace: &W,
        obstacles: &impl ObstaclesEnv<W>,
    ) -> Option<(Vec<Vertex>, f64)>
    where
        Vertex: Hash + Eq + Copy,
        Info: Into<Vertex>,
    {
        self.dijkstra_with(
            start,
            end_predicate,
            |a, b| NotNanF64::new(workspace.distance(pos_fn(a), pos_fn(b))),
            |a, b| !obstacles.collide_segment(workspace.segment(pos_fn(a), pos_fn(b))),
        )
        .map(|(path, weight)| (path, *weight))
    }

    fn subgraph<'a, F: Fn(&Vertex, &Vertex) -> bool + 'a>(
        self,
        filter: F,
    ) -> SubGraph<'a, Vertex, Self>
    where
        Self: Sized,
    {
        SubGraph {
            graph: self,
            filter: Arc::new(filter),
        }
    }
}

/// A graph where the collection of vertex is known
pub trait IterableGraph<V, Info = V>: Graph<V, Info> {
    fn iter(&self) -> impl Iterator<Item = V>;

    fn debug(&self)
    where
        V: Debug,
        Info: Into<V>,
    {
        for u in self.iter() {
            print!("{u:?} :     ");
            for v in self.neighbors(u) {
                print!("{:?} ; ", v.into());
            }
            println!();
        }
    }
}

/// Path graph
impl Graph<usize> for Range<usize> {
    fn neighbors(&self, vertex: usize) -> impl Iterator<Item = usize> {
        debug_assert!(self.start <= vertex && vertex < self.end);
        let next = vertex + 1;
        (next < self.end).then_some(next).into_iter()
    }
}
impl IterableGraph<usize> for Range<usize> {
    fn iter(&self) -> impl Iterator<Item = usize> {
        self.clone()
    }
}
