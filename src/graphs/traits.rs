use std::collections::{hash_map::Entry, HashMap};
use std::hash::Hash;

use crate::utils::numbers::NotNanF64;
use crate::utils::traits::Weight;
use crate::{datastructures::priority_queue::PriorityQueue, utils::traits::NormedSpace};

/// A graph interface
pub trait Graph<Vertex> {
    fn neighbors(&self, vertex: Vertex) -> impl Iterator<Item = Vertex>;
    fn dijkstra_with<W: Weight>(
        &self,
        start: Vertex,
        end: Vertex,
        cost_fn: impl Fn(Vertex, Vertex) -> W,
    ) -> Option<(Vec<Vertex>, W)>
    where
        Vertex: Hash + Eq + Copy,
    {
        let mut parent_weight = HashMap::new();
        let mut queue = PriorityQueue::default();
        parent_weight.insert(start, (start, W::ZERO));
        queue.push(W::ZERO, start);
        loop {
            let Some((weight, vertex)) = queue.pop_min() else {
                return None;
            };
            let (_, best_w) = parent_weight.get(&vertex).unwrap();
            if *best_w < weight {
                continue;
            }
            if vertex == end {
                let mut path = vec![end];
                let mut v = vertex;
                while v != start {
                    let (p, _) = parent_weight.get(&v).unwrap();
                    v = *p;
                    path.push(v);
                }
                path.reverse();
                return Some((path, weight));
            }
            for child in self.neighbors(vertex) {
                let new_weight = weight + cost_fn(vertex, child);
                assert!(new_weight >= weight);
                match parent_weight.entry(child) {
                    Entry::Vacant(e) => {
                        e.insert((vertex, new_weight));
                        queue.push(new_weight, child);
                    }
                    Entry::Occupied(mut e) => {
                        if e.get().1 <= new_weight {
                            continue;
                        } else {
                            e.insert((vertex, new_weight));
                            queue.push(new_weight, child);
                        }
                    }
                }
            }
        }
    }
    fn a_star_with<S: NormedSpace>(
        &self,
        start: Vertex,
        end: Vertex,
        pos_fn: impl Fn(Vertex) -> S,
    ) -> Option<(Vec<Vertex>, f64)>
    where
        Vertex: Hash + Eq + Copy,
    {
        let pos_end = pos_fn(end);
        self.dijkstra_with(start, end, |a, b| {
            let pos_a = pos_fn(a);
            let pos_b = pos_fn(b);
            NotNanF64::new(
                pos_a.distance(pos_b) + pos_b.distance(pos_end) - pos_a.distance(pos_end),
            )
        })
        .map(|(path, weight)| (path, *weight + pos_fn(start).distance(pos_end)))
    }
}

/// A graph where the collection of vertex is known
pub trait IterableGraph<V>: Graph<V> {
    fn iter(&self) -> impl Iterator<Item = V>;
}
