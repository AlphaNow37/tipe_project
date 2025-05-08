use std::collections::HashMap;
use std::hash::Hash;

use crate::{
    datastructures::{
        priority_queue::PriorityQueue,
        traits::{NiceF64, Weight},
    },
    geometry::traits::NormedSpace,
};

pub trait Graph {
    type Vertex;
    fn neighbors(&self, vertex: Self::Vertex) -> impl Iterator<Item = Self::Vertex>;
    fn dijkstra_with<W: Weight>(
        &self,
        start: Self::Vertex,
        end: Self::Vertex,
        cost_fn: impl Fn(Self::Vertex, Self::Vertex) -> W,
    ) -> Option<(Vec<Self::Vertex>, W)>
    where
        Self::Vertex: Hash + Eq + Clone,
    {
        let mut ancestor = HashMap::new();
        let mut queue = PriorityQueue::default();
        ancestor.insert(start.clone(), start.clone());
        queue.push(W::ZERO, start);
        loop {
            let Some((weight, vertex)) = queue.pop_min() else {return None;};
        }
    }
    fn a_star(&self, start: Self::Vertex, end: Self::Vertex) -> Option<(Vec<Self::Vertex>, f64)>
    where
        Self::Vertex: NormedSpace + Hash + Eq,
    {
        self.dijkstra_with(start, end, |a, b| {
            NiceF64::new(a.distance(b) + b.distance(end) - a.distance(end))
        })
        .map(|(path, weight)| (path, *weight))
    }
}

pub trait WeightedGraph: Graph {
    type Weight: Weight;
    fn weight_between(&self, a: Self::Vertex, b: Self::Vertex) -> Self::Weight;
    fn dijkstra(
        &self,
        start: Self::Vertex,
        end: Self::Vertex,
    ) -> Option<(Vec<Self::Vertex>, Self::Weight)>
    where
        Self::Vertex: Hash + Eq + Clone,
    {
        self.dijkstra_with(start, end, |a, b| self.weight_between(a, b))
    }
}
