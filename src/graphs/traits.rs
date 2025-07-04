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

<<<<<<< HEAD
<<<<<<< HEAD:src/graphs/graphs.rs
/// A graph represented using adjacency lists, and vertices are integers
=======
>>>>>>> cdf7a4c (Adding documentation)
#[derive(Default, Clone, Debug)]
pub struct LinkGraph {
    nexts: Vec<Vec<usize>>,
}
impl LinkGraph {
    pub fn add_new_link(&mut self, start: usize, end: usize) {
        let m = start.max(end);
        let n = self.nexts.len();
        if m >= n {
            self.nexts.extend((0..m - n + 1).map(|_| Vec::new()));
        }
        self.nexts[start].push(end);
    }
    pub fn add_link(&mut self, start: usize, end: usize) {
        let m = start.max(end);
        let n = self.nexts.len();
        if m >= n {
            self.nexts.extend((0..m - n + 1).map(|_| Vec::new()));
        }
        if !self.nexts[start].contains(&end) {
            self.nexts[start].push(end);
        }
    }
    pub fn remove_link(&mut self, start: usize, end: usize) {
        if start >= self.nexts.len() {
            return;
        }
        let row = &mut self.nexts[start];
        let mut i = 0;
        while i < row.len() {
            if row[i] == end {
                row.swap_remove(i);
            } else {
                i += 1
            }
        }
    }
}
impl Graph<usize> for LinkGraph {
    fn neighbors(&self, vertex: usize) -> impl Iterator<Item = usize> {
        self.nexts[vertex].iter().cloned()
    }
}
impl IterableGraph<usize> for LinkGraph {
    fn iter(&self) -> impl Iterator<Item = usize> {
        0..self.nexts.len()
    }
}

<<<<<<< HEAD
/// A graph represented using adjacency lists, stored in a hashmap
=======
>>>>>>> cdf7a4c (Adding documentation)
#[derive(Default, Clone, Debug)]
pub struct MapGraph<V> {
    nexts: HashMap<V, Vec<V>>,
}
impl<V: Hash + Eq + Copy> MapGraph<V> {
    pub fn add_new_link(&mut self, start: V, end: V) {
        self.nexts.entry(start).or_default().push(end);
    }
    pub fn add_link(&mut self, start: V, end: V) {
        if !self.nexts[&start].contains(&end) {
            self.nexts.entry(start).or_default().push(end);
        }
    }
    pub fn remove_link(&mut self, start: V, end: V) {
        let row = &mut self.nexts.entry(start).or_default();
        let mut i = 0;
        while i < row.len() {
            if row[i] == end {
                row.swap_remove(i);
            } else {
                i += 1
            }
        }
    }
    pub fn set_neighbors(&mut self, start: V, ends: Vec<V>) {
        self.nexts.insert(start, ends);
    }
    pub fn from_fn(verteces: impl Iterator<Item = V>, mut f: impl FnMut(V) -> Vec<V>) -> Self {
        Self {
            nexts: HashMap::from_iter(verteces.map(move |k| (k, f(k)))),
        }
    }
}
impl<V: Clone + Eq + Hash> Graph<V> for MapGraph<V> {
    fn neighbors(&self, vertex: V) -> impl Iterator<Item = V> {
        self.nexts[&vertex].iter().cloned()
    }
}
impl<V: Clone + Eq + Hash> IterableGraph<V> for MapGraph<V> {
    fn iter(&self) -> impl Iterator<Item = V> {
        self.nexts.keys().cloned()
    }
<<<<<<< HEAD
=======
=======
}

>>>>>>> cdf7a4c (Adding documentation)
#[test]
fn test_dijkstra() {
    use crate::geometry::VecN;

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

    dbg!(g.a_star_with(0, 5, |i| poss[i]));
>>>>>>> c9b1c2c (updated graph folder structure):src/graphs/traits.rs
}
