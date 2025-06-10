use crate::graphs::{Graph, IterableGraph};
use std::collections::HashMap;
use std::hash::Hash;

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
}


