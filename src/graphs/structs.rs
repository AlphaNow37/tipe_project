use crate::graphs::{Graph, IterableGraph};
use std::collections::HashMap;
use std::hash::Hash;
use std::sync::{Arc, Mutex};

/// A graph using integers as vertices and an adjacency list
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

/// A graph using an adjacency list stored in a hashmap
#[derive(Clone, Debug)]
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
impl<V> Default for MapGraph<V> {
    fn default() -> Self {
        Self {
            nexts: HashMap::default(),
        }
    }
}

/// A graph based on a function vertex->neighbors
#[derive(Default, Clone, Debug)]
pub struct FuncGraph<F> {
    nexts: F,
}
impl<F> FuncGraph<F> {
    pub fn new(f: F) -> Self {
        Self { nexts: f }
    }
}
impl<V: Hash + Eq + Copy, F: Fn(V) -> I, I: IntoIterator<Item = V>> Graph<V> for FuncGraph<F> {
    fn neighbors(&self, vertex: V) -> impl Iterator<Item = V> {
        (self.nexts)(vertex).into_iter()
    }
}

/// A graph based on a function, but with a cache
#[derive(Default, Debug)]
pub struct CachedFuncGraph<F, V> {
    cache_nexts: Mutex<(HashMap<V, Vec<V>>, F)>,
}
impl<V: Hash + Eq + Copy, F: FnMut(V) -> Vec<V>> CachedFuncGraph<F, V> {
    pub fn new(f: F) -> Self {
        Self {
            cache_nexts: Mutex::new((HashMap::default(), f)),
        }
    }
}
impl<V: Hash + Eq + Copy, F: FnMut(V) -> Vec<V>> Graph<V> for CachedFuncGraph<F, V> {
    fn neighbors(&self, vertex: V) -> impl Iterator<Item = V> {
        let mut lock = self.cache_nexts.lock().expect("Err while taking the lock");
        if lock.0.contains_key(&vertex) {
            lock.0.get(&vertex).unwrap().clone().into_iter()
        } else {
            let res = (lock.1)(vertex);
            lock.0.insert(vertex, res.clone());
            res.into_iter()
        }
        // (self.nexts)(vertex).into_iter()
    }
}

/// Filtre les arêtes selon la fonction filter
#[derive(Clone)]
pub struct SubGraph<'a, V, G> {
    pub graph: G,
    pub filter: Arc<dyn Fn(&V, &V)->bool + 'a>,
}
impl<'a, V: Copy, G: Graph<V>> Graph<V> for SubGraph<'a, V, G> {
    fn neighbors(&self, vertex: V) -> impl Iterator<Item = V> {
        self.graph.neighbors(vertex).filter(move |v| (self.filter)(&vertex, v))
    }
}
impl<'a, V: Copy, G: IterableGraph<V>> IterableGraph<V> for SubGraph<'a, V, G> {
    fn iter(&self) -> impl Iterator<Item = V> {
        self.graph.iter()
    }
}
