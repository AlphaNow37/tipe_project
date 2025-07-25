use std::{collections::HashMap, hash::Hash};

use crate::graphs::{Graph, IterableGraph};

#[derive(Clone, Debug)]
pub struct ParentTree<Id> {
    parent: HashMap<Id, Id>,
}
impl<Id: Hash + Eq + Copy> ParentTree<Id> {
    pub fn new() -> Self {
        Self {parent: HashMap::new()}
    }
    pub fn set_parent(&mut self, id: Id, parent: Id) {
        self.parent.insert(id, parent);
    }
    pub fn get_parent(&self, id: Id) -> Option<Id> {
        self.parent.get(&id).copied()
    }
    pub fn path_to(&self, mut x: Id) -> Vec<Id> {
        let mut path = vec![x];
        while let Some(p) = self.get_parent(x) {
            path.push(p);
            x = p;
        }
        path.reverse();
        path
    }
}
impl<Id: Hash + Eq + Copy> Graph<Id> for ParentTree<Id> {
    fn neighbors(&self, vertex: Id) -> impl Iterator<Item = Id> {
        self.parent.get(&vertex).copied().into_iter()
    }
}
impl<Id: Hash + Eq + Copy> IterableGraph<Id> for ParentTree<Id> {
    fn iter(&self) -> impl Iterator<Item = Id> {
        self.parent.keys().copied()
    }
}
