use std::{collections::HashMap, hash::Hash};

use crate::graphs::{Graph, IterableGraph};

/// A tree (or maybe forest) where each node know his parent
#[derive(Clone, Debug)]
pub struct ParentTree<Id> {
    parent: HashMap<Id, Id>,
}
impl<Id: Hash + Eq + Copy> ParentTree<Id> {
    pub fn new() -> Self {
        Self {
            parent: HashMap::new(),
        }
    }
    pub fn set_parent(&mut self, id: Id, parent: Id) {
        self.parent.insert(id, parent);
    }
    pub fn get_parent(&self, id: Id) -> Option<Id> {
        self.parent.get(&id).copied()
    }
    pub fn nb_links(&self) -> usize {
        self.parent.len()
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

/// A tree (or maybe forest) where each node know his parent and his children
#[derive(Clone, Debug)]
pub struct Tree<Id> {
    parent_children: HashMap<Id, (Option<Id>, Vec<Id>)>,
    nb_links: usize,
}
impl<Id: Hash + Eq + Copy> Tree<Id> {
    pub fn new() -> Self {
        Self {
            parent_children: HashMap::new(),
            nb_links: 0,
        }
    }
    pub fn set_parent(&mut self, child: Id, parent: Id) {
        let old_parent = self
            .parent_children
            .entry(child)
            .or_default()
            .0
            .replace(parent);

        if let Some(p) = old_parent {
            self.parent_children
                .get_mut(&p)
                .expect("There should be a parent")
                .1
                .retain(|e| *e != child);
        }

        self.parent_children
            .entry(parent)
            .or_default()
            .1
            .push(child);

        self.nb_links += 1
    }
    pub fn get_parent(&self, id: Id) -> Option<Id> {
        self.parent_children.get(&id).and_then(|(p, _)| *p)
    }
    pub fn nb_links(&self) -> usize {
        self.parent_children.len()
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
impl<Id: Hash + Eq + Copy> Graph<Id> for Tree<Id> {
    fn neighbors(&self, vertex: Id) -> impl Iterator<Item = Id> {
        self.parent_children
            .get(&vertex)
            .map(|(parent, children)| parent.iter().chain(children.iter()))
            .unwrap_or(None.iter().chain([].iter()))
            .copied()
    }
}
impl<Id: Hash + Eq + Copy> IterableGraph<Id> for Tree<Id> {
    fn iter(&self) -> impl Iterator<Item = Id> {
        self.parent_children.keys().copied()
    }
}
