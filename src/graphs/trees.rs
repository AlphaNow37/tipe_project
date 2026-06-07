use std::{collections::HashMap, hash::Hash};

use crate::graphs::{Graph, IterableGraph};

/// A tree (or maybe forest) where each node know his parent
/// Info is a reference to the parent + information about the edge, such that the weight
#[derive(Clone, Debug)]
pub struct ParentTree<Id, Info> {
    parent: HashMap<Id, Info>,
}
impl<Id: Hash + Eq + Copy, Info: Copy + Into<Id>> ParentTree<Id, Info> {
    pub fn new() -> Self {
        Self {
            parent: HashMap::new(),
        }
    }
    pub fn set_parent(&mut self, id: Id, parent: Info) {
        self.parent.insert(id, parent);
    }
    pub fn get_parent(&self, id: Id) -> Option<Info> {
        self.parent.get(&id).copied()
    }
    pub fn nb_links(&self) -> usize {
        self.parent.len()
    }
    
    /// Remonte le chemin jusqu'à la racine
    pub fn path_to(&self, mut x: Id) -> Vec<Info> {
        let mut path = vec![];
        while let Some(parent) = self.get_parent(x) {
            path.push(parent);
            x = parent.into();
        }
        path.reverse();
        path
    }
}
impl<Id: Hash + Eq + Copy, Info: Copy> Graph<Id, Info> for ParentTree<Id, Info> {
    fn neighbors(&self, vertex: Id) -> impl Iterator<Item = Info> {
        self.parent
            .get(&vertex)
            .into_iter()
            .map(|parent| *parent)
    }
}
impl<Id: Hash + Eq + Copy, Info: Copy> IterableGraph<Id, Info> for ParentTree<Id, Info> {
    fn iter(&self) -> impl Iterator<Item = Id> {
        self.parent.keys().copied()
    }
}

/// A tree (or maybe forest) where each node know his parent and his children
#[derive(Clone, Debug)]
pub struct Tree<Id, Info> {
    parent_children: HashMap<Id, (Option<Info>, Vec<Id>)>,
    nb_links: usize,
}
impl<Id: Hash + Eq + Copy, Info: Copy + Into<Id>> Tree<Id, Info> {
    pub fn new() -> Self {
        Self {
            parent_children: HashMap::new(),
            nb_links: 0,
        }
    }
    pub fn set_parent(&mut self, child: Id, parent: Info) {
        let old_parent = self
            .parent_children
            .entry(child)
            .or_default()
            .0
            .replace(parent);

        if let Some(p) = old_parent {
            self.parent_children
                .get_mut(&p.into())
                .expect("There should be a parent")
                .1
                .retain(|e| *e != child);
        }

        self.parent_children
            .entry(parent.into())
            .or_default()
            .1
            .push(child);

        self.nb_links += 1
    }
    pub fn get_parent(&self, id: Id) -> Option<Info> {
        self.parent_children.get(&id)
            .and_then(|(p, _)| *p)
    }
    pub fn get_children(&self, id: Id) -> &[Id] {
        self.parent_children
            .get(&id)
            .map(|(_, children)| &**children)
            .unwrap_or(&[])
    }
    pub fn nb_links(&self) -> usize {
        self.parent_children.len()
    }
    /// Remonte le chemin jusqu'à la racine
    pub fn path_to(&self, mut x: Id) -> Vec<Info> {
        let mut path = vec![];
        while let Some(parent) = self.get_parent(x) {
            path.push(parent);
            x = parent.into();
        }
        path.reverse();
        path
    }
}
impl<Id: Hash + Eq + Copy, Info: Copy> Graph<Id, Info> for Tree<Id, Info> {
    fn neighbors(&self, vertex: Id) -> impl Iterator<Item = Info> {
        // self.parent_children
        //     .get(&vertex)
        //     .map(|(parent, children)| parent.iter().chain(children.iter()))
        //     .unwrap_or(None.iter().chain([].iter()))
        //     .copied()
        self.parent_children
            .get(&vertex)
            .into_iter()
            .filter_map(|(parent, _)| *parent)
    }
}
impl<Id: Hash + Eq + Copy, Info: Copy> IterableGraph<Id, Info> for Tree<Id, Info> {
    fn iter(&self) -> impl Iterator<Item = Id> {
        self.parent_children.keys().copied()
    }
}
