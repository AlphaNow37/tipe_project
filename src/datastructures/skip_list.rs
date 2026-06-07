use rand::{rng, Rng};
use std::cmp::Ordering;
use std::fmt::{Debug, Formatter};

/// Ce fichier implémente des skip-list, dont les éléments sont des intervalles
/// Ces skip lists peuvent notamment être coupées et fusionnées.
/// Le tout est mis dans une arena pour limiter les allocations et pour plaire au compilo rust
/// L'objectif est principalement d'avoir une liste chaînée avec random access (via les curseurs)

const SKIP_LIST_HEIGHT: usize = 1;
const FAKE_NODE: usize = usize::MAX - 1;
const P: f64 = 0.5;

fn random_height() -> usize {
    let mut rng = rng();
    let mut i = 1;
    while rng.random_bool(P) && i < SKIP_LIST_HEIGHT {
        i += 1;
    }
    debug_assert!(1 <= i && i <= SKIP_LIST_HEIGHT);
    i
}

pub trait Interval {
    type Value;
    fn cmp_value(&self, value: &Self::Value) -> Ordering;
}

#[derive(Debug, Clone)]
struct SkipListNode<I> {
    id: usize,
    interval: I,
    height: usize,
    succs: [usize; SKIP_LIST_HEIGHT],
    preds: [usize; SKIP_LIST_HEIGHT],
}
impl<I> SkipListNode<I> {
    fn use_as_preds(&self, preds: &mut [usize; SKIP_LIST_HEIGHT]) {
        for i in 0..self.height {
            preds[i] = self.id;
        }
    }
    fn put_preds(&self, preds: &mut [usize; SKIP_LIST_HEIGHT]) {
        for i in 0..self.height {
            preds[i] = self.preds[i];
        }
    }

    fn use_as_succs(&self, succs: &mut [usize; SKIP_LIST_HEIGHT]) {
        for i in 0..self.height {
            succs[i] = self.id;
        }
    }
    fn put_succs(&self, succs: &mut [usize; SKIP_LIST_HEIGHT]) {
        for i in 0..self.height {
            succs[i] = self.succs[i];
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SkipListAccess {
    first_nodes: [usize; SKIP_LIST_HEIGHT],
}
impl SkipListAccess {
    pub fn new() -> Self {
        Self {
            first_nodes: [FAKE_NODE; SKIP_LIST_HEIGHT]
        }
    }
}

#[derive(Clone, Debug)]
pub struct IntervalSkipLists<I> {
    nodes: Vec<SkipListNode<I>>,
    free_idxs: Vec<usize>,
}

impl<I> IntervalSkipLists<I> {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            free_idxs: Vec::new(),
        }
    }
}
impl<I> IntervalSkipLists<I> {
    fn add_node(
        &mut self,
        access: &mut SkipListAccess,
        interval: I,
        preds: [usize; SKIP_LIST_HEIGHT],
        succs: [usize; SKIP_LIST_HEIGHT],
    ) -> usize {
        let (node, id) = if self.free_idxs.len() == 0 {
            let id = self.nodes.len();
            self.nodes.push(SkipListNode {
                height: 0,
                interval,
                preds,
                succs,
                id,
            });
            (&mut self.nodes[id], id)
        } else {
            let id = self.free_idxs.pop().unwrap();
            let node = &mut self.nodes[id];
            node.interval = interval;
            (node, id)
        };
        node.height = random_height();
        node.succs = succs;
        node.preds = preds;
        for i in 0..node.height {
            let p = self.nodes[id].preds[i];
            let s = self.nodes[id].succs[i];
            if p == FAKE_NODE {
                access.first_nodes[i] = id;
            } else {
                debug_assert!(self.nodes[p].height > i);
                self.nodes[p].succs[i] = id;
            }
            if s != FAKE_NODE {
                debug_assert!(self.nodes[s].height > i);
                self.nodes[s].preds[i] = id;
            }
        }
        id
    }
    fn remove_node(&mut self, access: &mut SkipListAccess, id: usize) {
        self.free_idxs.push(id);
        for i in 0..self.nodes[id].height {
            let p = self.nodes[id].preds[i];
            let s = self.nodes[id].succs[i];
            if p == FAKE_NODE {
                access.first_nodes[i] = s;
            } else {
                debug_assert!(self.nodes[p].height > i);
                self.nodes[p].succs[i] = s;
            }
            if s != FAKE_NODE {
                debug_assert!(self.nodes[s].height > i);
                self.nodes[s].preds[i] = p;
            }
        }
    }
    pub fn split_between(&mut self, access: &mut SkipListAccess, preds: [usize; SKIP_LIST_HEIGHT], succs: [usize; SKIP_LIST_HEIGHT]) -> SkipListAccess {
        for i in 0..SKIP_LIST_HEIGHT {
            let s = succs[i];
            let p = preds[i];
            if p == FAKE_NODE {
                access.first_nodes[i] = FAKE_NODE;
            } else {
                debug_assert!(self.nodes[p].height > i);
                self.nodes[p].succs[i] = FAKE_NODE;
            }
            if s != FAKE_NODE {
                debug_assert!(self.nodes[s].height > i);
                self.nodes[s].preds[i] = FAKE_NODE;
            }
        }
        SkipListAccess {first_nodes: succs}
    }
    /// intervals must be sorted
    pub fn bulk_load(&mut self, intervals: Vec<I>) -> SkipListAccess {
        let mut preds = [FAKE_NODE; SKIP_LIST_HEIGHT];
        let mut access = SkipListAccess::new();
        for interval in intervals {
            let id = self.add_node(&mut access, interval, preds, [FAKE_NODE; SKIP_LIST_HEIGHT]);
            self.nodes[id].use_as_preds(&mut preds);
        }
        access
    }
}
impl<I: Interval> IntervalSkipLists<I> {
    pub fn cursor<'a>(&'a mut self, access: &'a mut SkipListAccess, value: I::Value) -> Cursor<'a, I> {
        // Invariant:
        // - prec, succs forment un curseur entre deux intervalles
        // - Les valeurs avant ce curseur sont de valeur strictement inférieure
        let mut curr_i = SKIP_LIST_HEIGHT - 1;
        let mut preds = [FAKE_NODE; SKIP_LIST_HEIGHT];
        let mut succs = access.first_nodes;
        loop {
            let s_id = succs[curr_i];
            if s_id == FAKE_NODE {
                if curr_i == 0 {
                    debug_assert!(succs == [FAKE_NODE; SKIP_LIST_HEIGHT]);
                    return Cursor {
                        idx: None,
                        list: self,
                        preds,
                        succs: [FAKE_NODE; SKIP_LIST_HEIGHT],
                        access
                    };
                }
                curr_i -= 1;
                continue;
            }
            let node = &self.nodes[s_id];
            debug_assert!(node.height > curr_i);
            match node.interval.cmp_value(&value) {
                Ordering::Equal => {
                    if curr_i == 0 {
                        node.put_succs(&mut succs);
                        return Cursor {
                            idx: Some(s_id),
                            succs,
                            preds,
                            list: self,
                            access,
                        }
                    } else {
                        curr_i -= 1;
                    }
                }
                Ordering::Greater => {
                    node.use_as_preds(&mut preds);
                    node.put_succs(&mut succs);
                }
                Ordering::Less => {
                    if curr_i == 0 {
                        return Cursor {
                            idx: None,
                            succs,
                            preds,
                            list: self,
                            access,
                        }
                    } else {
                        curr_i -= 1;
                    }
                }
            }
        }
    }
    pub fn cursor_left<'a>(&'a mut self, access: &'a mut SkipListAccess) -> Cursor<'a, I> {
        Cursor {
            idx: None,
            succs: access.first_nodes,
            preds: [FAKE_NODE; SKIP_LIST_HEIGHT],
            access,
            list: self,
        }
    }
}

#[derive(Debug)]
pub struct Cursor<'a, I: Interval> {
    idx: Option<usize>,
    list: &'a mut IntervalSkipLists<I>,
    access: &'a mut SkipListAccess,
    preds: [usize; SKIP_LIST_HEIGHT],
    succs: [usize; SKIP_LIST_HEIGHT],
}
impl<'a, I: Interval> Cursor<'a, I> {
    pub fn get_interval(&self) -> Option<&I> {
        self.idx.map(|id| &self.list.nodes[id].interval)
    }
    pub fn get_interval_mut(&mut self) -> Option<&mut I> {
        self.idx.map(|id| &mut self.list.nodes[id].interval)
    }
    pub fn remove_interval(&mut self) {
        let id = self.idx.expect("Expected an interval to remove");
        #[cfg(debug_assertions)]
        {
            let node = &self.list.nodes[id];
            for i in 0..node.height {
                debug_assert!(node.preds[i] == self.preds[i]);
                debug_assert!(node.succs[i] == self.succs[i]);
            }
        };
        self.list.remove_node(self.access, id);
        self.idx = None;
    }
    pub fn push_interval(&mut self, interval: I) {
        assert_eq!(self.idx, None);
        let id = self.list.add_node(self.access, interval, self.preds, self.succs);
        self.idx = Some(id);
    }
    pub fn move_right(&mut self) {
        match self.idx {
            None => {
                let id = self.succs[0];
                if id == FAKE_NODE {
                    panic!("Can't move right");
                }
                self.idx = Some(id);
                self.list.nodes[id].put_succs(&mut self.succs);
            }
            Some(id) => {
                self.idx = None;
                self.list.nodes[id].use_as_preds(&mut self.preds);
            }
        }
    }
    pub fn move_left(&mut self) {
        match self.idx {
            None => {
                let id = self.preds[0];
                if id == FAKE_NODE {
                    panic!("Can't move left");
                }
                self.idx = Some(id);
                self.list.nodes[id].put_preds(&mut self.preds);
            }
            Some(id) => {
                self.idx = None;
                self.list.nodes[id].use_as_succs(&mut self.succs);
            }
        }
    }
    pub fn is_full_left(&self) -> bool {
        self.idx == None && self.preds[0] == FAKE_NODE
    }
    pub fn is_full_right(&self) -> bool {
        self.idx == None && self.succs[0] == FAKE_NODE
    }
    pub fn split_list(&mut self) -> SkipListAccess {
        debug_assert_eq!(self.idx, None);
        self.list.split_between(
            self.access,
            self.preds,
            self.succs
        )
    }
    pub fn concat_list(&mut self, other: SkipListAccess) {
        debug_assert!(self.is_full_right());
        for i in 0..SKIP_LIST_HEIGHT {
            let p = self.preds[i];
            let s = other.first_nodes[i];
            if s != FAKE_NODE {
                debug_assert!(self.list.nodes[s].height > i);
                self.list.nodes[s].preds[i] = p;
            }
            if p != FAKE_NODE {
                debug_assert!(self.list.nodes[p].height > i);
                self.list.nodes[p].succs[i] = s;
            } else {
                self.access.first_nodes[i] = s;
            }
        }
        self.succs = other.first_nodes;
    }
}
