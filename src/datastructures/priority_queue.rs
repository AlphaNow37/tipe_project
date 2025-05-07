use crate::macros::make_trait_alias;

use crate::geometry::traits::Zero;
use std::ops::Add;

make_trait_alias!(Weight = [Sized + Zero + Add<Output=Self> + Ord] {});

fn left_child(i: usize) -> usize {
    i * 2 + 1
}
fn right_child(i: usize) -> usize {
    i * 2 + 2
}
fn parent(i: usize) -> usize {
    (i - 1) / 2
}

/// Low priority come first
pub struct PriorityQueue<W: Weight, T> {
    heap: Vec<(W, T)>,
}
impl<W: Weight, T> Default for PriorityQueue<W, T> {
    fn default() -> Self {
        Self { heap: Vec::new() }
    }
}
impl<W: Weight, T> PriorityQueue<W, T> {
    pub fn push(&mut self, weight: W, value: T) {
        let mut i = self.heap.len();
        self.heap.push((weight, value));
        while i > 0 && self.heap[i].0 < self.heap[parent(i)].0 {
            self.heap.swap(i, parent(i));
            i = parent(i);
        }
    }
    pub fn pop_min(&mut self) -> Option<(W, T)> {
        let n = self.heap.len();
        if n > 1 {
            let m = n - 1;
            self.heap.swap(0, m);
            let mut i = 0;
            loop {
                let l = left_child(i);
                let r = right_child(i);
                let mut j = i;
                if l < m && self.heap[l].0 < self.heap[j].0 {
                    j = l;
                }
                if r < m && self.heap[r].0 < self.heap[j].0 {
                    j = r;
                }
                if i == j {
                    break;
                } else {
                    self.heap.swap(i, j);
                    i = j
                }
            }
        }

        self.heap.pop()
    }
}

#[cfg(test)]
mod test {
    #[test]
    fn test_queue() {
        use super::PriorityQueue;

        let mut q = PriorityQueue::default();
        assert!(q.pop_min().is_none());
        for i in [0, 5, 2, 1, 4, 3, 6] {
            q.push(i, i);
        }
        for i in 0..7 {
            assert_eq!(q.pop_min(), Some((i, i)));
        }
        assert!(q.pop_min().is_none());
    }
}
