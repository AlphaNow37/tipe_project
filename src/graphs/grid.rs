use crate::geometry::VecN;
use crate::graphs::{Graph, IterableGraph};

fn calculate_offsets<const N: usize>(sizes: [usize; N]) -> ([usize; N], usize) {
    let mut offsets = [0; N];
    let mut size = 1;
    for i in 0..N {
        offsets[i] = size;
        size *= sizes[i];
    }
    (offsets, size)
}

/// A N-dimensions grid graph
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Grid<const N: usize> {
    pub sizes: [usize; N],
    offsets: [usize; N],
    pub size: usize,
}
impl<const N: usize> Grid<N> {
    pub fn new(sizes: [usize; N]) -> Self {
        let (offsets, size) = calculate_offsets(sizes);
        Self {
            sizes,
            offsets,
            size,
        }
    }
    pub fn ith_coord(&self, index: usize, i: usize) -> usize {
        (index / self.offsets[i]) % self.sizes[i]
    }
    pub fn coords(&self, index: usize) -> VecN<N, usize> {
        VecN::from_fn(|i| self.ith_coord(index, i))
    }
    pub fn index(&self, coords: VecN<N, usize>) -> usize {
        coords.dot(VecN(self.offsets))
    }
    pub fn iter_cube(&self, start: VecN<N, usize>, end: VecN<N, usize>) -> impl Iterator<Item=usize> {
        let mut curr = start;
        let mut finished = false;
        let grid = *self;
        std::iter::from_fn(move || {
            if finished {
                return None;
            }
            let v = curr;
            let mut i = 0;
            while i < N && curr[i] == end[i] {
                curr[i] = start[i];
                i += 1;
            }
            if i == N {
                finished = true;
            } else {
                curr[i] += 1;
            }
            Some(grid.index(v))
        })
    }
}
impl<const N: usize> Graph<usize> for Grid<N> {
    fn neighbors(&self, vertex: usize) -> impl Iterator<Item = usize> {
        let negs = (0..N).filter_map(move |i| {
            (self.ith_coord(vertex, i) != 0).then(|| vertex - self.offsets[i])
        });
        let poss = (0..N).filter_map(move |i| {
            (self.ith_coord(vertex, i) + 1 != self.sizes[i]).then(|| vertex + self.offsets[i])
        });
        negs.chain(poss)
    }
}
impl<const N: usize> IterableGraph<usize> for Grid<N> {
    fn iter(&self) -> impl Iterator<Item = usize> {
        0..self.size
    }
}
