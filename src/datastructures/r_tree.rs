use crate::geometry::shapes::{Cube, Segment};
use crate::geometry::VecN;
use crate::utils::numbers::NotNanF64;

/// Taille avant laquelle les feuilles sont aplaties directement
const FINAL_SIZE_THRESHOLD: usize = 8;

pub trait RTreeleaf<const N: usize>: Clone {
    fn bounding_box(&self) -> Cube<N>;
}

impl<const N: usize> RTreeleaf<N> for Cube<N> {
    fn bounding_box(&self) -> Cube<N> {
        *self
    }
}

#[derive(Debug, Clone)]
pub enum RTree<const N: usize, T> {
    Node {
        bounding_box: Cube<N>,
        children: Vec<RTree<N, T>>,
    },
    Leaf(T),
}

impl<const N: usize, T: RTreeleaf<N>> RTree<N, T> {
    fn node_from_children(children: Vec<Self>) -> Self {
        Self::Node {
            bounding_box: children
                .iter()
                .map(|c| c.bounding_box())
                .reduce(Cube::join)
                .expect("Children list should not be empty"),
            children,
        }
    }

    fn load_from_dimension(objs: &mut [T], d: usize, out: &mut Vec<Self>) {
        if d == N {
            return out.push(Self::bulk_load(objs));
        }
        let mid = objs.len() / 2;
        objs.select_nth_unstable_by_key(mid, |t| {
            NotNanF64::new_debug_checked(t.bounding_box().start[d])
        });
        Self::load_from_dimension(&mut objs[..mid], d + 1, out);
        Self::load_from_dimension(&mut objs[mid..], d + 1, out);
    }

    /// Utilise l'algo STR (sort-tile-recursive)
    /// On divise le groupe en 2 sur chaque dimension
    pub fn bulk_load(objs: &mut [T]) -> Self {
        assert!(objs.len() > 0);
        if objs.len() == 1 {
            return Self::Leaf(objs[0].clone());
        }
        if objs.len() < (1 << N).max(FINAL_SIZE_THRESHOLD) {
            Self::node_from_children(objs.iter().map(|o| Self::Leaf(o.clone())).collect())
        } else {
            let mut out = Vec::new();
            Self::load_from_dimension(objs, 0, &mut out);
            Self::node_from_children(out)
        }
    }

    pub fn bounding_box(&self) -> Cube<N> {
        match self {
            Self::Leaf(t) => t.bounding_box(),
            Self::Node { bounding_box, .. } => *bounding_box,
        }
    }
    pub fn contains_point(&self, pt: VecN<N, f64>) -> bool {
        match self {
            Self::Leaf(t) => t.bounding_box().contains_point(pt),
            Self::Node {
                bounding_box,
                children,
            } => bounding_box.contains_point(pt) && children.iter().any(|child| child.contains_point(pt)),
        }
    }
    pub fn intersect_cube(&self, cube: Cube<N>) -> bool {
        match self {
            Self::Leaf(t) => t.bounding_box().intersect_cube(cube),
            Self::Node {
                bounding_box,
                children,
            } => {
                bounding_box.intersect_cube(cube)
                    && children.iter().any(|child| child.intersect_cube(cube))
            }
        }
    }
    pub fn intersect_segment(&self, segment: Segment<N>) -> bool {
        match self {
            Self::Leaf(t) => t.bounding_box().intersect_segment(segment),
            Self::Node {
                bounding_box,
                children,
            } => {
                bounding_box.intersect_segment(segment)
                    && children.iter().any(|child| child.intersect_segment(segment))
            }
        }
    }
}
