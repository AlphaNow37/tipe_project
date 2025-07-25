use crate::{
    datastructures::r_tree::RTree,
    geometry::{
        shapes::{Cube, Segment},
        workspace::WorkspaceTopology,
        VecN,
    },
};

pub trait ObstaclesEnv<V> {
    /// Retourne true ssi a est dans les obstacles
    fn contains(&self, a: V) -> bool;

    /// Retourne true ssi b est visible depuis a
    fn visible(&self, a: V, b: V) -> bool;
}

impl<const N: usize> ObstaclesEnv<VecN<N, f64>> for RTree<N, Cube<N>> {
    fn contains(&self, a: VecN<N, f64>) -> bool {
        self.contains_point(a)
    }
    fn visible(&self, a: VecN<N, f64>, b: VecN<N, f64>) -> bool {
        self.intersect_segment(Segment { start: a, end: b })
    }
}

pub struct ObstaclesApprox<'a, W: WorkspaceTopology> {
    pub contains_func: Box<dyn Fn(W::Vertex) -> bool + 'a>,
    pub visible_resolution: f64,
    pub workspace: W,
}
impl<'a, W: WorkspaceTopology> ObstaclesApprox<'a, W> {
    fn visible_recurse(&self, a: W::Vertex, b: W::Vertex, nbr_rec: usize) -> bool {
        if nbr_rec == 0 {
            true
        } else {
            let mid = self.workspace.lerp(a, b, 0.5);
            (!self.contains(mid))
                && self.visible_recurse(a, mid, nbr_rec - 1)
                && self.visible_recurse(mid, b, nbr_rec - 1)
        }
    }
}
impl<'a, W: WorkspaceTopology> ObstaclesEnv<W::Vertex> for ObstaclesApprox<'a, W> {
    fn contains(&self, a: W::Vertex) -> bool {
        (self.contains_func)(a)
    }
    fn visible(&self, a: W::Vertex, b: W::Vertex) -> bool {
        if self.contains(a) || self.contains(b) {
            return false;
        }
        let dist = self.workspace.distance(a, b);
        let nb_recurses = (dist / self.visible_resolution).log2().ceil().max(0.) as usize;
        self.visible_recurse(a, b, nb_recurses)
    }
}
