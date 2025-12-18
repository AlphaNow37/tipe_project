use crate::{
    datastructures::r_tree::RTree,
    geometry::{
        shapes::{Cube, Segment},
        VecN,
    },
    workspace::{
        cartesians::{CartesianTopology, Length},
        workspace::WorkspaceTopology,
    },
};
use crate::datastructures::r_tree::RTreeLeaf;

pub trait ObstaclesEnv<W: WorkspaceTopology> {
    /// Retourne true ssi a est dans les obstacles
    fn collide_vertex(&self, a: W::Vertex) -> bool;

    /// Retourne true ssi il existe s intersecte les obstacles
    fn collide_segment(&self, s: W::Segment) -> bool;
}

impl<const N: usize, D: Length<N>, T: RTreeLeaf<N>> ObstaclesEnv<CartesianTopology<N, D>> for RTree<N, T> {
    fn collide_vertex(&self, a: VecN<N, f64>) -> bool {
        self.contains_point(a)
    }
    fn collide_segment(&self, s: Segment<N>) -> bool {
        self.intersect_segment(s)
    }
}

/// Une approximation utile lorsque on possède seulement une fonction qui teste l'appartenance
/// Exemple: bras robotique
pub struct ObstaclesApprox<'a, W: WorkspaceTopology> {
    pub contains_func: Box<dyn Fn(W::Vertex) -> bool + 'a>,
    pub visible_resolution: f64,
    pub workspace: W,
}
impl<'a, W: WorkspaceTopology> ObstaclesApprox<'a, W> {
    /// Applique l'algo visible-recurse pour déterminer si le segment n'entre en collisions avec aucun obstacle
    /// Commence par regarder le milieu du segment, puis traite les deux cotés séparements
    /// 0. <= left < right <= 1.
    fn visible_recurse(&self, s: W::Segment, left: f64, right: f64, resolution: f64) -> bool {
        debug_assert!(left < right);
        debug_assert!(resolution > 0.);
        if right - left < resolution {
            true
        } else {
            let mid = left.midpoint(right);
            (!self.collide_vertex(self.workspace.lerp(s, mid)))
                && self.visible_recurse(s, left, mid, resolution)
                && self.visible_recurse(s, mid, right, resolution)
        }
    }
}
impl<'a, W: WorkspaceTopology> ObstaclesEnv<W> for ObstaclesApprox<'a, W> {
    fn collide_vertex(&self, a: W::Vertex) -> bool {
        (self.contains_func)(a)
    }
    fn collide_segment(&self, s: W::Segment) -> bool {
        if self.collide_vertex(self.workspace.segment_start(s))
            || self.collide_vertex(self.workspace.segment_end(s))
        {
            return true;
        }
        let dist = self.workspace.length(s);
        !self.visible_recurse(s, 0., 1., self.visible_resolution / dist)
    }
}
