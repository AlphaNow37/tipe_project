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

pub trait ObstaclesEnv<W: WorkspaceTopology> {
    /// Retourne true ssi a est dans les obstacles
    fn collide_vertex(&self, a: W::Vertex) -> bool;

    /// Retourne true ssi il existe s intersecte les obstacles
    fn collide_segment(&self, s: W::Segment) -> bool;
}

impl<const N: usize, D: Length<N>> ObstaclesEnv<CartesianTopology<N, D>> for RTree<N, Cube<N>> {
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
    fn visible_recurse(&self, s: W::Segment, nbr_rec: usize) -> bool {
        if nbr_rec == 0 {
            true
        } else {
            let (left, right) = self.workspace.split(s, 0.5);
            (!self.collide_vertex(self.workspace.segment_start(right)))
                && self.visible_recurse(left, nbr_rec - 1)
                && self.visible_recurse(right, nbr_rec - 1)
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
            return false;
        }
        let dist = self.workspace.length(s);
        let nb_recurses = (dist / self.visible_resolution).log2().ceil().max(0.) as usize;
        !self.visible_recurse(s, nb_recurses)
    }
}
