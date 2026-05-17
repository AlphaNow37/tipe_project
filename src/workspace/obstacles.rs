use crate::datastructures::r_tree::RTreeLeaf;
use crate::triangulations::triangulation::Triangulation;
use crate::utils::numbers::UsizeExt;
use crate::workspace::cartesians::{DiscreteCartesianTopology, DiscreteSegment};
use crate::{
    datastructures::r_tree::RTree,
    geometry::{shapes::Segment, VecN},
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

impl<const N: usize, D: Length<N>, T: RTreeLeaf<N>> ObstaclesEnv<CartesianTopology<N, D>>
    for RTree<N, T>
{
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

impl<'a, D: Length<2>> ObstaclesEnv<DiscreteCartesianTopology<'a, 2, D>> for Triangulation {
    fn collide_vertex(&self, _: usize) -> bool {
        false
    }
    fn collide_segment(&self, s: DiscreteSegment) -> bool {
        // 1: find a tri having s[0] as a vertex, and where the opposite segment crosses s
        let pos_start = self.vertex_poss[s.0];
        let pos_end = self.vertex_poss[s.1];
        let seg = Segment {
            start: pos_start,
            end: pos_end,
        };

        let (mut from_tri, mut to_tri) = 'a: {
            for i in &self.vertex_to_adj_tris[s.0] {
                // 1.1: find the candidate opposing edge
                let k = 'c: {
                    for k in 0..3 {
                        if self.triangles[*i][k].verts[0] == s.0 {
                            break 'c k;
                        }
                    }
                    unreachable!("Malformed vertex_to_adj_tris");
                };
                let kopp = k.add_rem(1, 3);
                let opp_verts = self.triangles[*i][kopp].verts;
                if opp_verts.contains(&s.1) {
                    return false;
                }
                let Some(j) = self.triangles[*i][kopp].other_tri else {
                    continue;
                };
                // 1.2: checks if there is a collision
                let opp_poss = opp_verts.map(|u| self.vertex_poss[u]);
                if seg.intersect_segment(Segment {
                    start: opp_poss[0],
                    end: opp_poss[1],
                }) {
                    break 'a (*i, j);
                }
            }
            return true;
        };

        // 2: continue among the line until finding s[1], or not being able to continue
        'a: loop {
            for k in 0..3 {
                let opp_verts = self.triangles[to_tri][k].verts;
                if opp_verts.contains(&s.1) {
                    return false;
                }
                let Some(j) = self.triangles[to_tri][k].other_tri else {
                    continue;
                };
                if j == from_tri {
                    continue;
                }
                let opp_poss = opp_verts.map(|u| self.vertex_poss[u]);
                if seg.intersect_segment(Segment {
                    start: opp_poss[0],
                    end: opp_poss[1],
                }) {
                    (from_tri, to_tri) = (to_tri, j);
                    continue 'a;
                }
            }
            return true;
        }
    }
}
