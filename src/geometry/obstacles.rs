use crate::{
    datastructures::r_tree::RTree,
    geometry::{
        shapes::{Cube, Segment},
        space::Space,
        VecN,
    },
};

pub trait ObstaclesEnv<S: Space> {
    /// Retourne true ssi a est dans les obstacles
    fn contains(&self, a: S) -> bool;

    /// Retourne true ssi b est visible depuis a
    fn visible(&self, a: S, b: S) -> bool;
}

impl<const N: usize> ObstaclesEnv<VecN<N, f64>> for RTree<N, Cube<N>> {
    fn contains(&self, a: VecN<N, f64>) -> bool {
        self.contains_point(a)
    }
    fn visible(&self, a: VecN<N, f64>, b: VecN<N, f64>) -> bool {
        self.intersect_segment(Segment { start: a, end: b })
    }
}

pub struct MapperObstacles<O, S1, S2> {
    sub_obstacles: O,
    map_position: Box<dyn Fn(S1) -> S2>,
    visible_resolution: f64,
}
impl<O: ObstaclesEnv<S2>, S1: Space, S2: Space> MapperObstacles<O, S1, S2> {
    fn visible_recurse(&self, a: S1, b: S1, nbr_rec: usize) -> bool {
        if nbr_rec == 0 {
            true
        } else {
            let mid = a.lerp(b, 0.5);
            (!self.contains(mid))
                && self.visible_recurse(a, mid, nbr_rec - 1)
                && self.visible_recurse(mid, b, nbr_rec - 1)
        }
    }
}
impl<O: ObstaclesEnv<S2>, S1: Space, S2: Space> ObstaclesEnv<S1> for MapperObstacles<O, S1, S2> {
    fn contains(&self, a: S1) -> bool {
        self.sub_obstacles.contains((self.map_position)(a))
    }
    fn visible(&self, a: S1, b: S1) -> bool {
        if self.contains(a) || self.contains(b) {
            return false;
        }
        let dist = a.distance(b);
        let nb_recurses = (dist / self.visible_resolution).log2().ceil().max(0.) as usize;
        self.visible_recurse(a, b, nb_recurses)
    }
}
