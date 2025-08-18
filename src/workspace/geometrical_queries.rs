use crate::{
    datastructures::bsp::Bsp,
    geometry::{shapes::Segment, VecN},
    workspace::{
        cartesians::{CartesianTopology, Length},
        workspace::WorkspaceTopology,
    },
};

/// Used to answer geometrical queries like nearest neighbors / R-nearest neighbors
pub trait GeometricalQueryDataStore<W: WorkspaceTopology> {
    /// Crée une structure vide
    fn new_store(workspace: W) -> Self;
    /// Ajoute un sommmet
    fn insert_vertex(&mut self, pt: W::Vertex);
    /// Applique f à tout les sommets à distance inférieure à radius de center
    fn foreach_r_neighbors(
        &self,
        center: W::Vertex,
        radius: f64,
        f: &mut impl FnMut(W::Segment, f64),
    );
    /// Retourne le sommet le plus proche (renvoie None s'il n'y en a pas)
    fn nearest_vertex(&self, pt: W::Vertex) -> Option<W::Segment>;
    /// Applique f à touts les sommets
    fn foreach_vertex(&self, f: &mut impl FnMut(W::Vertex));
}

/// L'implémentation naive
impl<W: WorkspaceTopology> GeometricalQueryDataStore<W> for (Vec<W::Vertex>, W) {
    fn new_store(workspace: W) -> Self {
        (Vec::new(), workspace)
    }
    fn insert_vertex(&mut self, pt: W::Vertex) {
        self.0.push(pt);
    }
    fn foreach_r_neighbors(
        &self,
        center: W::Vertex,
        radius: f64,
        f: &mut impl FnMut(W::Segment, f64),
    ) {
        for v in self.0.iter() {
            let segment = self.1.segment(center, *v);
            let distance = self.1.length(segment);
            if distance <= radius {
                f(segment, distance)
            }
        }
    }
    fn nearest_vertex(&self, pt: W::Vertex) -> Option<W::Segment> {
        let mut min_s = None;
        let mut min_dist = f64::INFINITY;
        for v in self.0.iter() {
            let segment = self.1.segment(pt, *v);
            let dist = self.1.length(segment);
            if dist < min_dist {
                min_dist = dist;
                min_s = Some(segment);
            }
        }
        min_s
    }
    fn foreach_vertex(&self, f: &mut impl FnMut(W::Vertex)) {
        for v in self.0.iter() {
            f(*v)
        }
    }
}

/// Implémentations optimisées
impl<D: Length<N>, const N: usize> GeometricalQueryDataStore<CartesianTopology<N, D>>
    for (Bsp<N>, CartesianTopology<N, D>)
{
    fn new_store(workspace: CartesianTopology<N, D>) -> Self {
        (Bsp::new_default_config(workspace.space), workspace)
    }
    fn insert_vertex(&mut self, pt: VecN<N, f64>) {
        self.0.insert(pt);
    }
    fn foreach_r_neighbors(
        &self,
        center: VecN<N, f64>,
        radius: f64,
        f: &mut impl FnMut(Segment<N>, f64),
    ) {
        self.0.foreach_r_neighborhood(
            radius,
            &|v| self.1.distance(center, v),
            &|c| self.1.distance_to_cube(center, c),
            &mut |end| f(Segment { start: center, end }, self.1.distance(center, end)),
        )
    }
    fn nearest_vertex(&self, pt: VecN<N, f64>) -> Option<Segment<N>> {
        self.0
            .nearest(&|v| self.1.distance(pt, v), &|c| {
                self.1.distance_to_cube(pt, c)
            })
            .map(|nearest| Segment {
                start: pt,
                end: nearest,
            })
    }
    fn foreach_vertex(&self, f: &mut impl FnMut(VecN<N, f64>)) {
        self.0.foreach(f);
    }
}
