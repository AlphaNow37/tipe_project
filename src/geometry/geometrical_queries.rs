use crate::{
    datastructures::bsp::Bsp,
    geometry::{
        shapes::Cube,
        workspace::{Length, UniformTopology, WorkspaceTopology},
        VecN,
    },
};

/// Used to answer geometrical queries like nearest neighbors / R-nearest neighbors
pub trait GeometricalQueryDataStore<W: WorkspaceTopology> {
    /// Crée une structure vide
    fn new_store(workspace: W) -> Self;
    /// Ajoute un sommmet
    fn insert_vertex(&mut self, pt: W::Vertex);
    /// Test l'appartenance d'un sommet
    fn contains_vertex(&self, pt: W::Vertex) -> bool;
    /// Applique f à tout les sommets à distance inférieure à radius de center
    fn foreach_r_neighbors(&self, center: W::Vertex, radius: f64, f: &mut impl FnMut(W::Vertex));
    /// Retourne le sommet le plus proche de pt (renvoie None s'il n'y en a pas)
    fn nearest_vertex(&self, pt: W::Vertex) -> Option<W::Vertex>;
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
    fn contains_vertex(&self, pt: W::Vertex) -> bool {
        self.0.contains(&pt)
    }
    fn foreach_r_neighbors(&self, center: W::Vertex, radius: f64, f: &mut impl FnMut(W::Vertex)) {
        for v in self.0.iter() {
            if self.1.distance(center, *v) <= radius {
                f(*v)
            }
        }
    }
    fn nearest_vertex(&self, pt: W::Vertex) -> Option<W::Vertex> {
        let mut min_v = None;
        let mut min_dist = f64::INFINITY;
        for v in self.0.iter() {
            let dist = self.1.distance(pt, *v);
            if dist < min_dist {
                min_dist = dist;
                min_v = Some(*v);
            }
        }
        min_v
    }
    fn foreach_vertex(&self, f: &mut impl FnMut(W::Vertex)) {
        for v in self.0.iter() {
            f(*v)
        }
    }
}

impl<D: Length<N>, const N: usize> GeometricalQueryDataStore<UniformTopology<N, D>>
    for (Bsp<N>, UniformTopology<N, D>)
{
    fn new_store(workspace: UniformTopology<N, D>) -> Self {
        (
            Bsp::new_default_config(
                Cube::from_point(workspace.offsets).with_point(workspace.offsets + workspace.sizes),
            ),
            workspace,
        )
    }
    fn insert_vertex(&mut self, pt: VecN<N, f64>) {
        self.0.insert(pt);
    }
    fn contains_vertex(&self, pt: VecN<N, f64>) -> bool {
        self.0.contains(pt)
    }
    fn foreach_r_neighbors(
        &self,
        center: VecN<N, f64>,
        radius: f64,
        f: &mut impl FnMut(VecN<N, f64>),
    ) {
        self.0.foreach_r_neighborhood(
            radius,
            &|v| self.1.distance(center, v),
            &|c| self.1.distance_to_cube(center, c),
            f,
        )
    }
    fn nearest_vertex(&self, pt: VecN<N, f64>) -> Option<VecN<N, f64>> {
        self.0.nearest(&|v| self.1.distance(pt, v), &|c| {
            self.1.distance_to_cube(pt, c)
        })
    }
    fn foreach_vertex(&self, f: &mut impl FnMut(VecN<N, f64>)) {
        self.0.foreach(f);
    }
}
