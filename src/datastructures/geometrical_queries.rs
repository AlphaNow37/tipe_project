use crate::geometry::workspace::WorkspaceTopology;

/// Used to answer geometrical queries like nearest neighbors / R-nearest neighbors
pub trait GeometricalQueryDataStore<W: WorkspaceTopology> {
    /// Crée une structure vide
    fn new_store(workspace: W) -> Self;
    /// Ajoute un sommmet
    fn insert_vertex(&mut self, pt: W::Vertex);
    /// Test l'appartenance d'un sommet
    fn contains_vertex(&self, pt: W::Vertex) -> bool;
    /// Applique f à tout les sommets à distance inférieure à radius de center
    fn map_r_neighbors(&self, center: W::Vertex, radius: f64, f: &mut impl FnMut(W::Vertex));
    /// Retourne le sommet le plus proche de pt (panique s'il n'y en a pas)
    fn nearest_vertex(&self, pt: W::Vertex) -> W::Vertex;
    /// Applique f à touts les sommets
    fn map_all_vertex(&self, f: &mut impl FnMut(W::Vertex));
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
    fn map_r_neighbors(&self, center: W::Vertex, radius: f64, f: &mut impl FnMut(W::Vertex)) {
        for v in self.0.iter() {
            if self.1.distance(center, *v) <= radius {
                f(*v)
            }
        }
    }
    fn nearest_vertex(&self, pt: W::Vertex) -> W::Vertex {
        let mut min_v = None;
        let mut min_dist = f64::INFINITY;
        for v in self.0.iter() {
            let dist = self.1.distance(pt, *v);
            if dist < min_dist {
                min_dist = dist;
                min_v = Some(*v);
            }
        }
        min_v.expect("There should be at least one vertex")
    }
    fn map_all_vertex(&self, f: &mut impl FnMut(W::Vertex)) {
        for v in self.0.iter() {
            f(*v)
        }
    }
}

// TODO
// pub struct BspTree {

// }
