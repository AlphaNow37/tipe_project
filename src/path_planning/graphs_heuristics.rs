use std::marker::PhantomData;

use rand::rng;

use crate::{
    datastructures::geometrical_queries::GeometricalQueryDataStore,
    geometry::{obstacles::ObstaclesEnv, workspace::WorkspaceTopology},
    graphs::{MapGraph, ParentTree},
};

pub struct GraphHeuristicParameters<
    'a,
    W: WorkspaceTopology,
    O: ObstaclesEnv<W::Vertex>,
    Q: GeometricalQueryDataStore<W>,
> {
    pub obstacles: &'a O,
    pub start: W::Vertex,
    pub goal: W::Vertex,
    pub workspace: W,
    pub moving_radius: f64,
    pub vertices: PhantomData<Q>,
}

pub fn rrt<W: WorkspaceTopology, Q: GeometricalQueryDataStore<W>>(
    param: GraphHeuristicParameters<W, impl ObstaclesEnv<W::Vertex>, Q>,
) -> (Option<Vec<W::Vertex>>, ParentTree<W::Vertex>) {
    // Initialisation des structures de données
    let mut rng = rng();
    let mut tree = ParentTree::new();
    let mut vertices = Q::new_store(param.workspace.clone());
    vertices.insert_vertex(param.start);

    // Boucle principale
    for _ in 0..100000 {
        let xrand = param.workspace.sample_random(&mut rng);
        let xnear = vertices.nearest_vertex(xrand);
        let xnew = param
            .workspace
            .steer_in_disc(xrand, xnear, param.moving_radius);

        if !param.obstacles.visible(xnear, xnew) {
            continue;
        }

        vertices.insert_vertex(xnew);
        tree.set_parent(xnew, xnear);

        // Early return pour rrt
        if param.workspace.distance(xnew, param.goal) <= param.moving_radius
            && param.obstacles.visible(xnew, param.goal)
        {
            vertices.insert_vertex(param.goal);
            tree.set_parent(param.goal, xnew);

            return (Some(tree.path_to(param.goal)), tree);
        }
    }
    (None, tree)
}
