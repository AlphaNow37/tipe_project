use std::marker::PhantomData;

use rand::rng;

use crate::{
    datastructures::geometrical_queries::GeometricalQueryDataStore,
    geometry::{obstacles::ObstaclesEnv, workspace::WorkspaceTopology},
    graphs::{Graph, MapGraph, ParentTree},
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
) -> (Option<(Vec<W::Vertex>, f64)>, ParentTree<W::Vertex>) {
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
            let path = tree.path_to(param.goal);
            let length = (0..(path.len() - 1))
                .map(|i| param.workspace.distance(path[i], path[i + 1]))
                .sum::<f64>();
            return (Some((path, length)), tree);
        }
    }
    (None, tree)
}

pub fn prm<W: WorkspaceTopology, Q: GeometricalQueryDataStore<W>>(
    param: GraphHeuristicParameters<W, impl ObstaclesEnv<W::Vertex>, Q>,
) -> (Option<(Vec<W::Vertex>, f64)>, MapGraph<W::Vertex>) {
    // Initialisation des structures de données
    let mut rng = rng();
    let mut graph = MapGraph::default();
    let mut vertices = Q::new_store(param.workspace.clone());

    vertices.insert_vertex(param.start);
    vertices.insert_vertex(param.goal);

    // Boucle principale
    for _ in 0..10000 {
        let xrand = param.workspace.sample_random(&mut rng);

        if param.obstacles.contains(xrand) {
            continue;
        }

        vertices.map_r_neighbors(xrand, param.moving_radius, &mut |xnear| {
            if !param.obstacles.visible(xrand, xnear) {
                return;
            }
            graph.add_new_link(xrand, xnear);
            graph.add_new_link(xnear, xrand);
        });

        vertices.insert_vertex(xrand);
    }
    (
        graph.a_star_with(param.start, param.goal, |v| v, &param.workspace),
        graph,
    )
}
