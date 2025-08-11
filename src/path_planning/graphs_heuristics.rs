use std::{marker::PhantomData, time::Instant};

use rand::rng;

use crate::{
    geometry::{
        geometrical_queries::GeometricalQueryDataStore,
        obstacles::ObstaclesEnv,
        workspace::{path_length, WorkspaceTopology},
    },
    graphs::{Graph, MapGraph, ParentTree},
};

pub trait ExecutionManager<V> {
    fn logs(&mut self, _graph: &impl Graph<V>, _length: impl FnOnce() -> Option<f64>) {}
    fn must_stop(&self, _nb_samples: usize) -> bool {
        false
    }
}

pub struct ContinueUntil(pub Instant);
impl<V> ExecutionManager<V> for ContinueUntil {
    fn must_stop(&self, _nb_samples: usize) -> bool {
        Instant::now() > self.0
    }
}

pub struct SampleNTimes(pub usize);
impl<V> ExecutionManager<V> for SampleNTimes {
    fn must_stop(&self, nb_samples: usize) -> bool {
        nb_samples > self.0
    }
}

/// Standard parameters for graph heuristics
pub struct GraphHeuristicParameters<
    'a,
    W: WorkspaceTopology,
    O: ObstaclesEnv<W::Vertex>,
    Q: GeometricalQueryDataStore<W>,
    M: ExecutionManager<W::Vertex>,
> {
    /// The obstacles space
    pub obstacles: &'a O,
    /// The path's start
    pub start: W::Vertex,
    /// The path's end
    pub end: W::Vertex,
    /// The workspace where we try to find a path
    pub workspace: W,
    /// The maximum distance between two vertices on the graph
    pub moving_radius: f64,
    /// The data structure than shall be used for geometrical queries
    pub vertices: PhantomData<Q>,
    /// Decides when to return and allow intermediate logs
    pub execution_manager: M,
}

/// Algo RRT
/// N'est pas asymptotiquement optimal
pub fn rrt<W: WorkspaceTopology, Q: GeometricalQueryDataStore<W>>(
    mut param: GraphHeuristicParameters<
        W,
        impl ObstaclesEnv<W::Vertex>,
        Q,
        impl ExecutionManager<W::Vertex>,
    >,
) -> (Option<(Vec<W::Vertex>, f64)>, ParentTree<W::Vertex>) {
    // Initialisation des structures de données
    let mut rng = rng();
    let mut tree = ParentTree::new();
    let mut vertices = Q::new_store(param.workspace.clone());
    vertices.insert_vertex(param.start);

    // Boucle principale
    let mut n = 0;
    while !param.execution_manager.must_stop(n) {
        n += 1;
        param.execution_manager.logs(&tree, || None);

        let xrand = param.workspace.sample_random(&mut rng);
        let xnear = vertices
            .nearest_vertex(xrand)
            .expect("There shoudl be at least one vertex");
        let xnew = param
            .workspace
            .steer_in_disc(xrand, xnear, param.moving_radius);

        if !param.obstacles.visible(xnear, xnew) {
            continue;
        }

        vertices.insert_vertex(xnew);
        tree.set_parent(xnew, xnear);

        // Early return pour rrt
        if param.workspace.distance(xnew, param.end) <= param.moving_radius
            && param.obstacles.visible(xnew, param.end)
        {
            vertices.insert_vertex(param.end);
            tree.set_parent(param.end, xnew);
            let path = tree.path_to(param.end);
            let length = path_length(&param.workspace, &path);
            return (Some((path, length)), tree);
        }
    }
    (None, tree)
}

/// Algo PRM
/// Est asymptotiquement optimal
pub fn prm<W: WorkspaceTopology, Q: GeometricalQueryDataStore<W>>(
    mut param: GraphHeuristicParameters<
        W,
        impl ObstaclesEnv<W::Vertex>,
        Q,
        impl ExecutionManager<W::Vertex>,
    >,
) -> (Option<(Vec<W::Vertex>, f64)>, MapGraph<W::Vertex>) {
    // Initialisation des structures de données
    let mut rng = rng();
    let mut graph = MapGraph::default();
    let mut vertices = Q::new_store(param.workspace.clone());

    vertices.insert_vertex(param.start);
    vertices.insert_vertex(param.end);

    // Boucle principale
    let mut n = 0;
    while !param.execution_manager.must_stop(n) {
        n += 1;
        param.execution_manager.logs(&graph, || {
            graph
                .a_star_with(param.start, param.end, |v| v, &param.workspace)
                .map(|(_, l)| l)
        });

        let xrand = param.workspace.sample_random(&mut rng);

        if param.obstacles.contains(xrand) {
            continue;
        }

        vertices.foreach_r_neighbors(xrand, param.moving_radius, &mut |xnear| {
            if param.obstacles.visible(xrand, xnear) {
                graph.add_new_link(xrand, xnear);
                graph.add_new_link(xnear, xrand);
            }
        });

        vertices.insert_vertex(xrand);
    }
    (
        graph.a_star_with(param.start, param.end, |v| v, &param.workspace),
        graph,
    )
}
