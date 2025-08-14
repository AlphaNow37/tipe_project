use std::{collections::HashMap, marker::PhantomData, time::Instant};

use rand::rng;

use crate::{
    workspace::{
        geometrical_queries::GeometricalQueryDataStore,
        obstacles::ObstaclesEnv,
        workspace::{path_length, WorkspaceTopology},
    },
    graphs::{Graph, MapGraph, ParentTree, Tree},
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
    /// The base distance (gamma) used by rrt* where it tries to rewire
    pub base_rewire_radius: f64,
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
    let mut n = 1;
    while !param.execution_manager.must_stop(n) {
        n += 1;
        param.execution_manager.logs(&tree, || None);

        let xrand = param.workspace.sample_random(&mut rng);
        let xnearest = vertices
            .nearest_vertex(xrand)
            .expect("There should be at least one vertex");
        let xnew = param
            .workspace
            .steer_in_disc(xrand, xnearest, param.moving_radius);

        if !param.obstacles.visible(xnearest, xnew) {
            continue;
        }

        vertices.insert_vertex(xnew);
        tree.set_parent(xnew, xnearest);

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

/// Algo RRT*
/// Est asymptotiquement optimal
pub fn rrt_star<W: WorkspaceTopology, Q: GeometricalQueryDataStore<W>>(
    mut param: GraphHeuristicParameters<
        W,
        impl ObstaclesEnv<W::Vertex>,
        Q,
        impl ExecutionManager<W::Vertex>,
    >,
) -> (Option<(Vec<W::Vertex>, f64)>, Tree<W::Vertex>) {
    // Initialisation des structures de données
    let mut rng = rng();
    let mut tree = Tree::new();
    let mut distance: HashMap<W::Vertex, f64> = HashMap::new();
    let mut vertices = Q::new_store(param.workspace.clone());

    vertices.insert_vertex(param.start);
    distance.insert(param.start, 0.);

    distance.insert(param.end, f64::INFINITY);

    let mut visible_nears = Vec::new();

    // Boucle principale
    let mut n = 1;
    while !param.execution_manager.must_stop(n) {
        n += 1;
        param
            .execution_manager
            .logs(&tree, || distance.get(&param.end).copied());

        // Usual sampling and steering, like in RRT
        let xrand = param.workspace.sample_random(&mut rng);
        let xnearest = vertices
            .nearest_vertex(xrand)
            .expect("There should be at least one vertex");
        let xnew = param
            .workspace
            .steer_in_disc(xrand, xnearest, param.moving_radius);

        if !param.obstacles.visible(xnearest, xnew) {
            continue;
        }

        // dbg!("adding a new vertex");

        // We find near & visible vertices
        visible_nears.clear();
        let radius = param.base_rewire_radius
            * (((n as f64).ln() + 1.) / (n as f64)).powf(1. / (W::NB_DIMENSIONS + 1) as f64);
        vertices.foreach_r_neighbors(xnew, radius, &mut |xnear| {
            if param.obstacles.visible(xnew, xnear) {
                visible_nears.push((xnear, param.workspace.distance(xnew, xnear)))
            }
        });

        // dbg!(&visible_nears.len());

        // We find the best parent for xnew
        let mut best_parent = xnearest;
        let mut best_cost = distance[&xnearest] + param.workspace.distance(xnearest, xnew);
        for (xnear, dist) in &visible_nears {
            let cost = distance[&xnear] + dist;
            if cost < best_cost {
                best_parent = *xnear;
                best_cost = cost;
            }
        }

        // dbg!(xnew, best_parent, best_cost);
        vertices.insert_vertex(xnew);
        tree.set_parent(xnew, best_parent);
        distance.insert(xnew, best_cost);

        // We rewire
        for (xnear, dist) in &visible_nears {
            if distance[xnear] > best_cost + dist {
                // dbg!("rewiring");
                tree.set_parent(*xnear, xnew);
                let mut stack = vec![(xnew, *xnear, best_cost)];
                while let Some((parent, child, parent_cost)) = stack.pop() {
                    // dbg!(child);
                    let child_cost = parent_cost + param.workspace.distance(parent, child);
                    distance.insert(child, child_cost);
                    for c in tree.get_children(child) {
                        stack.push((child, *c, child_cost))
                    }
                }
            }
        }

        let dist_to_end = param.workspace.distance(xnew, param.end);
        if dist_to_end <= param.moving_radius
            && param.obstacles.visible(xnew, param.end)
            && best_cost + dist_to_end < distance[&param.end]
        {
            distance.insert(param.end, best_cost + dist_to_end);
            tree.set_parent(param.end, xnew);
        }
    }

    let end_cost = distance[&param.end];
    if end_cost == f64::INFINITY {
        (None, tree)
    } else {
        let path = tree.path_to(param.end);
        (Some((path, end_cost)), tree)
    }
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
    let mut n = 1;
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
        graph.theta_star_with(
            param.start,
            param.end,
            |v| v,
            &param.workspace,
            param.obstacles,
        ),
        graph,
    )
}
