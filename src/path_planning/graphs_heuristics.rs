use std::{collections::HashMap, marker::PhantomData, time::Instant};

use rand::rng;

use crate::graphs::IterableGraph;
use crate::utils::numbers::F64_EPSILON;
use crate::{
    graphs::{Graph, MapGraph, ParentTree, Tree},
    workspace::{
        geometrical_queries::GeometricalQueryDataStore,
        obstacles::ObstaclesEnv,
        workspace::{path_length, WorkspaceTopology},
    },
};


/// Ce fichier implémente RRT, RRT*, PRM

/// Ce trait sert à récuperer des infos sur l'exécution et à terminer l'algorithme au moment qu'on veut
pub trait ExecutionManager<V, I, W: WorkspaceTopology> {
    fn logs(
        &mut self,
        _graph: &impl Graph<V, I>,
        _length: impl FnOnce() -> Option<Vec<W::Segment>>,
    ) {
    }
    fn must_stop(&self, _nb_samples: usize) -> bool {
        false
    }
}

pub struct ContinueUntil(pub Instant);
impl<V, I, W: WorkspaceTopology> ExecutionManager<V, I, W> for ContinueUntil {
    fn must_stop(&self, _nb_samples: usize) -> bool {
        Instant::now() > self.0
    }
}

pub struct SampleNTimes(pub usize);
impl<V, I, W: WorkspaceTopology> ExecutionManager<V, I, W> for SampleNTimes {
    fn must_stop(&self, nb_samples: usize) -> bool {
        nb_samples > self.0
    }
}

/// Objectif recherché
#[derive(Copy, Clone)]
pub enum Goal<'a, W: WorkspaceTopology> {
    Vertex(W::Vertex),
    Predicate(&'a dyn Fn(W::Vertex) -> bool),
}

/// Standard parameters for graph heuristics
pub struct GraphHeuristicParameters<
    'a,
    W: WorkspaceTopology,
    O: ObstaclesEnv<W>,
    Q: GeometricalQueryDataStore<W>,
    M: ExecutionManager<W::Vertex, W::Segment, W>,
> {
    /// The obstacles space
    pub obstacles: &'a O,
    /// The path's start
    pub start: W::Vertex,
    /// The path's end
    pub end: Goal<'a, W>,
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
    mut p: GraphHeuristicParameters<
        W,
        impl ObstaclesEnv<W>,
        Q,
        impl ExecutionManager<W::Vertex, W::Segment, W>,
    >,
) -> (
    Option<(Vec<W::Segment>, f64)>,
    ParentTree<W::Vertex, W::Segment>,
) {
    // Initialisation des structures de données
    let mut rng = rng();
    let mut tree = ParentTree::new();
    let mut vertices = Q::new_store(p.workspace.clone());
    vertices.insert_vertex(p.start);

    // Boucle principale
    let mut n = 1;
    while !p.execution_manager.must_stop(n) {
        n += 1;
        p.execution_manager.logs(&tree, || None);

        let xrand = p.workspace.sample_random(&mut rng);
        let snearest = vertices
            .nearest_vertex_rev(xrand)
            .expect("There should be at least one vertex");
        let snew = p.workspace.steer_in_disc(snearest, p.moving_radius);

        if p.obstacles.collide_segment(snew) {
            continue;
        }

        let xnew = p.workspace.segment_end(snew);
        // let xnearest = p.workspace.segment_start(snew);
        vertices.insert_vertex(xnew);
        tree.set_parent(xnew, snew);

        match p.end {
            Goal::Vertex(end) => {
                // Early return pour rrt
                if p.workspace.distance(xnew, end) <= p.moving_radius
                    && !p.obstacles.collide_segment(p.workspace.segment(xnew, end))
                {
                    vertices.insert_vertex(end);
                    tree.set_parent(end, p.workspace.segment_reverse(snew));
                    let path = tree.path_to(end);
                    let length = path_length(&p.workspace, &path);
                    p.execution_manager.logs(&tree, || Some(path.clone()));
                    return (Some((path, length)), tree);
                }
            }
            Goal::Predicate(predicate) => {
                if predicate(xnew) {
                    let path = tree.path_to(xnew);
                    let length = path_length(&p.workspace, &path);
                    p.execution_manager.logs(&tree, || Some(path.clone()));
                    return (Some((path, length)), tree);
                }
            }
        }
    }
    (None, tree)
}

/// Algo RRT*
/// Est asymptotiquement optimal
pub fn rrt_star<W: WorkspaceTopology, Q: GeometricalQueryDataStore<W>>(
    mut p: GraphHeuristicParameters<
        W,
        impl ObstaclesEnv<W>,
        Q,
        impl ExecutionManager<W::Vertex, W::Segment, W>,
    >,
) -> (Option<(Vec<W::Segment>, f64)>, Tree<W::Vertex, W::Segment>) {
    // Initialisation des structures de données
    let mut rng = rng();
    let mut tree = Tree::new();
    let mut distance: HashMap<W::Vertex, f64> = HashMap::new();
    let mut vertices = Q::new_store(p.workspace.clone());

    vertices.insert_vertex(p.start);
    distance.insert(p.start, 0.);

    let mut best_end_vertex = None;

    let end_path = |distance: &HashMap<W::Vertex, f64>,
                    tree: &Tree<W::Vertex, W::Segment>,
                    best_end_vertex: Option<W::Vertex>| {
        match best_end_vertex {
            None => None,
            Some(best_end) => {
                let end_cost = distance[&best_end];
                let path = tree
                    .path_to(best_end)
                    .into_iter()
                    .map(|edge| edge)
                    .collect();
                Some((path, end_cost))
            }
        }
    };

    // Boucle principale
    let mut n = 1;
    while !p.execution_manager.must_stop(n) {
        n += 1;
        p.execution_manager.logs(&tree, || {
            end_path(&distance, &tree, best_end_vertex).map(|(p, _)| p)
        });

        // Usual sampling and steering, like in RRT
        let xrand = p.workspace.sample_random(&mut rng);
        // snearest: xnearest->xrand
        let snearest = vertices
            .nearest_vertex_rev(xrand)
            .expect("There should be at least one vertex");
        // snew: xnearest->xnew
        let snew = p.workspace.steer_in_disc(snearest, p.moving_radius);

        debug_assert!(
            p.workspace.length(snew) <= p.moving_radius + F64_EPSILON,
            "{}, {}",
            p.workspace.length(snew),
            p.moving_radius
        );
        debug_assert_eq!(
            p.workspace.segment_start(snew),
            p.workspace.segment_start(snearest)
        );
        debug_assert_eq!(p.workspace.segment_end(snearest), xrand);

        if p.obstacles.collide_segment(snew) {
            continue;
        }

        let xnew = p.workspace.segment_end(snew);
        let xnearest = p.workspace.segment_start(snew);

        // We find near & visible vertices
        let radius = p.base_rewire_radius
            * (((n as f64).ln() + 1.) / (n as f64)).powf(1. / (W::NB_DIMENSIONS + 1) as f64);

        // dbg!(&visible_nears.len());

        // We find the best parent for xnew
        let mut best_parent = snew;
        let mut best_cost = distance[&xnearest] + p.workspace.length(snew);
        vertices.foreach_r_neighbors_rev(xnew, radius, &mut |snear, dist| {
            debug_assert_eq!(dist, p.workspace.length(snear));
            debug_assert_eq!(p.workspace.segment_end(snear), xnew);
            if !p.obstacles.collide_segment(snear) {
                let xnear = p.workspace.segment_start(snear);
                let cost = distance[&xnear] + dist;
                if cost < best_cost {
                    best_parent = snear;
                    best_cost = cost;
                    debug_assert!(!p.obstacles.collide_segment(snear));
                }
            }
        });

        // dbg!(xnew, best_parent, best_cost);
        debug_assert!(!p.obstacles.collide_segment(best_parent));

        vertices.insert_vertex(xnew);
        tree.set_parent(xnew, best_parent);
        distance.insert(xnew, best_cost);

        // We rewire
        vertices.foreach_r_neighbors(xnew, radius, &mut |snear, dist| {
            debug_assert_eq!(dist, p.workspace.length(snear));
            debug_assert_eq!(p.workspace.segment_start(snear), xnew);
            if !p.obstacles.collide_segment(snear) {
                let xnear = p.workspace.segment_end(snear);
                let delta = best_cost + dist - distance[&xnear];
                // We see an improvement
                if delta < 0. {
                    tree.set_parent(xnear, snear);
                    let mut stack = vec![xnear];
                    while let Some(child) = stack.pop() {
                        distance.entry(child).and_modify(|cost| *cost += delta);
                        for subchild in tree.get_children(child) {
                            stack.push(*subchild)
                        }
                    }
                }
            }
        });
        match p.end {
            Goal::Vertex(end) => {
                let dist_to_end = p.workspace.distance(xnew, end);
                if dist_to_end <= p.moving_radius {
                    let s_end = p.workspace.segment(xnew, end);
                    if !p.obstacles.collide_segment(s_end)
                        && (!distance.contains_key(&end)
                            || best_cost + dist_to_end < distance[&end])
                    {
                        best_end_vertex = Some(end);
                        distance.insert(end, best_cost + dist_to_end);
                        tree.set_parent(end, s_end);
                    }
                }
            }
            Goal::Predicate(predicate) => {
                if predicate(xnew) {
                    if let Some(best) = best_end_vertex {
                        if distance[&best] > distance[&xnew] {
                            best_end_vertex = Some(xnew)
                        }
                    } else {
                        best_end_vertex = Some(xnew)
                    }
                }
            }
        }
    }

    debug_assert!(tree
        .iter()
        .flat_map(|v| tree.neighbors(v))
        .all(|parent_to_child| !p.obstacles.collide_segment(parent_to_child)));

    (end_path(&distance, &tree, best_end_vertex), tree)
}

/// Algo PRM
/// Est asymptotiquement optimal
pub fn prm<W: WorkspaceTopology, Q: GeometricalQueryDataStore<W>>(
    mut p: GraphHeuristicParameters<
        W,
        impl ObstaclesEnv<W>,
        Q,
        impl ExecutionManager<W::Vertex, W::Segment, W>,
    >,
) -> (
    Option<(Vec<W::Segment>, f64)>,
    MapGraph<W::Vertex, W::Segment>,
) {
    // Initialisation des structures de données
    let mut rng = rng();
    let mut graph = MapGraph::default();
    let mut vertices = Q::new_store(p.workspace.clone());

    vertices.insert_vertex(p.start);
    if let Goal::Vertex(end) = p.end {
        vertices.insert_vertex(end);
    }

    // Boucle principale
    let mut n = 1;
    while !p.execution_manager.must_stop(n) {
        n += 1;
        p.execution_manager.logs(&graph, || {
            todo!()
            // graph
            //     .a_star_with(p.start, p.end, |v| v, &p.workspace)
            //     .map(|(_, l)| l)
        });

        let xrand = p.workspace.sample_random(&mut rng);

        if p.obstacles.collide_vertex(xrand) {
            continue;
        }

        vertices.foreach_r_neighbors(xrand, p.moving_radius, &mut |snear, _| {
            if !p.obstacles.collide_segment(snear) {
                let xnear = p.workspace.segment_end(snear);
                graph.add_new_link(xrand, p.workspace.segment_reverse(snear));
                graph.add_new_link(xnear, snear);
            }
        });

        vertices.insert_vertex(xrand);
    }

    let res = match p.end {
        Goal::Vertex(end) => graph.theta_star_with(p.start, end, |v| v, &p.workspace, p.obstacles),
        Goal::Predicate(predicate) => {
            graph.theta_with(p.start, predicate, |v| v, &p.workspace, p.obstacles)
        }
    };
    if let Some((path, length)) = res {
        let mut edges = Vec::new();
        for i in 0..(path.len() - 1) {
            edges.push(p.workspace.segment(path[i], path[i + 1]))
        }
        (Some((edges, length)), graph)
    } else {
        (None, graph)
    }
}
