use std::{collections::HashMap, marker::PhantomData, time::Instant};

use rand::rng;

use crate::utils::numbers::F64_EPSILON;
use crate::{
    graphs::{Graph, MapGraph, ParentTree, Tree},
    workspace::{
        geometrical_queries::GeometricalQueryDataStore,
        obstacles::ObstaclesEnv,
        workspace::{path_length, WorkspaceTopology},
    },
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
    O: ObstaclesEnv<W>,
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
    mut p: GraphHeuristicParameters<W, impl ObstaclesEnv<W>, Q, impl ExecutionManager<W::Vertex>>,
) -> (Option<(Vec<W::Vertex>, f64)>, ParentTree<W::Vertex>) {
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
            .nearest_vertex(xrand)
            .map(|s| p.workspace.segment_reverse(s))
            .expect("There should be at least one vertex");
        let snew = p.workspace.steer_in_disc(snearest, p.moving_radius);

        if p.obstacles.collide_segment(snew) {
            continue;
        }

        let xnew = p.workspace.segment_end(snew);
        let xnearest = p.workspace.segment_start(snew);
        vertices.insert_vertex(xnew);
        tree.set_parent(xnew, xnearest);

        // Early return pour rrt
        if p.workspace.distance(xnew, p.end) <= p.moving_radius
            && !p
                .obstacles
                .collide_segment(p.workspace.segment(xnew, p.end))
        {
            vertices.insert_vertex(p.end);
            tree.set_parent(p.end, xnew);
            let path = tree.path_to(p.end);
            let length = path_length(&p.workspace, &path);
            return (Some((path, length)), tree);
        }
    }
    (None, tree)
}

/// Algo RRT*
/// Est asymptotiquement optimal
pub fn rrt_star<W: WorkspaceTopology, Q: GeometricalQueryDataStore<W>>(
    mut p: GraphHeuristicParameters<W, impl ObstaclesEnv<W>, Q, impl ExecutionManager<W::Vertex>>,
) -> (Option<(Vec<W::Vertex>, f64)>, Tree<W::Vertex>) {
    // Initialisation des structures de données
    let mut rng = rng();
    let mut tree = Tree::new();
    let mut distance: HashMap<W::Vertex, f64> = HashMap::new();
    let mut vertices = Q::new_store(p.workspace.clone());

    vertices.insert_vertex(p.start);
    distance.insert(p.start, 0.);

    distance.insert(p.end, f64::INFINITY);

    // xnew->xnear
    let mut visible_nears = Vec::new();

    // Boucle principale
    let mut n = 1;
    while !p.execution_manager.must_stop(n) {
        n += 1;
        p.execution_manager
            .logs(&tree, || distance.get(&p.end).copied());

        // Usual sampling and steering, like in RRT
        let xrand = p.workspace.sample_random(&mut rng);
        // snearest: xnearest->xrand
        let snearest = vertices
            .nearest_vertex(xrand)
            .map(|s| p.workspace.segment_reverse(s))
            .expect("There should be at least one vertex");
        // snew: xnearest->xnew
        let snew = p.workspace.steer_in_disc(snearest, p.moving_radius);

        debug_assert!(
            p.workspace.length(snew) <= p.moving_radius + F64_EPSILON,
            "{}, {}",
            p.workspace.length(snew),
            p.moving_radius
        );
        debug_assert_eq!(p.workspace.segment_start(snew), p.workspace.segment_start(snearest));
        debug_assert_eq!(p.workspace.segment_end(snearest), xrand);

        if p.obstacles.collide_segment(snew) {
            continue;
        }

        let xnew = p.workspace.segment_end(snew);
        let xnearest = p.workspace.segment_start(snew);

        // We find near & visible vertices
        visible_nears.clear();
        let radius = p.base_rewire_radius
            * (((n as f64).ln() + 1.) / (n as f64)).powf(1. / (W::NB_DIMENSIONS + 1) as f64);
        vertices.foreach_r_neighbors(xnew, radius, &mut |snear, dist| {
            debug_assert_eq!(dist, p.workspace.length(snear));
            debug_assert_eq!(p.workspace.segment_start(snear), xnew);
            if !p.obstacles.collide_segment(snear) {
                visible_nears.push((snear, dist))
            }
        });

        // dbg!(&visible_nears.len());

        // We find the best parent for xnew
        let mut best_parent = xnearest;
        let mut best_cost = distance[&xnearest] + p.workspace.length(snew);
        for (snear, dist) in &visible_nears {
            let xnear = p.workspace.segment_end(*snear);
            let cost = distance[&xnear] + dist;
            if cost < best_cost {
                best_parent = xnear;
                best_cost = cost;
            }
        }

        // dbg!(xnew, best_parent, best_cost);
        vertices.insert_vertex(xnew);
        tree.set_parent(xnew, best_parent);
        distance.insert(xnew, best_cost);

        // We rewire
        for (snear, dist) in &visible_nears {
            let xnear = p.workspace.segment_end(*snear);
            let delta = best_cost + dist - distance[&xnear];
            // We see an improvement
            if delta < 0. {
                tree.set_parent(xnear, xnew);
                let mut stack = vec![xnear];
                while let Some(child) = stack.pop() {
                    distance.entry(child).and_modify(|cost| *cost += delta);
                    for subchild in tree.get_children(child) {
                        stack.push(*subchild)
                    }
                }
            }
        }

        // TODO place it in a vertex filter operation..
        let dist_to_end = p.workspace.distance(xnew, p.end);
        if dist_to_end <= p.moving_radius
            && !p
                .obstacles
                .collide_segment(p.workspace.segment(xnew, p.end))
            && best_cost + dist_to_end < distance[&p.end]
        {
            distance.insert(p.end, best_cost + dist_to_end);
            tree.set_parent(p.end, xnew);
        }
    }

    let end_cost = distance[&p.end];
    if end_cost == f64::INFINITY {
        (None, tree)
    } else {
        let path = tree.path_to(p.end);
        (Some((path, end_cost)), tree)
    }
}

/// Algo PRM
/// Est asymptotiquement optimal
pub fn prm<W: WorkspaceTopology, Q: GeometricalQueryDataStore<W>>(
    mut p: GraphHeuristicParameters<W, impl ObstaclesEnv<W>, Q, impl ExecutionManager<W::Vertex>>,
) -> (Option<(Vec<W::Vertex>, f64)>, MapGraph<W::Vertex>) {
    // Initialisation des structures de données
    let mut rng = rng();
    let mut graph = MapGraph::default();
    let mut vertices = Q::new_store(p.workspace.clone());

    vertices.insert_vertex(p.start);
    vertices.insert_vertex(p.end);

    // Boucle principale
    let mut n = 1;
    while !p.execution_manager.must_stop(n) {
        n += 1;
        p.execution_manager.logs(&graph, || {
            graph
                .a_star_with(p.start, p.end, |v| v, &p.workspace)
                .map(|(_, l)| l)
        });

        let xrand = p.workspace.sample_random(&mut rng);

        if p.obstacles.collide_vertex(xrand) {
            continue;
        }

        vertices.foreach_r_neighbors(xrand, p.moving_radius, &mut |snear, _| {
            if !p.obstacles.collide_segment(snear) {
                let xnear = p.workspace.segment_end(snear);
                graph.add_new_link(xrand, xnear);
                graph.add_new_link(xnear, xrand);
            }
        });

        vertices.insert_vertex(xrand);
    }
    (
        graph.theta_star_with(p.start, p.end, |v| v, &p.workspace, p.obstacles),
        graph,
    )
}
