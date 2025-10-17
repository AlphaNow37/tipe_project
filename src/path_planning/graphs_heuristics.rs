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

// struct RrtStarState<'a, R, W: WorkspaceTopology, Q, E, M> {
//     rng: R,
//     workspace: W,
//     vertices: Q,
//     end: W::Vertex,
//     execution_manager: M,
//     obstacles: &'a E,
//     moving_radius: f64,
//     base_rewire_radius: f64,
//     tree: Tree<W::Vertex>,
//     distance: HashMap<W::Vertex, f64>,
//     visible_nears: Vec<(W::Segment, f64)>,
//     iteration_count: usize,
// }
// impl<
//         'a,
//         R: rand::Rng,
//         W: WorkspaceTopology,
//         Q: GeometricalQueryDataStore<W>,
//         E: ObstaclesEnv<W>,
//         M: ExecutionManager<W::Vertex>,
//     > RrtStarState<'a, R, W, Q, E, M>
// {
//     fn new(params: GraphHeuristicParameters<'a, W, E, Q, M>, rng: R) -> Self {
//         let mut s = Self {
//             rng,
//             vertices: Q::new_store(params.workspace.clone()),
//             obstacles: params.obstacles,
//             end: params.end,
//             execution_manager: params.execution_manager,
//             moving_radius: params.moving_radius,
//             base_rewire_radius: params.base_rewire_radius,
//             workspace: params.workspace,
//             tree: Tree::new(),
//             distance: HashMap::new(),
//             visible_nears: Vec::new(),
//             iteration_count: 1,
//         };
//         s.add_node(params.start, 0., None);
//         s.distance.insert(params.end, f64::INFINITY);
//         s
//     }
//     fn add_node(&mut self, vertex: W::Vertex, distance: f64, parent: Option<W::Vertex>) {
//         self.vertices.insert_vertex(vertex);
//         self.distance.insert(vertex, distance);
//         if let Some(p) = parent {
//             self.tree.set_parent(vertex, p);
//         }
//     }
//     fn random_node(&mut self) -> W::Segment {
//         // Usual sampling and steering, like in RRT
//         let xrand = self.workspace.sample_random(&mut self.rng);
//         // snearest: xnearest->xrand
//         let snearest = self
//             .vertices
//             .nearest_vertex_rev(xrand)
//             .expect("There should be at least one vertex");
//         // snew: xnearest->xnew
//         let snew = self.workspace.steer_in_disc(snearest, self.moving_radius);
//
//         debug_assert!(
//             self.workspace.length(snew) <= self.moving_radius + F64_EPSILON,
//             "{}, {}",
//             self.workspace.length(snew),
//             self.moving_radius
//         );
//         debug_assert_eq!(
//             self.workspace.segment_start(snew),
//             self.workspace.segment_start(snearest)
//         );
//         debug_assert_eq!(self.workspace.segment_end(snearest), xrand);
//
//         snew
//     }
//     fn update_visible_verteces(&mut self, xnew: W::Vertex, reverse: bool) {
//         // We find near & visible vertices
//         self.visible_nears.clear();
//         let radius = self.base_rewire_radius
//             * (((self.iteration_count as f64).ln() + 1.) / (self.iteration_count as f64)).powf(1. / (W::NB_DIMENSIONS + 1) as f64);
//         let mut f = |snear, dist| {
//             debug_assert_eq!(dist, self.workspace.length(snear));
//             debug_assert_eq!(self.workspace.segment_start(snear), xnew);
//             if !self.obstacles.collide_segment(snear) {
//                 self.visible_nears.push((snear, dist))
//             }
//         };
//         if reverse {
//             self.vertices.foreach_r_neighbors_rev(xnew, radius, &mut f);
//         } else {
//             self.vertices.foreach_r_neighbors(xnew, radius, &mut f);
//         }
//     }
//     fn find_best_parent(&mut self, xnew: W::Vertex, xnearest: W::Vertex, snew: W::Segment) -> (W::Vertex, f64) {
//         let mut best_parent = xnearest;
//         let mut best_cost = self.distance[&xnearest] + self.workspace.length(snew);
//         for (snear, dist) in &self.visible_nears {
//             let xnear = self.workspace.segment_end(*snear);
//             let cost = self.distance[&xnear] + dist;
//             if cost < best_cost {
//                 best_parent = xnear;
//                 best_cost = cost;
//             }
//         }
//         (best_parent, best_cost)
//     }
//     fn change_parent(&mut self, xnode: W::Vertex, xparent: W::Vertex, delta: f64) {
//         self.tree.set_parent(xnode, xparent);
//         let mut stack = vec![xnode];
//         while let Some(child) = stack.pop() {
//             self.distance.entry(child).and_modify(|cost| *cost += delta);
//             for subchild in self.tree.get_children(child) {
//                 stack.push(*subchild)
//             }
//         }
//     }
//     fn rewire_symetric(&mut self, xnew: W::Vertex, xnew_cost: f64) {
//         for (snear, dist) in &self.visible_nears {
//             let xnear = self.workspace.segment_end(*snear);
//             let delta = xnew_cost + dist - self.distance[&xnear];
//             // We see an improvement
//             if delta < 0. {
//                 self.change_parent(xnear, xnew, delta)
//             }
//         }
//     }
//     fn rewire_non_symetric(&mut self, xnew: W::Vertex) {
//         self.update_visible_verteces(xnew, true);
//     }
//     fn main_loop(&mut self) {
//         self.iteration_count = 1;
//         while !self.execution_manager.must_stop(self.iteration_count) {
//             self.iteration_count += 1;
//             self.execution_manager
//                 .logs(&self.tree, || self.distance.get(&self.end).copied());
//
//             let snew = self.random_node();
//             if self.obstacles.collide_segment(snew) {
//                 continue;
//             }
//
//             let xnew = self.workspace.segment_end(snew);
//             let xnearest = self.workspace.segment_start(snew);
//
//             self.update_visible_verteces(xnew, false);
//
//             let (xparent, cost) = self.find_best_parent(xnew, xnearest, snew);
//
//             self.add_node(xnew, cost, Some(xparent))
//
//
//
//         }
//     }
// }

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
        }    fn rewire() {

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
