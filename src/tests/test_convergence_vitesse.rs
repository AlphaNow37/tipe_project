use crate::datastructures::bsp::Bsp;
use crate::datastructures::r_tree::RTree;
use crate::geometry::angles::Angle;
use crate::geometry::shapes::Cube;
use crate::geometry::VecN;
use crate::graphs::Graph;
use crate::path_planning::graphs_heuristics::{
    rrt, rrt_star, ExecutionManager, Goal, GraphHeuristicParameters,
};
use crate::path_planning::shortcuts::shortcut;
use crate::tests::out_dir;
use crate::utils::benchmark::Benchmark;
use crate::utils::numbers::Zero;
use crate::workspace::cartesians::{CartesianTopology, EuclidianDistance};
use crate::workspace::obstacles::{ObstaclesApprox, ObstaclesEnv};
use crate::workspace::reeds_shepp::{OrientedCoord, ReedsSheppWorkspace};
use crate::workspace::workspace::{path_length, WorkspaceTopology};
use rand::{rng, Rng};
use std::marker::PhantomData;
use std::time::Instant;

const MAX_DURATION: f64 = 5.;
const NB_GRAPH_POINTS: usize = 10000;
const NDIM: usize = 3;

struct Manager<'a, W> {
    last_seen: Instant,
    max_duration: f64,
    curr_duration: f64,
    workspace: W,
    benchmark: &'a mut Benchmark,
    name: &'static str,
}

impl<'a, V, I, W: WorkspaceTopology> ExecutionManager<V, I, W> for Manager<'a, W> {
    fn logs(
        &mut self,
        _graph: &impl Graph<V, I>,
        get_length: impl FnOnce() -> Option<Vec<W::Segment>>,
    ) {
        let delta = self.last_seen.elapsed().as_secs_f64();
        let path = get_length();
        let length = path.map(|p| path_length(&self.workspace, &p));
        let delta_2 = self.last_seen.elapsed().as_secs_f64();
        // dbg!(delta_2-delta);
        if let Some(l) = length {
            self.benchmark
                .add_datapoint(self.name, self.curr_duration + delta_2, l);
        }
        self.curr_duration += delta;
        self.last_seen = Instant::now();
    }
    fn must_stop(&self, _nb_samples: usize) -> bool {
        self.max_duration <= self.curr_duration
    }
}

struct ShortcutManager<'a, M2, W, O> {
    inner: M2,
    workspace: W,
    obstacles: &'a O,
}
impl<'a, V, I, M2: ExecutionManager<V, I, W>, W: WorkspaceTopology, O: ObstaclesEnv<W>>
    ExecutionManager<V, I, W> for ShortcutManager<'a, M2, W, O>
{
    fn logs(&mut self, graph: &impl Graph<V, I>, length: impl FnOnce() -> Option<Vec<W::Segment>>) {
        self.inner.logs(graph, || {
            let path = length();
            path.map(|p| {
                let p2 = shortcut(&self.workspace, p, self.obstacles, 15);
                p2
            })
        })
    }
    fn must_stop(&self, nb_samples: usize) -> bool {
        self.inner.must_stop(nb_samples)
    }
}

fn generate_obstacles<const N: usize>(
    n: usize,
    min_size: f64,
    max_size: f64,
    min_coord: f64,
    max_coord: f64,
) -> RTree<N, Cube<N>> {
    let mut obs = Vec::new();
    let mut r = rng();

    while obs.len() < n {
        let middle = VecN::from_fn(|_| r.random_range(min_coord..max_coord));
        let size = VecN::from_fn(|_| {
            r.random_range(min_size.powf(1. / 5.)..max_size.powf(1. / 5.))
                .powf(5.)
        });
        let start = (middle - size / 2.).map_component(|a| a.max(min_coord));
        let end = (middle + size / 2.).map_component(|a| a.min(max_coord));
        obs.push(Cube::from_point(start).with_point(end))
    }
    RTree::bulk_load(&mut obs)
}

fn generate_times() -> Vec<f64> {
    (1..NB_GRAPH_POINTS)
        .map(|i| i as f64 / NB_GRAPH_POINTS as f64)
        .map(|t| (-1. / t + 1.).exp())
        .map(|t| t * MAX_DURATION)
        .collect()
}

fn test_convergence<W>(
    benchmark: &mut Benchmark,
    name: &'static str,
    workspace: W,
    mut get_path: impl FnMut(Manager<W>),
) {
    let manager = Manager {
        max_duration: MAX_DURATION,
        last_seen: Instant::now(),
        benchmark,
        curr_duration: 0.,
        workspace,
        name,
    };

    get_path(manager);
}

pub fn test_convergence_reeds() {
    let times = generate_times();
    let mut benchmark = Benchmark::new(out_dir().join("cvg_benchmark_reeds_shepp.csv"));

    let space = Cube::from_point(VecN([-1000., -1000.])).with_point(VecN([1000., 1000.]));
    let workspace = ReedsSheppWorkspace {
        forward_only: true,
        physical_space: space,
        steering_radius: 5.,
    };
    let start = (VecN([-750., -750.]), Angle::ZERO);
    let goal = (VecN([750., 750.]), Angle::ZERO);
    let moving_radius = 50.;
    let rewire_radius = 100.;
    let rtree = generate_obstacles(500, 5., 50., -730., 730.);

    let obstacles = ObstaclesApprox {
        workspace,
        visible_resolution: 0.1,
        contains_func: Box::new(|(p, _)| rtree.contains_point(p)),
    };

    test_convergence(&mut benchmark, "rrt", workspace, |ma| {
        let (_, g) = rrt(GraphHeuristicParameters {
            start,
            end: Goal::Vertex(goal),
            workspace,
            execution_manager: ma,
            obstacles: &obstacles,
            moving_radius,
            base_rewire_radius: rewire_radius,
            vertices: PhantomData::<(Vec<OrientedCoord>, ReedsSheppWorkspace)>,
        });
        dbg!(g.nb_links());
    });
    test_convergence(&mut benchmark, "rrt_shortcut", workspace, |ma| {
        let (_, g) = rrt(GraphHeuristicParameters {
            start,
            end: Goal::Vertex(goal),
            workspace,
            execution_manager: ShortcutManager {
                workspace,
                inner: ma,
                obstacles: &obstacles,
            },
            obstacles: &obstacles,
            moving_radius,
            base_rewire_radius: rewire_radius,
            vertices: PhantomData::<(Vec<OrientedCoord>, ReedsSheppWorkspace)>,
        });
        dbg!(g.nb_links());
    });
    test_convergence(&mut benchmark, "rrt_star", workspace, |ma| {
        let (_, g) = rrt_star(GraphHeuristicParameters {
            start,
            end: Goal::Vertex(goal),
            workspace,
            execution_manager: ma,
            obstacles: &obstacles,
            moving_radius,
            base_rewire_radius: rewire_radius,
            vertices: PhantomData::<(Vec<OrientedCoord>, ReedsSheppWorkspace)>,
        });
        dbg!(g.nb_links());
    });
    test_convergence(&mut benchmark, "rrt_star_shortcut", workspace, |ma| {
        let (_, g) = rrt_star(GraphHeuristicParameters {
            start,
            end: Goal::Vertex(goal),
            workspace,
            execution_manager: ShortcutManager {
                workspace,
                inner: ma,
                obstacles: &obstacles,
            },
            obstacles: &obstacles,
            moving_radius,
            base_rewire_radius: rewire_radius,
            vertices: PhantomData::<(Vec<OrientedCoord>, ReedsSheppWorkspace)>,
        });
        dbg!(g.nb_links());
    })
}

pub fn test_convergence_straight() {
    let times = generate_times();
    let mut benchmark = Benchmark::new(out_dir().join("cvg_benchmark_straight.csv"));

    let space = Cube::from_point(VecN::splat(-760.)).with_point(VecN::splat(760.));
    let workspace = CartesianTopology {
        space,
        dist: EuclidianDistance,
    };
    let start = VecN::splat(-500.);
    let goal = VecN::splat(500.);
    let moving_radius = 50.;
    let rewire_radius = 200.;
    let rtree = generate_obstacles(250, 5., 400., -770., 770.);

    let obstacles = rtree;

    test_convergence(&mut benchmark, "rrt", workspace, |ma| {
        let (a, g) = rrt(GraphHeuristicParameters {
            start,
            end: Goal::Vertex(goal),
            workspace,
            execution_manager: ma,
            obstacles: &obstacles,
            moving_radius,
            base_rewire_radius: rewire_radius,
            vertices: PhantomData::<(Bsp<NDIM>, CartesianTopology<NDIM, EuclidianDistance>)>,
        });
        dbg!(g.nb_links());
        dbg!(a.is_some());
    });
    // test_convergence(&mut benchmark, "rrt_shortcut", &times, workspace, |ma| {
    //     let (a, g) = rrt(GraphHeuristicParameters {
    //         start,
    //         end: Goal::Vertex(goal),
    //         workspace,
    //         execution_manager: ShortcutManager {
    //             workspace,
    //             inner: ma,
    //             obstacles: &obstacles,
    //         },
    //         obstacles: &obstacles,
    //         moving_radius,
    //         base_rewire_radius: rewire_radius,
    //         vertices: PhantomData::<(Bsp<NDIM>, CartesianTopology<NDIM, EuclidianDistance>)>,
    //     });
    //     dbg!(g.nb_links());
    // dbg!(a.is_some());

    // });
    test_convergence(&mut benchmark, "rrt_star", workspace, |ma| {
        let (a, g) = rrt_star(GraphHeuristicParameters {
            start,
            end: Goal::Vertex(goal),
            workspace,
            execution_manager: ma,
            obstacles: &obstacles,
            moving_radius,
            base_rewire_radius: rewire_radius,
            vertices: PhantomData::<(Bsp<NDIM>, CartesianTopology<NDIM, EuclidianDistance>)>,
        });
        dbg!(g.nb_links());
        dbg!(a.is_some());
    });
    // test_convergence(
    //     &mut benchmark,
    //     "rrt_star_shortcut",
    //     &times,
    //     workspace,
    //     |ma| {
    //         let (a, g) = rrt_star(GraphHeuristicParameters {
    //             start,
    //             end: Goal::Vertex(goal),
    //             workspace,
    //             execution_manager: ShortcutManager {
    //                 workspace,
    //                 inner: ma,
    //                 obstacles: &obstacles,
    //             },
    //             obstacles: &obstacles,
    //             moving_radius,
    //             base_rewire_radius: rewire_radius,
    //             vertices: PhantomData::<(Bsp<NDIM>, CartesianTopology<NDIM, EuclidianDistance>)>,
    //         });
    //         dbg!(g.nb_links());
    //  dbg!(a.is_some());

    //     },
    // );
    // test_convergence(&mut benchmark, "rrt_nobsp", &times, workspace, |ma| {
    //     let (a, g) = rrt(GraphHeuristicParameters {
    //         start,
    //         end: Goal::Vertex(goal),
    //         workspace,
    //         execution_manager: ma,
    //         obstacles: &obstacles,
    //         moving_radius,
    //         base_rewire_radius: rewire_radius,
    //         vertices: PhantomData::<(Vec<VecN<NDIM, f64>>, CartesianTopology<NDIM, EuclidianDistance>)>,
    //     });
    //     dbg!(g.nb_links());
    //     dbg!(a.is_some());
    // });
    // test_convergence(&mut benchmark, "rrt_star_nobsp", &times, workspace, |ma| {
    //     let (a, g) = rrt_star(GraphHeuristicParameters {
    //         start,
    //         end: Goal::Vertex(goal),
    //         workspace,
    //         execution_manager: ma,
    //         obstacles: &obstacles,
    //         moving_radius,
    //         base_rewire_radius: rewire_radius,
    //         vertices: PhantomData::<(Vec<VecN<NDIM, f64>>, CartesianTopology<NDIM, EuclidianDistance>)>,
    //     });
    //     dbg!(g.nb_links());
    //     dbg!(a.is_some());
    // });
}
