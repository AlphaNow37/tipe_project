/// A benchmark for performances
use rand::{distr::Distribution, Rng};
use std::hint::black_box;

use super::{giggle_coords, out_dir};
use crate::geometry::polygon_map_generator::gen_pol_map_global;
use crate::libs::l_polyanya::{
    find_path_polyanya_lib, precompute_polyanya_lib, shortest_path_polyanya_lib,
};
use crate::parallel::{compute_vis_graph_gpu_adjacencymatrix, compute_vis_graph_gpu_edgelist};
use crate::path_planning::polyanya::{
    find_start_goal_idx, polyanya, shortest_path_polyanya, PolyanyaMode,
};
use crate::triangulations::delaunay::make_delaynay;
use crate::triangulations::triangulation_lineaire::triangulate_linear;
use crate::workspace::cartesians::DiscreteCartesianTopology;
use crate::{
    geometry::{polygon_map_generator::gen_pol_map_square, shapes::Polygon},
    graphs::Graph,
    path_planning::visibility_graph::{
        compute_vis_graph_cachemap, compute_vis_graph_fullmap, vis_graph_naive, vis_graph_opt1,
    },
    utils::benchmark::{time_bench, Benchmark},
    workspace::cartesians::{CartesianTopology, EuclidianDistance},
};
use std::time::Duration;

const WORKSPACE: CartesianTopology<2, EuclidianDistance> =
    CartesianTopology::new_borderless(EuclidianDistance);

const MAX_TIME: f64 = 10.;
const NB_RETRY: usize = 1;

fn estimate_nb_edges_vis_g(npts: usize) -> usize {
    npts * 30
}

struct Param {
    polys: Vec<Polygon>,
    start: (usize, usize),
    end: (usize, usize),
    nb_pts: usize,
}

fn run_naive_full(param: &Param) -> f64 {
    time_bench(|| {
        black_box(
            compute_vis_graph_fullmap(&param.polys, vis_graph_naive).a_star_with(
                param.start,
                param.end,
                |(i, j)| param.polys[i].0[j],
                &WORKSPACE,
            ),
        )
    })
}
fn run_opt1_full(param: &Param) -> f64 {
    time_bench(|| {
        black_box(
            compute_vis_graph_fullmap(&param.polys, vis_graph_opt1).a_star_with(
                param.start,
                param.end,
                |(i, j)| param.polys[i].0[j],
                &WORKSPACE,
            ),
        )
    })
}
fn run_naive_cache(param: &Param) -> f64 {
    time_bench(|| {
        black_box(
            compute_vis_graph_cachemap(&param.polys, vis_graph_naive).a_star_with(
                param.start,
                param.end,
                |(i, j)| param.polys[i].0[j],
                &WORKSPACE,
            ),
        )
    })
}
fn run_opt1_cache(param: &Param) -> f64 {
    time_bench(|| {
        black_box(
            compute_vis_graph_cachemap(&param.polys, vis_graph_opt1).a_star_with(
                param.start,
                param.end,
                |(i, j)| param.polys[i].0[j],
                &WORKSPACE,
            ),
        )
    })
}

fn run_naive_gpu_mat(param: &Param) -> f64 {
    time_bench(|| {
        black_box(
            compute_vis_graph_gpu_adjacencymatrix(&param.polys).a_star_with(
                param.start,
                param.end,
                |(i, j)| param.polys[i].0[j],
                &WORKSPACE,
            ),
        )
    })
}

fn run_naive_gpu_elist(param: &Param) -> f64 {
    time_bench(|| {
        black_box(
            compute_vis_graph_gpu_edgelist(&param.polys, estimate_nb_edges_vis_g(param.nb_pts)).a_star_with(
                param.start,
                param.end,
                |(i, j)| param.polys[i].0[j],
                &WORKSPACE,
            ),
        )
    })
}

fn run_astar_only(param: &Param) -> f64 {
    let g = compute_vis_graph_gpu_edgelist(&param.polys, estimate_nb_edges_vis_g(param.nb_pts));
    time_bench(|| {
        black_box(g.a_star_with(
            param.start,
            param.end,
            |(i, j)| param.polys[i].0[j],
            &WORKSPACE,
        ))
    })
}

fn run_polyanya_lib(param: &Param) -> f64 {
    time_bench(|| {
        black_box(shortest_path_polyanya_lib(
            &param.polys,
            param.polys[param.start.0].0[param.start.1],
            param.polys[param.end.0].0[param.end.1],
        ))
    })
}

fn run_polyanya_lib_astar_only(param: &Param) -> f64 {
    let mesh = precompute_polyanya_lib(&param.polys);
    time_bench(move || {
        black_box(find_path_polyanya_lib(
            param.polys[param.start.0].0[param.start.1],
            param.polys[param.end.0].0[param.end.1],
            mesh,
        ))
    })
}

fn run_polyanya_me_astar(param: &Param) -> f64 {
    time_bench(|| {
        black_box(shortest_path_polyanya(
            &param.polys,
            (param.start.0, param.start.1),
            (param.end.0, param.end.1),
            PolyanyaMode::AStar,
        ))
    })
}

fn run_polyanya_me_dijkstra(param: &Param) -> f64 {
    time_bench(|| {
        black_box(shortest_path_polyanya(
            &param.polys,
            (param.start.0, param.start.1),
            (param.end.0, param.end.1),
            PolyanyaMode::Dijkstra,
        ))
    })
}

fn run_polyanya_me_dijkstra_exhaustive(param: &Param) -> f64 {
    time_bench(|| {
        black_box(shortest_path_polyanya(
            &param.polys,
            (param.start.0, param.start.1),
            (param.end.0, param.end.1),
            PolyanyaMode::DijkstraExhaustive,
        ))
    })
}

fn run_polyanya_me_astar_only_nodelaunay(param: &Param) -> f64 {
    let tri = triangulate_linear(&param.polys, 50.);
    time_bench(|| {
        let (new_start, new_goal) = find_start_goal_idx(param.start, param.end, &param.polys, &tri);
        black_box(polyanya(&tri, new_start, new_goal, PolyanyaMode::AStar));
    })
}

fn run_polyanya_me_astar_only(param: &Param) -> f64 {
    let mut tri = triangulate_linear(&param.polys, 50.);
    make_delaynay(&mut tri);
    time_bench(|| {
        let (new_start, new_goal) = find_start_goal_idx(param.start, param.end, &param.polys, &tri);
        black_box(polyanya(&tri, new_start, new_goal, PolyanyaMode::AStar));
    })
}

fn run_polyanya_me_astar_nodelaunay(param: &Param) -> f64 {
    time_bench(|| {
        let tri = triangulate_linear(&param.polys, 50.);
        let (new_start, new_goal) = find_start_goal_idx(param.start, param.end, &param.polys, &tri);
        black_box(polyanya(&tri, new_start, new_goal, PolyanyaMode::AStar));
    })
}

fn tri_me(param: &Param) -> f64 {
    time_bench(|| {
        black_box(triangulate_linear(&param.polys, 50.));
    })
}

fn tri_delaunay_me(param: &Param) -> f64 {
    time_bench(|| {
        let mut tri = triangulate_linear(&param.polys, 50.);
        make_delaynay(&mut tri);
        black_box(tri);
    })
}

fn run_theta_star_tri(param: &Param) -> f64 {
    time_bench(|| {
        let mut tri = triangulate_linear(&param.polys, 50.);
        tri.build_vertex_to_adj_tris();
        let (new_start, new_goal) = find_start_goal_idx(param.start, param.end, &param.polys, &tri);
        let g = tri.to_vertex_graph();
        black_box(g.theta_star_with(
            new_start,
            new_goal,
            |i| i,
            &DiscreteCartesianTopology {
                positions: &tri.vertex_poss,
                dist: EuclidianDistance,
            },
            &tri,
        ))
    })
}

pub fn test_perf() {
    let mut benchmark: Benchmark = Benchmark::new(out_dir().join("perf_benchmark.json"));

    let mut fns = vec![
        // ("naive_full", run_naive_full as (fn(&Param) -> f64)),
        // ("opt1_full", run_opt1_full as fn(&Param) -> f64),
        // ("naive_cache", run_naive_cache),
        // ("opt1_cache", run_opt1_cache),
        // ("naive_gpu_matrix", run_naive_gpu_mat),
        ("naive_gpu_elist", run_naive_gpu_elist as fn(&Param) -> f64),
        // ("graph_astar_only", run_astar_only),
        // ("polyanya_lib", run_polyanya_lib as(fn(&Param) -> f64)),
        // (
        //     "polyanya_lib_astar_only",
        //     run_polyanya_lib_astar_only as fn(&Param) -> f64,
        // ),
        // ("polyanya_me_astar", run_polyanya_me_astar),
        // ("polyanya_me_dijstra", run_polyanya_me_dijkstra),
        // (
        //     "polyanya_me_dijstra_exhaustive",
        //     run_polyanya_me_dijkstra_exhaustive,
        // ),
        // ("polyanya_me_astar_only", run_polyanya_me_astar_only),
        // (
        //     "polyanya_me_astar_nodelaunay",
        //     run_polyanya_me_astar_nodelaunay as fn(&Param) -> f64,
        // ),
        // (
        //     "polyanya_me_astar_only_nodelaunay",
        //     run_polyanya_me_astar_only_nodelaunay,
        // ),
        // ("tri_me", tri_me as fn(&Param) -> f64),
        // ("tri_delaunay_me", tri_delaunay_me),
        // ("theta_star", run_theta_star_tri),
    ];

    let mut rng = rand::rng();
    for n in 1..100 {
        // let npts = n * 100;
        let npts = n*n*4;

        dbg!(n);

        let mut sums = vec![0.; fns.len()];

        for _ in 0..NB_RETRY {
            // std::thread::sleep(Duration::from_secs_f32(2.));
            let nmerges = n * n * 12 / 10; // Gives nice maps
            let mut obs = gen_pol_map_square(n, 10000., nmerges);
            // let mut obs = gen_pol_map_global(npts, 10000.);
            giggle_coords(&mut obs);
            let distr =
                rand::distr::weighted::WeightedIndex::new(obs.iter().map(|p| p.0.len().pow(2)))
                    .unwrap();
            // let start_i = 0;
            // let end_i = 0;
            // let start_j = npts / 8;
            // let end_j = (5 * npts) / 8;
            let start_i = distr.sample(&mut rng);
            let start_j = rng.random_range(0..obs[start_i].0.len());
            let end_i = distr.sample(&mut rng);
            let end_j = rng.random_range(0..obs[end_i].0.len());
            let params = Param {
                polys: obs,
                start: (start_i, start_j),
                end: (end_i, end_j),
                nb_pts: npts,
            };

            for (i, (name, f)) in fns.iter().enumerate() {
                println!("Running: {name}");
                let res = f(&params);
                sums[i] += res;
            }
        }

        let mut new_fns = Vec::new();
        for (i, (name, f)) in fns.iter().enumerate() {
            let avg = sums[i] / (NB_RETRY as f64);
            benchmark.add_datapoint(name, npts as f64, avg);
            if avg <= MAX_TIME {
                new_fns.push((*name, *f))
            } else {
                println!("REMOVING: {name}")
            }
        }
        fns = new_fns;
    }
}
