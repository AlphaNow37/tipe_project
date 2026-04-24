/// A benchmark for performances
use rand::{distr::Distribution, Rng};

use super::{giggle_coords, out_dir};
use crate::parallel::{
    compute_vis_graph_gpu_adjacencymatrix, compute_vis_graph_gpu_edgelist,
};
use crate::{
    geometry::{polygon_map_generator::gen_pol_map_square, shapes::Polygon},
    graphs::Graph,
    path_planning::visibility_graph::{
        compute_vis_graph_cachemap, compute_vis_graph_fullmap, vis_graph_naive, vis_graph_opt1,
    },
    utils::benchmark::{time_bench, Benchmark},
    workspace::cartesians::{CartesianTopology, EuclidianDistance},
};
use crate::libs::l_polyanya::shortest_path_polyanya;

const WORKSPACE: CartesianTopology<2, EuclidianDistance> =
    CartesianTopology::new_borderless(EuclidianDistance);

struct Param {
    polys: Vec<Polygon>,
    start: (usize, usize),
    end: (usize, usize),
    nb_pts: usize,
}

fn run_naive_full(param: &Param) {
    compute_vis_graph_fullmap(&param.polys, vis_graph_naive).a_star_with(
        param.start,
        param.end,
        |(i, j)| param.polys[i].0[j],
        &WORKSPACE,
    );
}
fn run_opt1_full(param: &Param) {
    compute_vis_graph_fullmap(&param.polys, vis_graph_opt1).a_star_with(
        param.start,
        param.end,
        |(i, j)| param.polys[i].0[j],
        &WORKSPACE,
    );
}
fn run_naive_cache(param: &Param) {
    compute_vis_graph_cachemap(&param.polys, vis_graph_naive).a_star_with(
        param.start,
        param.end,
        |(i, j)| param.polys[i].0[j],
        &WORKSPACE,
    );
}
fn run_opt1_cache(param: &Param) {
    compute_vis_graph_cachemap(&param.polys, vis_graph_opt1).a_star_with(
        param.start,
        param.end,
        |(i, j)| param.polys[i].0[j],
        &WORKSPACE,
    );
}

fn run_naive_gpu_mat(param: &Param) {
    compute_vis_graph_gpu_adjacencymatrix(&param.polys).a_star_with(
        param.start,
        param.end,
        |(i, j)| param.polys[i].0[j],
        &WORKSPACE,
    );
}

fn run_naive_gpu_elist(param: &Param) {
    compute_vis_graph_gpu_edgelist(&param.polys, param.nb_pts * 50).a_star_with(
        param.start,
        param.end,
        |(i, j)| param.polys[i].0[j],
        &WORKSPACE,
    );
}

fn run_polyanya_lib(param: &Param) {
    shortest_path_polyanya(
        &param.polys,
        param.polys[param.start.0].0[param.start.1],
        param.polys[param.end.0].0[param.end.1],
    );
}

pub fn test_perf() {
    let mut benchmark: Benchmark = Benchmark::new(out_dir().join("perf_benchmark.csv"));

    benchmark.add_row(vec![
        "map_width".to_string(),
        "map_nb_vertices".to_string(),
        // "time_naive_full".to_string(),
        // "time_opt1_full".to_string(),
        // "time_naive_cache".to_string(),
        // "time_opt1_cache".to_string(),
        // "time_naive_gpu_mat".to_string(),
        // "time_naive_gpu_elist".to_string(),
        "time_polyanya_lib".to_string()
    ]);

    let mut rng = rand::rng();
    for n in 1..100 {
        dbg!(n);
        let nmerges = n * n * 12 / 10; // Gives nice maps
        let mut obs = gen_pol_map_square(n, 10000., nmerges);
        giggle_coords(&mut obs);
        let distr = rand::distr::weighted::WeightedIndex::new(obs.iter().map(|p| p.0.len().pow(2)))
            .unwrap();
        let start_i = distr.sample(&mut rng);
        let start_j = rng.random_range(0..obs[start_i].0.len());
        let end_i = distr.sample(&mut rng);
        let end_j = rng.random_range(0..obs[end_i].0.len());
        let npts = obs.iter().map(|p| p.0.len()).sum::<usize>();
        let params = Param {
            polys: obs,
            start: (start_i, start_j),
            end: (end_i, end_j),
            nb_pts: npts,
        };
        let mut row = vec![n.to_string(), npts.to_string()];
        for f in [
            // run_naive_full,
            // run_opt1_full,
            // run_naive_cache,
            // run_opt1_cache,
            // run_naive_gpu_mat,
            // run_naive_gpu_elist,
            run_polyanya_lib,
        ] {
            let time = time_bench(f)(&params);
            row.push(time.to_string());
        }
        benchmark.add_row(row);
    }
}
