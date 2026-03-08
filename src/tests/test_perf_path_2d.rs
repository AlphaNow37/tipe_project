/// A benchmark for performances
use rand::{distr::Distribution, Rng};

use super::{giggle_coords, out_dir};
use crate::parallel::vis_graphs::compute_vis_graph_gpu;
use crate::{
    geometry::{polygon_map_generator::gen_pol_map_square, shapes::Polygon},
    graphs::Graph,
    path_planning::visibility_graph::{
        compute_vis_graph_cachemap, compute_vis_graph_fullmap, vis_graph_naive, vis_graph_opt1,
    },
    utils::benchmark::{time_bench, Benchmark},
    workspace::cartesians::{CartesianTopology, EuclidianDistance},
};

const WORKSPACE: CartesianTopology<2, EuclidianDistance> =
    CartesianTopology::new_borderless(EuclidianDistance);

struct Param {
    polys: Vec<Polygon>,
    start: (usize, usize),
    end: (usize, usize),
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

fn run_naive_gpu(param: &Param) {
    compute_vis_graph_gpu(&param.polys).a_star_with(
        param.start,
        param.end,
        |(i, j)| param.polys[i].0[j],
        &WORKSPACE,
    );
}

pub fn test_perf() {
    let mut benchmark: Benchmark = Benchmark::new(out_dir().join("perf_benchmark.csv"));

    benchmark.add_row(vec![
        "map_width".to_string(),
        "map_nb_vertices".to_string(),
        // "time_naive_full".to_string(),
        "time_opt1_full".to_string(),
        // "time_naive_cache".to_string(),
        // "time_opt1_cache".to_string(),
        "time_naive_gpu".to_string(),
    ]);

    let mut rng = rand::rng();
    for n in 1..50 {
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
        let params = Param {
            polys: obs,
            start: (start_i, start_j),
            end: (end_i, end_j),
        };

        let mut row = vec![
            n.to_string(),
            params
                .polys
                .iter()
                .map(|p| p.0.len())
                .sum::<usize>()
                .to_string(),
        ];
        for f in [
            // run_naive_full,
            run_opt1_full,
            // run_naive_cache,
            // run_opt1_cache,
            run_naive_gpu,
        ] {
            let time = time_bench(f)(&params);
            row.push(time.to_string());
        }
        benchmark.add_row(row);
    }
}
