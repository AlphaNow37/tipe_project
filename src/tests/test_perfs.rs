use std::path::Path;

use rand::{distr::Distribution, Rng};

use crate::{
    geometry::{polygon_map_generator::gen_pol_map_square, shapes::Polygon},
    graphs::Graph,
    path_planning::visibility_graph::{
        compute_vis_graph_cachemap, compute_vis_graph_fullmap, vis_graph_naive, vis_graph_opt1,
    },
    utils::benchmark::{time_bench, Benchmark},
};

use super::{giggle_coords, out_dir};

struct Param {
    polys: Vec<Polygon>,
    start: (usize, usize),
    end: (usize, usize),
}

pub fn test_perf() {
    let mut benchmark: Benchmark<Param, f64, usize> =
        Benchmark::new(|param: &Param| param.polys.iter().map(|p| p.0.len()).sum::<usize>());

    benchmark.add_func(
        time_bench(|param: &Param| {
            compute_vis_graph_fullmap(&param.polys, vis_graph_naive).a_star_with(
                param.start,
                param.end,
                |(i, j)| param.polys[i].0[j],
            );
        }),
        "naive_full",
    );

    benchmark.add_func(
        time_bench(|param: &Param| {
            compute_vis_graph_fullmap(&param.polys, vis_graph_opt1).a_star_with(
                param.start,
                param.end,
                |(i, j)| param.polys[i].0[j],
            );
        }),
        "opt1_full",
    );

    benchmark.add_func(
        time_bench(|param: &Param| {
            compute_vis_graph_cachemap(&param.polys, vis_graph_naive).a_star_with(
                param.start,
                param.end,
                |(i, j)| param.polys[i].0[j],
            );
        }),
        "naive_cache",
    );

    benchmark.add_func(
        time_bench(|param: &Param| {
            compute_vis_graph_cachemap(&param.polys, vis_graph_opt1).a_star_with(
                param.start,
                param.end,
                |(i, j)| param.polys[i].0[j],
            );
        }),
        "opt1_cache",
    );

    let mut rng = rand::rng();
    for n in 1..30 {
        dbg!(n);
        let mut obs = gen_pol_map_square(n, 10000., n * n * 12 / 10);
        giggle_coords(&mut obs);
        let distr = rand::distr::weighted::WeightedIndex::new(obs.iter().map(|p| p.0.len().pow(2)))
            .unwrap();
        let start_i = distr.sample(&mut rng);
        let start_j = rng.random_range(0..obs[start_i].0.len());
        let end_i = distr.sample(&mut rng);
        let end_j = rng.random_range(0..obs[end_i].0.len());
        benchmark.add_param(&Param {
            polys: obs,
            start: (start_i, start_j),
            end: (end_i, end_j),
        });
        benchmark
            .write_to_file(&out_dir().join("perf_benchmark.json"))
            .unwrap();
    }

    benchmark
        .write_to_file(&out_dir().join("perf_benchmark.json"))
        .unwrap();
}
