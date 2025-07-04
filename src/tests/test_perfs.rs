use std::path::Path;

use crate::{
    geometry::{polygon_map_generator::gen_pol_map_square, shapes::Polygon},
    path_planning::visibility_graph::{compute_vis_graph, compute_vis_graph_naive},
    utils::benchmark::{time_bench, Benchmark},
};

use super::{giggle_coords, out_dir};

struct Param {
    polys: Vec<Polygon>,
}

pub fn test_perf() {
    let mut benchmark: Benchmark<Param, f64, usize> =
        Benchmark::new(|param: &Param| param.polys.iter().map(|p| p.0.len()).sum::<usize>());

    benchmark.add_func(
        time_bench(|param: &Param| {
            compute_vis_graph_naive(&param.polys);
        }),
        "naive",
    );

    benchmark.add_func(
        time_bench(|param: &Param| {
            compute_vis_graph(&param.polys);
        }),
        "opt1",
    );

    for n in 1..30 {
        dbg!(n);
        let mut obs = gen_pol_map_square(n, 10000., n * n * 12 / 10);
        giggle_coords(&mut obs);
        benchmark.add_param(&Param { polys: obs });
    }

    benchmark
        .write_to_file(&out_dir().join("perf_benchmark.json"))
        .unwrap();
}
