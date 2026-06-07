use crate::geometry::VecN;
use crate::graphs::IterableGraph;
use crate::workspace::cartesians::{EuclidianDistance, Length};
use lib_space_animation::math::{Transform, Vec3};
use lib_space_animation::world::primitives::color::Color;
use lib_space_animation::world::visuals::Pipe;
use lib_space_animation::world::world_builder::WorldBuilder;
use std::collections::{HashMap, HashSet};
use std::hash::Hash;


/// Place un graphe sur le monde
pub fn place_graph<V: Hash + Eq + Copy, I: Into<V>, G: IterableGraph<V, I>>(
    world: &mut WorldBuilder,
    graph: &G,
    pos: impl Fn(V) -> VecN<3, f64>,
    color: Color,
    width: f64,
    glob_tr: Transform,
) {
    let col_ref = world.push(color);
    let glob_tr_ref = world.push(glob_tr);
    world.push_visual((col_ref, glob_tr_ref));

    let mut placed = HashSet::new();

    for v in graph.iter() {
        let pos_v = pos(v);
        for n in graph.neighbors(v) {
            let n = n.into();
            if placed.contains(&(n, v)) {
                continue;
            }
            let pos_n = pos(n);
            placed.insert((v, n));

            let tr = world.push(
                Transform::from_transv((pos_v).into())
                    * Transform::from_z_looking_at(Vec3::from(
                        (pos_n - pos_v).map_component(|c| c as f32).0,
                    ))
                    * Transform::from_scalef(
                        width as f32 / 2.,
                        width as f32 / 2.,
                        EuclidianDistance.length(pos_v - pos_n) as f32,
                    ),
            );
            world.push_visual(Pipe(tr));
        }
    }
}

pub fn place_graph_populars<V: Hash + Eq + Copy, I: Into<V>, G: IterableGraph<V, I>>(
    world: &mut WorldBuilder,
    graph: &G,
    pos: impl Fn(V) -> VecN<3, f64>,
    color: Color,
    width: f64,
    glob_tr: Transform,
) {
    let col_ref = world.push(color);
    let glob_tr_ref = world.push(glob_tr);
    world.push_visual((col_ref, glob_tr_ref));

    let mut placed = HashSet::new();
    let mut n_count = HashMap::new();

    for v in graph.iter() {
        for n in graph.neighbors(v) {
            let n = n.into();
            n_count.entry(n).and_modify(|c| *c += 1).or_insert(1);
            n_count.entry(v).and_modify(|c| *c += 1).or_insert(1);
        }
    }

    for v in graph.iter() {
        if *n_count.get(&v).unwrap() <= 1 {
            continue;
        }
        let pos_v = pos(v);
        for n in graph.neighbors(v) {
            let n = n.into();
            if *n_count.get(&v).unwrap() <= 1 {
                continue;
            }
            if placed.contains(&(n, v)) {
                continue;
            }

            let pos_n = pos(n);
            placed.insert((v, n));

            let tr = world.push(
                Transform::from_transv((pos_v).into())
                    * Transform::from_z_looking_at(Vec3::from(
                        (pos_n - pos_v).map_component(|c| c as f32).0,
                    ))
                    * Transform::from_scalef(
                        width as f32 / 2.,
                        width as f32 / 2.,
                        EuclidianDistance.length(pos_v - pos_n) as f32,
                    ),
            );
            world.push_visual(Pipe(tr));
        }
    }
}
