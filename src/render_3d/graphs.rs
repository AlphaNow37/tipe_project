use crate::workspace::cartesians::{EuclidianDistance, Length};
use crate::geometry::VecN;
use crate::graphs::IterableGraph;
use lib_space_animation::math::{Transform, Vec3};
use lib_space_animation::world::primitives::color::Color;
use lib_space_animation::world::visuals::Pipe;
use lib_space_animation::world::world_builder::WorldBuilder;
use std::collections::HashSet;
use std::hash::Hash;

pub fn place_graph<V: Hash + Eq + Copy, G: IterableGraph<V>>(
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
