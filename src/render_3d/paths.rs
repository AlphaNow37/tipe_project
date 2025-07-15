use crate::geometry::VecN;
use crate::utils::traits::NormedSpace;
use lib_space_animation::math::{Transform, Vec3};
use lib_space_animation::world::primitives::color::Color;
use lib_space_animation::world::visuals::Pipe;
use lib_space_animation::world::world_builder::WorldBuilder;

pub fn place_path(
    world: &mut WorldBuilder,
    path: &[VecN<3, f64>],
    color: Color,
    width: f64,
    glob_tr: Transform,
) {
    let col_ref = world.push(color);
    let glob_tr_ref = world.push(glob_tr);
    world.push_visual((col_ref, glob_tr_ref));

    for i in 0..(path.len() - 1) {
        let a = path[i];
        let b = path[i + 1];
        let tr = world.push(
            Transform::from_transv((a / 2. + b / 2.).into())
                * Transform::from_z_looking_at(Vec3::from((b - a).map_component(|c| c as f32).0))
                * Transform::from_scalef(
                    width as f32 / 2.,
                    width as f32 / 2.,
                    a.distance(b) as f32,
                ),
        );
        world.push_visual(Pipe(tr));
    }
}
