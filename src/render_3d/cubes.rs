use crate::geometry::shapes::Cube;
use lib_space_animation::math::Transform;
use lib_space_animation::world::primitives::color::Color;
use lib_space_animation::world::visuals::Cube as VisuCube;
use lib_space_animation::world::world_builder::WorldBuilder;

pub fn place_cubes(world: &mut WorldBuilder, cubes: &[Cube<3>], color: Color, glob_tr: Transform) {
    let col = world.push(color);
    let id = world.push(Transform::ID);
    for c in cubes {
        let tr = world.push(
            glob_tr
                * Transform::from_transv((c.start / 2. + c.end / 2.).into())
                * Transform::from_scalev((c.end / 2. - c.start / 2.).into()),
        );
        world.push_visual((id, col, VisuCube(tr)));
    }
}
