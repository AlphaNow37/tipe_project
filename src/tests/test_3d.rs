use crate::geometry::shapes::Cube;
use crate::geometry::VecN;
use crate::render_3d::cubes::place_cubes;
use lib_space_animation::world::primitives::color::Color;
use lib_space_animation::world::world_builder::WorldsBuilder;

pub fn test_3d() {
    lib_space_animation::run(|| {
        let worlds = WorldsBuilder::default();

        let mut world = worlds.add_world(0);
        place_cubes(
            &mut world,
            &[Cube::from_point(VecN([0., 0., 0.])).with_point(VecN([2., 5., 3.]))],
            Color::RED,
        );

        let worlds = world.finalize();
        worlds
    });
}
