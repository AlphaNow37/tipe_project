use crate::geometry::shapes::Cube;
use crate::geometry::VecN;
use crate::render_3d::cubes::place_cubes;
use lib_space_animation::math::trans;
use lib_space_animation::world::primitives::color::Color;
use lib_space_animation::world::world_builder::WorldsBuilder;

pub fn test_3d() {
    let cubes = vec![
        Cube::from_point(VecN([-0.1, -0.1, -0.1])).with_point(VecN([4.1, 1.1, 0.1])),
        Cube::from_point(VecN([-0.1, -0.1, -0.1])).with_point(VecN([4.1, 0.1, 1.1])),
        Cube::from_point(VecN([-0.1, -0.1, -0.1])).with_point(VecN([0.1, 1.1, 1.1])),
        Cube::from_point(VecN([-0.1, 1.1, -0.1])).with_point(VecN([4.1, 0.9, 1.1])),
        Cube::from_point(VecN([4.1, -0.1, -0.1])).with_point(VecN([3.9, 1.1, 1.1])),
        Cube::from_point(VecN([0.95, 0., 0.])).with_point(VecN([1.05, 0.75, 1.])),
        Cube::from_point(VecN([1.95, 0.25, 0.])).with_point(VecN([2.05, 1., 1.])),
        Cube::from_point(VecN([2.95, 0., 0.])).with_point(VecN([3.05, 1., 0.5])),
    ];

    lib_space_animation::run(move || {
        let worlds = WorldsBuilder::default();

        let mut world = worlds.add_world(0);
        let obstacles_tr = trans(-2., 0., -2.);

        place_cubes(&mut world, &cubes[..5], Color::BLUE, obstacles_tr);
        place_cubes(&mut world, &cubes[5..], Color::RED, obstacles_tr);

        let tr_axis = world.push(trans(0., 0., 2.));
        lib_space_animation::models::put_axis(&mut world, tr_axis);

        let worlds = world.finalize();
        worlds
    });
}
