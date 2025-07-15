use crate::geometry::shapes::Cube;
use crate::geometry::VecN;
use crate::path_planning::accessibility_grid::AccesibilityGrid;
use crate::render_3d::cubes::place_cubes;
use crate::render_3d::grid::place_grid;
use crate::render_3d::paths::place_path;
use lib_space_animation::math::trans;
use lib_space_animation::world::primitives::color::Color;
use lib_space_animation::world::world_builder::WorldsBuilder;

pub fn test_3d() {
    let cubes = vec![
        Cube::from_point(VecN([-0.1, -0.1, -0.1])).with_point(VecN([4.1, 1.1, 0.1])),
        Cube::from_point(VecN([-0.1, -0.1, -0.1])).with_point(VecN([4.1, 0.1, 1.6])),
        Cube::from_point(VecN([-0.1, -0.1, -0.1])).with_point(VecN([0.1, 1.1, 1.6])),
        Cube::from_point(VecN([-0.1, 1.1, -0.1])).with_point(VecN([4.1, 0.9, 1.6])),
        Cube::from_point(VecN([4.1, -0.1, -0.1])).with_point(VecN([3.9, 1.1, 1.6])),
        Cube::from_point(VecN([0.95, 0., 0.])).with_point(VecN([1.05, 0.75, 1.5])),
        Cube::from_point(VecN([1.95, 0.25, 0.])).with_point(VecN([2.05, 1., 1.5])),
        Cube::from_point(VecN([2.95, 0., 0.35])).with_point(VecN([3.05, 1., 1.5])),
    ];

    lib_space_animation::run(move || {
        let worlds = WorldsBuilder::default();

        let mut world = worlds.add_world(0);
        let obstacles_tr = trans(-2., 0., -2.);

        place_cubes(&mut world, &cubes[..5], Color::BLUE, obstacles_tr, true);
        place_cubes(&mut world, &cubes[5..], Color::RED, obstacles_tr, true);

        let mut cubes2 = cubes.clone();
        let grid = AccesibilityGrid::new_with_rtree(&mut cubes2, 0.04);
        place_grid(&mut world, &grid, obstacles_tr);

        match grid.shortest_path(VecN([0.2, 0.4, 0.4]), VecN([3.8, 0.4, 0.4])) {
            None => println!("No path found !"),
            Some((path, length)) => {
                place_path(&mut world, &path, Color::YELLOW, 0.05, obstacles_tr);
            }
        }

        let tr_axis = world.push(trans(0., 0., 2.));
        lib_space_animation::models::put_axis(&mut world, tr_axis);

        let worlds = world.finalize();
        worlds
    });
}
