use lib_space_animation::math::Transform;
use lib_space_animation::world::primitives::color::Color;
use lib_space_animation::world::world_builder::WorldBuilder;
use crate::geometry::shapes::Cube;
use crate::geometry::VecN;
use crate::path_planning::accessibility_grid::AccesibilityGrid;
use crate::render_3d::cubes::place_cubes;

const MINI_CUBE_SIZE_FACTOR: f64 = 0.05;

pub fn place_grid(world: &mut WorldBuilder, grid: &AccesibilityGrid<3>, glob_tr: Transform) {
    place_cubes(world, &[grid.bounding_box], Color::BLUE, glob_tr, false);

    let mut acc = Vec::new();
    let mut not_acc = Vec::new();
    let delta_mini_cube = VecN::from_fn(|_| grid.resolution * MINI_CUBE_SIZE_FACTOR);
    for i in 0..grid.grid.size {
        let coords = grid.grid.coords(i);
        let pos = grid.position_flaot_from_int(coords);
        let cube = Cube {
            start: pos - delta_mini_cube,
            end: pos + delta_mini_cube,
        };
        if grid.accessible[i] {
            acc.push(cube)
        } else {
            not_acc.push(cube);
        }
    }

    place_cubes(world, &acc, Color::GREEN, glob_tr, true);
    place_cubes(world, &not_acc, Color::RED, glob_tr, true);
}
