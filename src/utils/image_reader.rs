use crate::geometry::shapes::Cube;
use crate::geometry::VecN;
use crate::path_planning::accessibility_grid::AccesibilityGrid;
use crate::utils::numbers::Zero;
use std::path::Path;

/// Read an image formed by the image_converter tool
pub fn read_image(input_path: &Path) -> (AccesibilityGrid<2>, VecN<2, f64>, VecN<2, f64>) {
    let content = std::fs::read_to_string(input_path).expect("Failed to read the file");

    let [xstart_line, ystart_line, xend_line, yend_line, rest]: [&str; 5] = content
        .trim()
        .splitn(5, "\n")
        .collect::<Vec<&str>>()
        .try_into()
        .unwrap();

    let grid = rest
        .trim()
        .lines()
        .map(|line| line.chars().map(|c| c == '1').collect::<Vec<_>>())
        .collect::<Vec<_>>();

    let acces_grid = AccesibilityGrid::new_with_check(
        |VecN([x, y])| grid[y][x],
        Cube {
            start: VecN::ZERO,
            end: VecN([grid[0].len(), grid.len()]).map_component(|c| c as f64),
        },
    );

    let [x1, y1, x2, y2] = [xstart_line, ystart_line, xend_line, yend_line]
        .map(|l| l.trim().parse::<usize>().expect("Failed to parse to int") as f64);

    (acces_grid, VecN([x1, y1]), VecN([x2, y2]))
}
