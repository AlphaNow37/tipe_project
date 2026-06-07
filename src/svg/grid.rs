use crate::geometry::shapes::Cube;
use crate::path_planning::accessibility_grid::AccesibilityGrid;
use crate::svg::object::Style;
use crate::svg::SvgGroup;

/// Draws an accessibility grid on the svg
pub fn put_grid(
    svg: &mut SvgGroup,
    grid: &AccesibilityGrid<2>,
    height: f64,
    acc_style: Style,
    unacc_style: Style,
) {
    let size = grid.grid_size();
    for i in 0..grid.grid.size {
        let coords = grid.grid.coords(i);
        let pos = grid.position_flaot_from_int(coords);
        let cube = Cube {
            start: pos - size / 2.,
            end: pos + size / 2.,
        };
        svg.push(
            cube,
            height,
            if grid.accessible[i] {
                acc_style.clone()
            } else {
                unacc_style.clone()
            },
        )
    }
}
