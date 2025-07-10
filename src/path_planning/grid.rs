use crate::geometry::shapes::Cube;
use crate::graphs::{Grid, SubGraph};

type SpaceGrid<const N: usize> = SubGraph<usize, Grid<N>>;

pub fn make_grid_graph_cubes<const N: usize>(obstacles: Vec<Cube<N>>) -> SpaceGrid<N> {
    todo!()
}
