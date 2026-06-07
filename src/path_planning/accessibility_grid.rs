use crate::datastructures::r_tree::RTree;
use crate::geometry::shapes::Cube;
use crate::geometry::VecN;
use crate::graphs::{Graph, Grid};
use crate::utils::numbers::F64_EPSILON;
use crate::workspace::cartesians::{CartesianTopology, Length};

/// A N-dimensionnal graph using a grid
/// The grid manages the conversions usize<->coords
pub struct AccesibilityGrid<const N: usize> {
    pub grid: Grid<N>,
    pub accessible: Vec<bool>,
    pub bounding_box: Cube<N>,
    pub resolution: f64,
}
impl<const N: usize> AccesibilityGrid<N> {
    fn get_grid(bbox: Cube<N>, resolution: f64) -> Grid<N> {
        let sizes = bbox.size().map(|w| (w / resolution).ceil() as usize);
        Grid::new(sizes)
    }
    pub fn new_with_check(accessible_f: impl Fn(VecN<N, usize>) -> bool, bounding_box: Cube<N>) -> Self {
        let resolution = 1.;
        let grid = Self::get_grid(bounding_box, resolution);
        let accessible = vec![false; grid.size];

        let mut res = Self {
            accessible,
            grid,
            bounding_box,
            resolution,
        };

        for i in 0..res.grid.size {
            let coords = res.grid.coords(i);
            if accessible_f(coords) {
                res.accessible[i] = true;
            }
        }

        res
    }
    /// Crée une grille depuis un r-tree d'obstacles
    pub fn new_with_rtree(
        tree: &RTree<N, Cube<N>>,
        resolution: f64,
        bounding_box: Cube<N>,
    ) -> Self {
        let grid = Self::get_grid(bounding_box, resolution);
        let accessible = vec![true; grid.size];

        let mut res = Self {
            accessible,
            grid,
            bounding_box,
            resolution,
        };

        for i in 0..res.grid.size {
            let start_int = res.grid.coords(i);
            let end_int = start_int + VecN::from_fn(|_| 1);
            let cube = Cube {
                start: res.position_flaot_from_int(start_int),
                end: res.position_flaot_from_int(end_int),
            };
            if tree.intersect_cube(cube) {
                res.accessible[i] = false;
            }
        }

        res
    }
    
    /// Crée une grille en "peignant" des obstacles
    pub fn new_with_painting(
        cubes: &[Cube<N>],
        resolution: f64,
        mut bounding_box: Cube<N>,
    ) -> Self {
        bounding_box.end = bounding_box.end + VecN::from_fn(|_| F64_EPSILON);
        let grid = Self::get_grid(bounding_box, resolution);
        let accessible = vec![true; grid.size];

        let mut res = Self {
            accessible,
            grid,
            bounding_box,
            resolution,
        };

        for &cube in cubes {
            for i in res.grid.iter_cube(
                res.position_int_from_float(cube.start),
                res.position_int_from_float(cube.end),
            ) {
                res.accessible[i] = false;
            }
        }

        res
    }

    pub fn position_int_from_float(&self, pos: VecN<N, f64>) -> VecN<N, usize> {
        VecN::from_fn(|i| {
            let delta = pos[i] - self.bounding_box.start[i];
            let i_float = delta / (self.bounding_box.end[i] - self.bounding_box.start[i])
                * (self.grid.sizes[i] as f64);
            i_float.floor() as usize
        })
    }
    pub fn position_flaot_from_int(&self, pos: VecN<N, usize>) -> VecN<N, f64> {
        VecN::from_fn(|i| {
            let i_float = pos[i] as f64 + 0.5;
            let delta = i_float * (self.bounding_box.end[i] - self.bounding_box.start[i])
                / (self.grid.sizes[i] as f64);
            self.bounding_box.start[i] + delta
        })
    }
    pub fn grid_size(&self) -> VecN<N, f64> {
        VecN::from_fn(|i| {
            (self.bounding_box.end[i] - self.bounding_box.start[i]) / (self.grid.sizes[i] as f64)
        })
    }

    pub fn contains_point(&self, pos: VecN<N, f64>) -> bool {
        if !self.bounding_box.contains_point(pos) {
            return true;
        }
        let coords = self.position_int_from_float(pos);
        let index = self.grid.index(coords);
        !self.accessible[index]
    }
    
    /// Calcule le chemin le plus court via a*
    pub fn shortest_path(
        &self,
        start: VecN<N, f64>,
        end: VecN<N, f64>,
        workspace: CartesianTopology<N, impl Length<N>>,
    ) -> Option<(Vec<VecN<N, f64>>, f64)> {
        let start_idx = self.grid.index(self.position_int_from_float(start));
        let end_idx = self.grid.index(self.position_int_from_float(end));

        self.grid
            .subgraph(|_, b| self.accessible[*b])
            .a_star_with(
                start_idx,
                end_idx,
                |i| self.position_flaot_from_int(self.grid.coords(i)),
                &workspace,
            )
            .map(|(path, length)| {
                (
                    path.into_iter()
                        .map(|i| self.position_flaot_from_int(self.grid.coords(i)))
                        .collect(),
                    length,
                )
            })
    }
}
