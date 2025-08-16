mod rtree {
    use crate::datastructures::r_tree::RTree;
    use crate::geometry::shapes::Cube;
    use crate::geometry::VecN;
    use crate::svg::object::Style;
    use crate::svg::rtree::put_rtree;
    use crate::svg::SvgGroup;
    use crate::tests::out_dir;
    use rand::{rng, Rng};
    use std::array::from_fn;

    const NCUBES: usize = 10;
    const TOTAL_WIDTH: f64 = 200.0;
    const MIN_WIDTH: f64 = 10.;
    const MAX_WIDTH: f64 = 15.;

    pub fn test_rtree_2d() {
        let mut rng = rng();

        let mut cubes = (0..NCUBES)
            .map(|_| {
                let p1: VecN<2, f64> = VecN(from_fn(|_| {
                    rng.random_range(0.0..(TOTAL_WIDTH - MAX_WIDTH))
                }));
                let p2: VecN<2, f64> =
                    p1 + VecN(from_fn(|_| rng.random_range(MIN_WIDTH..MAX_WIDTH)));
                Cube::from_point(p1).with_point(p2)
            })
            .collect::<Vec<_>>();
        let rtree = RTree::bulk_load(&mut cubes);

        let mut svg = SvgGroup::default();

        put_rtree(
            &mut svg,
            &rtree,
            Style::stroke("black", 0.6).with_fill("none"),
            0.,
            Some(&|svg: &mut SvgGroup, c: &Cube<2>| svg.push(*c, 1., Style::fill("red"))),
        );

        svg.write_to_file(&out_dir().join("test_rtree.svg"));
    }
}

mod grid {
    use crate::geometry::shapes::Cube;
    use crate::geometry::VecN;
    use crate::path_planning::accessibility_grid::AccesibilityGrid;
    use crate::svg::grid::put_grid;
    use crate::svg::object::Style;
    use crate::svg::SvgGroup;
    use crate::tests::out_dir;
    use crate::utils::numbers::F64_EPSILON;
    use crate::workspace::cartesians::{EuclidianDistance, CartesianTopology};
    use rand::{rng, Rng};
    use std::array::from_fn;

    const NCUBES: usize = 10;
    const TOTAL_WIDTH: f64 = 200.0;
    const MIN_WIDTH: f64 = 20.;
    const MAX_WIDTH: f64 = 50.;
    const RESOLUTION: f64 = 12.;

    pub fn test_grid_2d() {
        let mut rng = rng();

        let cubes = (0..NCUBES)
            .map(|_| {
                let p1: VecN<2, f64> = VecN(from_fn(|_| {
                    rng.random_range(0.0..(TOTAL_WIDTH - MAX_WIDTH))
                }));
                let p2: VecN<2, f64> =
                    p1 + VecN(from_fn(|_| rng.random_range(MIN_WIDTH..MAX_WIDTH)));
                Cube::from_point(p1).with_point(p2)
            })
            .collect::<Vec<_>>();

        let grid = AccesibilityGrid::new_with_painting(
            &cubes,
            RESOLUTION,
            Cube {
                start: VecN::splat(0.),
                end: VecN::splat(TOTAL_WIDTH),
            },
        );

        let mut svg = SvgGroup::default();

        for c in &cubes {
            svg.push(*c, 1., Style::fill("red"))
        }

        put_grid(
            &mut svg,
            &grid,
            0.,
            Style::stroke("black", 0.1).with_fill("none"),
            Style::fill("#AAAAAA"),
        );

        let size = grid.grid_size();
        let workspace = CartesianTopology::new_borderless(EuclidianDistance);
        if let Some((path, _)) = grid.shortest_path(
            grid.bounding_box.start,
            grid.bounding_box.end - VecN::splat(F64_EPSILON),
            workspace,
        ) {
            for pos in path {
                svg.push(
                    Cube {
                        start: pos - size / 2.,
                        end: pos + size / 2.,
                    },
                    2.,
                    Style::fill("#00AA00"),
                );
            }
        }

        svg.write_to_file(&out_dir().join("test_grid.svg"));
    }
}

pub use grid::test_grid_2d;
pub use rtree::test_rtree_2d;
