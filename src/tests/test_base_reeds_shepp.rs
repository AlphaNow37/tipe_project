use super::out_dir;
use crate::utils::numbers::Zero;
/// A very simple test for svg
use crate::{
    geometry::{angles::Angle, shapes::Cube, VecN},
    svg::{self, curves::put_reeds_shepp, object::Style},
    workspace::{reeds_shepp::ReedsSheppWorkspace, workspace::WorkspaceTopology},
};
use rand::{rng, Rng};
use std::f64::consts::TAU;

pub fn test_base_reeds_shepp() {
    let mut svg = svg::SvgGroup::default();

    let points = [
        (
            (VecN([0., 0.]), Angle::from_degrees(0.)),
            (VecN([0., 1.]), Angle::from_degrees(180.)),
            "red",
            1.,
        ),
        // (
        //     (VecN([0., 0.]), Angle::from_degrees(0.)),
        //     (VecN([10., 0.]), Angle::from_degrees(0.)),
        //     "red",
        //     1.,
        // ),
        // (
        //     (VecN([0., 0.]), Angle::from_degrees(0.)),
        //     (VecN([0., 10.]), Angle::from_degrees(0.)),
        //     "blue",
        //     2.,
        // ),
        // (
        //     (VecN([0., 0.]), Angle::from_degrees(0.)),
        //     (VecN([10., 10.]), Angle::from_degrees(0.)),
        //     "yellow",
        //     1.,
        // ),
        // (
        //     (VecN([0., 0.]), Angle::from_degrees(0.)),
        //     (VecN([10., 10.]), Angle::from_degrees(0.)),
        //     "yellow",
        //     1.,
        // ),
        // (
        //     (VecN([0., 0.]), Angle::from_degrees(0.)),
        //     (VecN([0., -10.]), Angle::from_degrees(90.)),
        //     "green",
        //     4.,
        // ),
        // (
        //     (VecN([0., 0.]), Angle::from_degrees(0.)),
        //     (VecN([-5., 0.]), Angle::from_degrees(180.)),
        //     "purple",
        //     1.,
        // ),
        // (
        //     (VecN([0., 0.]), Angle::from_degrees(0.)),
        //     (VecN([2., 2.]), Angle::from_degrees(-90.)),
        //     "orange",
        //     1.,
        // ),
    ];

    let mut points = Vec::new();
    let mut rng = rng();
    for i in 0..20 {
        let x = rng.random_range(-4.0..4.0);
        let y = rng.random_range(-4.0..4.0);
        let theta = rng.random_range(0.0..TAU);
        points.push((
            (VecN([0., 0.]), Angle::ZERO),
            (VecN([x, y]), Angle::new(theta)),
            "white",
            1.,
        ));
    }

    for (start, end, color, radius) in points {
        let workspace = ReedsSheppWorkspace::new_borderless(radius, true);
        let segment = workspace.segment(start, end);
        // dbg!(segment);
        svg.push(
            Cube::from_point(start.0 - VecN([0.1, 0.1])).with_point(start.0 + VecN([0.1, 0.1])),
            -1.,
            Style::fill("white"),
        );
        svg.push(
            Cube::from_point(end.0 - VecN([0.1, 0.1])).with_point(end.0 + VecN([0.1, 0.1])),
            -1.,
            Style::fill("black"),
        );
        for i in 0..25 {
            let f = i as f64 / 25.;
            let pt = workspace.lerp(segment, f);
            svg.push(
                Cube::from_point(pt.0 - VecN([0.1, 0.1])).with_point(pt.0 + VecN([0.1, 0.1])),
                -1.,
                Style::fill("green"),
            );
        }

        put_reeds_shepp(
            &mut svg,
            Style::stroke(color, 0.05).with_fill("none"),
            segment,
            0.,
        );
    }

    svg.write_to_file(&out_dir().join("test_base_reeds_shepp.svg"));
}
