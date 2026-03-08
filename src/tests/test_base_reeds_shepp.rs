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
use crate::svg::curves::put_arrow;

pub fn test_base_reeds_shepp() {
    let mut svg = svg::SvgGroup::default();
    svg.no_margin = true;

    let points = [
        // (
        //     (VecN([0., 0.]), Angle::from_degrees(0.)),
        //     (VecN([0., 1.]), Angle::from_degrees(180.)),
        //     "red",
        //     1.,
        // ),
        // (
        //     (VecN([0., 0.]), Angle::from_degrees(0.)),
        //     (VecN([2.5, 1.5]), Angle::from_degrees(90.)),
        //     "green",
        //     1.,
        // ),
        // (
        //     (VecN([0., 0.]), Angle::from_degrees(0.)),
        //     (VecN([2., 2.]), Angle::from_degrees(0.)),
        //     "blue",
        //     1.,
        // ),
        (
            (VecN([0., 0.]), Angle::from_degrees(0.)),
            (VecN([3., 2.]), Angle::from_degrees(0.)),
            "blue",
            1.5,
        ),
        (
            (VecN([4., 2.]), Angle::from_degrees(0.)),
            (VecN([7., 0.]), Angle::from_degrees(0.)),
            "red",
            1.5,
        ),
        (
            (VecN([0., 4.]), Angle::from_degrees(-90.)),
            (VecN([3., 4.]), Angle::from_degrees(90.)),
            "green",
            1.,
        ),
        (
            (VecN([4., 3.]), Angle::from_degrees(90.)),
            (VecN([7., 3.]), Angle::from_degrees(-90.)),
            "orange",
            1.,
        ),
        (
            (VecN([1., 5.]), Angle::from_degrees(90.)),
            (VecN([2., 5.]), Angle::from_degrees(-90.)),
            "purple",
            1.,
        ),
        (
            (VecN([5., 7.]), Angle::from_degrees(-90.)),
            (VecN([6., 7.]), Angle::from_degrees(90.)),
            "turquoise",
            1.,
        ),
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

    // let mut points = Vec::new();
    // let mut rng = rng();
    // for i in 0..20 {
    //     let x = rng.random_range(-4.0..4.0);
    //     let y = rng.random_range(-4.0..4.0);
    //     let theta = rng.random_range(0.0..TAU);
    //     points.push((
    //         (VecN([0., 0.]), Angle::ZERO),
    //         (VecN([x, y]), Angle::new(theta)),
    //         "white",
    //         1.,
    //     ));
    // }

    for (start, end, color, radius) in points {
        let workspace = ReedsSheppWorkspace::new_borderless(radius, true);
        let segment = workspace.segment(start, end);
        // dbg!(segment);
        // svg.push(
        //     Cube::from_point(start.0 - VecN([0.1, 0.1])).with_point(start.0 + VecN([0.1, 0.1])),
        //     -1.,
        //     Style::fill("black"),
        // );
        // svg.push(
        //     Cube::from_point(end.0 - VecN([0.1, 0.1])).with_point(end.0 + VecN([0.1, 0.1])),
        //     -1.,
        //     Style::fill(color),
        // );
        put_arrow(
            &mut svg,
            start.0,
            start.0 + start.1.to_vec() * 0.5,
            Style::stroke("black", 0.05),
            20.,
            0.2
        );
        put_arrow(
            &mut svg,
            end.0,
            end.0 + end.1.to_vec() * 0.5,
            Style::stroke("black", 0.05),
            20.,
            0.2
        );
        // for i in 0..25 {
        //     let f = i as f64 / 25.;
        //     let pt = workspace.lerp(segment, f);
        //     svg.push(
        //         Cube::from_point(pt.0 - VecN([0.1, 0.1])).with_point(pt.0 + VecN([0.1, 0.1])),
        //         -1.,
        //         Style::fill("green"),
        //     );
        // }

        put_reeds_shepp(
            &mut svg,
            Style::stroke(color, 0.05).with_fill("none"),
            segment,
            0.,
        );
    }

    svg.write_to_file(&out_dir().join("test_base_reeds_shepp.svg"));
}
