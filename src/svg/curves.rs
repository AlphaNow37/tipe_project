use std::f64::consts::PI;
use std::ops::Add;
use crate::geometry::angles::Angle;
use crate::geometry::shapes::{CircleArc, Segment};
use crate::geometry::VecN;
use crate::svg;
use crate::svg::object::Style;
use crate::svg::SvgGroup;
use crate::workspace::reeds_shepp::{Gear, ReedsSheppSegment, Steering};

pub fn put_reeds_shepp(svg: &mut SvgGroup, style: Style, curve: ReedsSheppSegment, height: f64) {
    let mut curr_pos = curve.start;

    for p in curve.parts {
        let next_pos = p.at_distance(curr_pos, p.length);
        let forward = p.gear == Gear::Forward;
        let large_arc = p.length / p.radius > PI;
        match p.steering {
            Steering::Left => svg.push(
                CircleArc {
                    start: curr_pos.0,
                    end: next_pos.0,
                    clockwise: forward,
                    large_arc,
                    radius: p.radius,
                },
                height,
                style.clone(),
            ),
            Steering::Right => svg.push(
                CircleArc {
                    start: curr_pos.0,
                    end: next_pos.0,
                    clockwise: !forward,
                    large_arc,
                    radius: p.radius,
                },
                height,
                style.clone(),
            ),
            Steering::Straight => svg.push(
                Segment {
                    start: curr_pos.0,
                    end: next_pos.0,
                },
                height,
                style.clone(),
            ),
        }

        curr_pos = next_pos;
    }
}

pub fn put_arrow(
    svg: &mut svg::SvgGroup,
    start: VecN<2, f64>,
    end: VecN<2, f64>,
    style: Style,
    height: f64,
    caplength: f64,
) {
    let delta = end - start;

    svg.push(Segment { start, end }, height, style.clone());
    svg.push(
        Segment {
            start: end,
            end: end
                + Angle::from_point(delta)
                .add(Angle::from_degrees(135.))
                .to_vec()
                * caplength,
        },
        height,
        style.clone(),
    );
    svg.push(
        Segment {
            start: end,
            end: end
                + Angle::from_point(delta)
                .add(Angle::from_degrees(-135.))
                .to_vec()
                * caplength,
        },
        height,
        style.clone(),
    );
}
