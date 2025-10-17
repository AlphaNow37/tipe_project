use super::out_dir;
use crate::geometry::shapes::{Cube, Polygon, Ray, Segment};
use crate::svg::object::Text;
use crate::utils::numbers::{UsizeExt, Zero};
use crate::workspace::cartesians::{EuclidianDistance, Length};
use crate::{
    geometry::VecN,
    svg::{self, object::Style},
};

pub fn illustration_line_sweep() {
    let center_pos = VecN([0., 0.]);
    let polygons = vec![
        Polygon::new(vec![VecN([-1., 3.]), VecN([2., -1.]), VecN([2.5, 2.])]),
        Polygon::new(vec![
            VecN([4., 3.]),
            VecN([3.5, -3.]),
            VecN([5.5, -1.5]),
            VecN([7., 5.]),
        ]),
        Polygon::new(vec![VecN([1., 3.]), VecN([3., 2.5]), VecN([4., 5.])]),
    ];
    let ray = Ray {
        start: center_pos,
        end: center_pos + VecN([7.5, 2.5]),
    };

    let mut svg = svg::SvgGroup::default();

    svg.push(
        Cube::from_point(center_pos + VecN([-0.05, -0.05]))
            .with_point(center_pos + VecN([0.05, 0.05])),
        20.,
        Style::fill("black"),
    );
    svg.push(
        Text {
            content: "center".into(),
            position: center_pos + VecN([-0.5, -0.25]),
            font_size: 0.5,
        },
        20.,
        Style::fill("black"),
    );

    for (p_i, p) in polygons.into_iter().enumerate() {
        let p_center = p.0.iter().fold(VecN::ZERO, |a, b| a + *b) / (p.len() as f64);
        let p_letter = "ABCDEF".chars().nth(p_i).unwrap();
        for i in 0..p.len() {
            let s = Segment {
                start: p.0[i],
                end: p.0[i.add_rem(1, p.0.len())],
            };
            if ray.intersect_segment(s) {
                let normal = (s.end - s.start).rotate_right();
                svg.push(s, 1., Style::stroke("#FF0000", 0.1).with_fill("none"));
                svg.push(
                    Text {
                        content: format!("{p_letter}{}", i+1),
                        position: (s.start + s.end) / 2.
                            + (normal / EuclidianDistance.length(normal)) * 0.4
                            + VecN([-0.25, 0.]),
                        font_size: 0.5,
                    },
                    20.,
                    Style::fill("#FF0000"),
                );
            }
        }
        svg.push(p, 0., Style::fill("#555555"));
    }
    svg.push(
        Segment {
            start: ray.start,
            end: ray.end,
        },
        3.,
        Style::stroke("#FFAA00", 0.1),
    );

    svg.set_background("#FFFFFF".to_string());
    svg.write_to_file(&out_dir().join("illustration_line_sweep.svg"));
}
