use crate::utils::numbers::{NotNanF64, UsizeExt, F64_EPSILON};
use std::f64::consts::PI;

use crate::geometry::angles::Angle;

use super::VecN;

#[derive(Default, Clone, Copy, Debug, PartialEq)]
pub struct Cube<const N: usize> {
    pub start: VecN<N, f64>,
    pub end: VecN<N, f64>,
}
impl<const N: usize> Cube<N> {
    pub fn join(self, other: Self) -> Self {
        let start = self
            .start
            .zip(other.start)
            .map_component(|(a, b)| f64::min(a, b));
        let end = self
            .end
            .zip(other.end)
            .map_component(|(a, b)| f64::max(a, b));
        Self { start, end }
    }
    pub fn with_point(self, pt: VecN<N, f64>) -> Self {
        self.join(Self { start: pt, end: pt })
    }
    pub fn from_point(pt: VecN<N, f64>) -> Self {
        Self { start: pt, end: pt }
    }
    pub fn size(self) -> VecN<N, f64> {
        self.end - self.start
    }
}

#[derive(Default, Clone, Debug, PartialEq)]
pub struct Polygon(pub Vec<VecN<2, f64>>);
impl Polygon {
    pub fn is_counter_clockwise(&self) -> bool {
        if self.0.len() <= 2 {
            true
        } else {
            let n = self.0.len();
            let mut min_i = (0..n)
                .min_by_key(|i| NotNanF64::new(self.0[*i][1]))
                .unwrap();
            loop {
                let p = self.0[min_i.add_rem(0, n)];
                let next_p = self.0[min_i.add_rem(1, n)];
                let last_p = self.0[min_i.add_rem(-1, n)];
                let angle = Angle::from_points(last_p, p, next_p);
                if *angle < PI {
                    return false;
                } else if *angle > PI {
                    return true;
                } else {
                    min_i += 1;
                }
            }
        }
    }
}

#[derive(Default, Clone, Copy, Debug, PartialEq)]
pub struct InfiniteLine<const N: usize> {
    pub start: VecN<N, f64>,
    pub end: VecN<N, f64>,
}
impl<const N: usize> InfiniteLine<N> {
    pub fn point_at_time(self, t: f64) -> VecN<N, f64> {
        return self.start + (self.end - self.start) * t;
    }
}
impl InfiniteLine<2> {
    pub fn intersection_time(self, other: Self) -> Option<(f64, f64)> {
        let delta_x = self.end[0] - self.start[0];
        let delta_y = self.end[1] - self.start[1];
        let delta_a = other.end[0] - other.start[0];
        let delta_b = other.end[1] - other.start[1];
        let delta_1 = self.start[0] - other.start[0];
        let delta_2 = self.start[1] - other.start[1];
        let denominator = delta_y * delta_a - delta_x * delta_b;
        let numerator_1 = delta_1 * delta_b - delta_2 * delta_a;
        let numerator_2 = delta_1 * delta_y - delta_2 * delta_x;
        if denominator.abs() < F64_EPSILON {
            return None;
        }
        Some((numerator_1 / denominator, numerator_2 / denominator))
    }
}

#[derive(Default, Clone, Copy, Debug, PartialEq)]
pub struct Segment<const N: usize> {
    pub start: VecN<N, f64>,
    pub end: VecN<N, f64>,
}
impl<const N: usize> Segment<N> {
    pub fn to_line(self) -> InfiniteLine<N> {
        InfiniteLine {
            start: self.start,
            end: self.end,
        }
    }
}
impl Segment<2> {
    pub fn intersect_line(self, line: InfiniteLine<2>) -> bool {
        self.to_line()
            .intersection_time(line)
            .map(|(t1, _)| 0. <= t1 && t1 <= 1.)
            .unwrap_or(false)
    }
    pub fn intersect_segment(self, segment: Segment<2>) -> bool {
        self.to_line()
            .intersection_time(segment.to_line())
            .map(|(t1, t2)| 0. <= t1 && t1 <= 1. && 0. <= t2 && t2 <= 1.)
            .unwrap_or(false)
    }
}

#[derive(Default, Clone, Copy, Debug, PartialEq)]
pub struct Ray<const N: usize> {
    pub start: VecN<N, f64>,
    pub end: VecN<N, f64>,
}
impl<const N: usize> Ray<N> {
    pub fn to_line(self) -> InfiniteLine<N> {
        InfiniteLine {
            start: self.start,
            end: self.end,
        }
    }
}
impl Ray<2> {
    pub fn intersect_line(self, line: InfiniteLine<2>) -> bool {
        self.to_line()
            .intersection_time(line)
            .map(|(t1, _)| 0. <= t1)
            .unwrap_or(false)
    }
    pub fn intersect_segment(self, segment: Segment<2>) -> bool {
        self.to_line()
            .intersection_time(segment.to_line())
            .map(|(t1, t2)| 0. <= t1 && 0. <= t2 && t2 <= 1.)
            .unwrap_or(false)
    }
}

#[test]
fn test_collisions() {
    let s1 = Segment {
        start: VecN([0., 0.]),
        end: VecN([1., 1.]),
    };
    let s2 = Segment {
        start: VecN([1., 0.]),
        end: VecN([0., 1.]),
    };
    let s3 = Segment {
        start: VecN([2., 2.]),
        end: VecN([3., 3.]),
    };
    let s4 = Segment {
        start: VecN([0.5, 0.]),
        end: VecN([1., 0.5]),
    };
    assert!(s1.intersect_segment(s2));
    assert!(!s1.intersect_segment(s3));
    assert!(!s1.intersect_segment(s4));
    assert!(s2.intersect_segment(s4));
}

#[test]
fn test_polygon() {
    assert!(Polygon(vec![VecN([0., 1.]), VecN([1., 0.]), VecN([1., 1.]),]).is_counter_clockwise());

    assert!(!Polygon(vec![VecN([1., 0.]), VecN([0., 1.]), VecN([1., 1.]),]).is_counter_clockwise());

    assert!(!Polygon(vec![
        VecN([1., 0.]),
        VecN([0., 1.]),
        VecN([1., 1.]),
        VecN([2., 1.]),
    ])
    .is_counter_clockwise());
}
