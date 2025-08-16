use crate::{
    geometry::{angles::Angle, shapes::Cube, VecN},
    workspace::{
        cartesians::{EuclidianDistance, Length},
        workspace::WorkspaceTopology,
    },
};

mod path;

pub type OrientedCoord = (VecN<2, f64>, Angle);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Gear {
    Forward,
    Backward,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Steering {
    Straight,
    Left,
    Right,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ReedsSheppSegmentPart {
    Rotation {
        center: VecN<2, f64>,
        start_angle: Angle,
        end_angle: Angle,
        radius: f64,
        direction: f64, // -1 if clockwise, 1 if counterclockwise
    },
    Straight {
        start: VecN<2, f64>,
        end: VecN<2, f64>,
        angle: Angle,
    },
    Single {
        coords: OrientedCoord,
    },
}
impl ReedsSheppSegmentPart {
    /// Returns the length of the segment
    pub fn length(self) -> f64 {
        match self {
            Self::Rotation {
                start_angle,
                end_angle,
                radius,
                direction,
                ..
            } => *((start_angle - end_angle) * direction) * radius,
            Self::Straight { start, end, .. } => EuclidianDistance.length(end - start),
            Self::Single { .. } => 0.,
        }
    }
    /// Returns the point p on the curve such that the distance to the start following the curve is dist
    /// Should be called with 0. <= dist <= lenght()
    pub fn at_distance(self, dist: f64) -> OrientedCoord {
        match self {
            Self::Rotation {
                center,
                start_angle,
                radius,
                direction,
                ..
            } => {
                let angle = start_angle + Angle::new(dist / radius * direction);
                (
                    center + angle.to_vec() * radius,
                    angle + Angle::QUARTER * direction,
                )
            }
            Self::Straight { start, end, angle } => {
                let delta = end - start;
                (
                    start + delta * dist / EuclidianDistance.length(delta),
                    angle,
                )
            }
            Self::Single { coords } => coords,
        }
    }
    /// Truncate the segment part at dist
    /// Should be called with 0. <= dist <= lenght()
    pub fn truncate_at_distance(self, dist: f64) -> Self {
        match self {
            Self::Rotation {
                center,
                start_angle,
                radius,
                direction,
                ..
            } => Self::Rotation {
                center,
                start_angle,
                end_angle: start_angle + Angle::new(dist / radius * direction),
                radius,
                direction,
            },
            Self::Straight { start, end, angle } => Self::Straight {
                start,
                end: start + (end - start) * dist / EuclidianDistance.length(end - start),
                angle,
            },
            Self::Single { .. } => self,
        }
    }
    pub fn reverse(self) -> Self {
        match self {
            Self::Rotation {
                center,
                start_angle,
                end_angle,
                radius,
                direction,
            } => Self::Rotation {
                center,
                start_angle: end_angle,
                end_angle: start_angle,
                radius,
                direction: -direction,
            },
            Self::Straight { start, end, angle } => Self::Straight {
                start: end,
                end: start,
                angle: angle + Angle::HALF,
            },
            Self::Single {
                coords: (pos, angle),
            } => Self::Single {
                coords: (pos, angle + Angle::HALF),
            },
        }
    }
    pub fn last(self) -> OrientedCoord {
        match self {
            Self::Rotation {
                center,
                end_angle,
                radius,
                direction,
                ..
            } => (
                center + end_angle.to_vec() * radius,
                end_angle + Angle::QUARTER * direction,
            ),
            Self::Straight { end, angle, .. } => (end, angle),
            Self::Single { coords } => coords,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct ReedsSheppSegment {
    pub parts: [ReedsSheppSegmentPart; 5],
    pub start: OrientedCoord,
    pub end: OrientedCoord,
    pub length: f64,
}

#[derive(Clone, Debug)]
pub struct ReedsSheppWorkspace {
    pub physical_space: Cube<2>,
    pub turning_radius: f64,
}
impl WorkspaceTopology for ReedsSheppWorkspace {
    type Vertex = OrientedCoord;
    type Segment = ReedsSheppSegment;
    const NB_DIMENSIONS: usize = 3;

    fn segment(&self, start: Self::Vertex, end: Self::Vertex) -> Self::Segment {
        todo!()
    }
    fn segment_start(&self, segment: Self::Segment) -> Self::Vertex {
        segment.start
    }
    fn segment_end(&self, segment: Self::Segment) -> Self::Vertex {
        segment.end
    }
    fn segment_reverse(&self, mut segment: Self::Segment) -> Self::Segment {
        segment.parts.reverse();
        segment.parts = segment.parts.map(ReedsSheppSegmentPart::reverse);
        std::mem::swap(&mut segment.start, &mut segment.end);
        segment
    }
    fn length(&self, segment: Self::Segment) -> f64 {
        segment.length
    }
    fn lerp(&self, segment: Self::Segment, time: f64) -> Self::Vertex {
        debug_assert!(0. <= time && time <= 1.);
        let mut dist_restante = segment.length * time;
        for p in segment.parts {
            let length = p.length();
            if dist_restante <= length {
                return p.at_distance(dist_restante);
            } else {
                dist_restante -= length
            }
        }
        unreachable!("Bug in lerp")
    }
    fn sample_random(&self, rng: &mut impl rand::Rng) -> Self::Vertex {
        (self.physical_space.random_vertex(rng), rng.random())
    }
    fn steer_in_disc(&self, mut segment: Self::Segment, radius: f64) -> Self::Segment {
        if segment.length > radius {
            let mut dist_seen = 0.;
            for p in &mut segment.parts {
                let length = p.length();
                if dist_seen >= radius {
                    *p = ReedsSheppSegmentPart::Single {
                        coords: segment.end,
                    }
                } else if dist_seen + length >= radius {
                    *p = p.truncate_at_distance(dist_seen + length - radius);
                    segment.end = p.last();
                }
                dist_seen += length;
            }
        }
        segment
    }
}
