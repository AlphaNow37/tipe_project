use crate::{
    geometry::{angles::Angle, shapes::Cube, VecN},
    utils::numbers::F64_EPSILON,
    workspace::{
        cartesians::{EuclidianDistance, Length},
        reeds_shepp::path::get_best_path,
        workspace::WorkspaceTopology,
    },
};

mod path;

pub type OrientedCoord = (VecN<2, f64>, Angle);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Gear {
    Forward,
    Stopped,
    Backward,
}
impl Gear {
    fn reverse(&self) -> Self {
        match self {
            Gear::Forward => Gear::Backward,
            Gear::Backward => Gear::Forward,
            Gear::Stopped => Gear::Stopped,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Steering {
    Straight,
    Left,
    Right,
}
impl Steering {
    fn reverse(&self) -> Self {
        match self {
            Steering::Left => Steering::Right,
            Steering::Right => Steering::Left,
            Steering::Straight => Steering::Straight,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct ReedsSheppSegmentPart {
    pub length: f64,
    pub gear: Gear,
    pub steering: Steering,
    pub radius: f64,
}
impl ReedsSheppSegmentPart {
    const NONE: Self = Self {
        length: 0.,
        gear: Gear::Stopped,
        steering: Steering::Straight,
        radius: 1.,
    };

    pub fn new(length: f64, steering: Steering, gear: Gear) -> Self {
        if length.abs() < F64_EPSILON {
            return Self::NONE;
        }

        if length >= 0. {
            Self {
                length,
                steering,
                gear,
                radius: 1.,
            }
        } else {
            Self {
                length: -length,
                steering,
                gear: gear.reverse(),
                radius: 1.,
            }
        }
    }
    pub fn timeflip(self) -> Self {
        Self {
            gear: self.gear.reverse(),
            ..self
        }
    }
    pub fn reflect(self) -> Self {
        Self {
            steering: self.steering.reverse(),
            ..self
        }
    }
    pub fn scale(self, scale: f64) -> Self {
        Self {
            length: self.length * scale,
            radius: self.radius * scale,
            ..self
        }
    }

    /// Returns the length of the segment
    pub fn length(self) -> f64 {
        self.length
    }
    /// Returns the point p on the curve such that the distance to the start following the curve is dist
    /// Should be called with 0. <= dist <= lenght()
    pub fn at_distance(self, (pos, direction): OrientedCoord, dist: f64) -> OrientedCoord {
        debug_assert!(0. <= dist && dist <= self.length());
        let dist_signed = match self.gear {
            Gear::Forward => dist,
            Gear::Backward => -dist,
            Gear::Stopped => return (pos, direction),
        };
        let side = match self.steering {
            Steering::Straight => return (pos + direction.to_vec() * dist_signed, direction),
            Steering::Left => 1.,
            Steering::Right => -1.,
        };
        let normal_angle = direction + Angle::QUARTER * side;
        let final_direction = direction + Angle::new(dist_signed / self.radius * side);
        let final_normal_angle = final_direction + Angle::QUARTER * side;
        (
            pos + normal_angle.to_vec() * self.radius - final_normal_angle.to_vec() * self.radius,
            final_direction,
        )
    }
    /// Truncate the segment part at dist
    /// Should be called with 0. <= dist <= lenght()
    pub fn truncate_at_distance(self, dist: f64) -> Self {
        debug_assert!(0. <= dist && dist <= self.length());
        Self {
            length: dist,
            ..self
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ReedsSheppSegment {
    pub parts: [ReedsSheppSegmentPart; 5],
    pub start: OrientedCoord,
    pub end: OrientedCoord,
    pub length: f64,
}

#[derive(Clone, Debug, Copy)]
pub struct ReedsSheppWorkspace {
    pub physical_space: Cube<2>,
    pub steering_radius: f64,
    /// True -> Dubins, False -> Reeds-Shepp
    pub forward_only: bool,
}
impl ReedsSheppWorkspace {
    pub fn new_borderless(steering_radius: f64, forward_only: bool) -> Self {
        Self {
            physical_space: Cube::INFINTE,
            steering_radius,
            forward_only,
        }
    }
    fn check_segment_invariants(&self, seg: ReedsSheppSegment) {
        debug_assert!(
            (seg.parts.iter().map(|p| p.length).sum::<f64>() - seg.length).abs() < F64_EPSILON,
            "{}, {}",
            seg.parts.iter().map(|p| p.length).sum::<f64>(),
            seg.length
        );
        debug_assert!(
            EuclidianDistance.length(self.lerp(seg, 1.).0 - seg.end.0) < F64_EPSILON,
            "{:?}, {:?}",
            self.lerp(seg, 1.),
            seg.end
        );
        for s in seg.parts {
            debug_assert!(s.length >= 0.);
        }
        if self.forward_only {
            for s in seg.parts {
                debug_assert!(s.gear != Gear::Backward);
            }
        }
    }
}
impl WorkspaceTopology for ReedsSheppWorkspace {
    type Vertex = OrientedCoord;
    type Segment = ReedsSheppSegment;
    const NB_DIMENSIONS: usize = 3;

    fn is_distance_symetric(&self) -> bool {
        !self.forward_only
    }
    fn segment(&self, start: Self::Vertex, end: Self::Vertex) -> Self::Segment {
        let segment = get_best_path(start, end, self.steering_radius, self.forward_only);
        // self.check_segment_invariants(segment);
        segment
    }
    fn segment_start(&self, segment: Self::Segment) -> Self::Vertex {
        segment.start
    }
    fn segment_end(&self, segment: Self::Segment) -> Self::Vertex {
        segment.end
    }
    fn segment_reverse(&self, mut segment: Self::Segment) -> Self::Segment {
        debug_assert!(!self.forward_only);
        segment.parts.reverse();
        segment.parts = segment.parts.map(ReedsSheppSegmentPart::timeflip);
        std::mem::swap(&mut segment.start, &mut segment.end);
        self.check_segment_invariants(segment);
        segment
    }
    fn length(&self, segment: Self::Segment) -> f64 {
        segment.length
    }
    fn lerp(&self, segment: Self::Segment, time: f64) -> Self::Vertex {
        debug_assert!(0. <= time && time <= 1.);
        let mut dist_restante = segment.length * time;
        let mut pos = segment.start;
        for p in segment.parts {
            let length = p.length();
            if dist_restante <= length {
                return p.at_distance(pos, dist_restante);
            } else {
                dist_restante -= length;
                pos = p.at_distance(pos, p.length());
            }
        }
        if dist_restante < F64_EPSILON {
            return pos;
        }
        unreachable!("Bug in lerp")
    }
    fn sample_random(&self, rng: &mut impl rand::Rng) -> Self::Vertex {
        (self.physical_space.random_vertex(rng), rng.random())
    }
    fn steer_in_disc(&self, mut segment: Self::Segment, radius: f64) -> Self::Segment {
        if segment.length > radius {
            let mut pos = segment.start;
            let mut dist_seen = 0.;
            for p in &mut segment.parts {
                let length = p.length();
                if dist_seen >= radius {
                    *p = ReedsSheppSegmentPart::NONE
                } else if dist_seen + length >= radius {
                    *p = p.truncate_at_distance(radius - dist_seen);
                    segment.end = p.at_distance(pos, p.length());
                    dist_seen += length;
                } else {
                    pos = p.at_distance(pos, p.length());
                    dist_seen += length;
                }
            }
            segment.length = radius;
        }
        self.check_segment_invariants(segment);
        segment
    }
}
