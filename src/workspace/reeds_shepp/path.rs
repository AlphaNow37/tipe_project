/// Implements reeds-shepp path using a general implementation
/// Inspiration from https://github.com/LinusWeigand/reeds-shepp-rust
use std::f64::consts::PI;

use crate::{
    geometry::VecN,
    workspace::reeds_shepp::{
        Gear, OrientedCoord, ReedsSheppSegment, ReedsSheppSegmentPart, Steering,
    },
};

fn normalize_angle_closest(theta: f64) -> f64 {
    let mut theta = theta % (2. * PI);
    if theta >= PI {
        theta -= 2. * PI
    } else if theta < -PI {
        theta += 2. * PI
    }
    theta
}
fn normalize_angle_positive(theta: f64) -> f64 {
    let mut theta = theta % (2. * PI);
    if theta < 0. {
        theta += 2. * PI;
    }
    theta
}

fn cartesian_to_polar(x: f64, y: f64) -> (f64, f64) {
    (x.hypot(y), y.atan2(x))
}

fn change_basis(start: OrientedCoord, end: OrientedCoord, radius: f64) -> OrientedCoord {
    let dpos = end.0 - start.0;
    let proj = start.1.to_vec();

    let new_x = proj.dot(dpos);
    let new_y = proj.dot(dpos.rotate_right());

    (VecN([new_x, new_y]) / radius, end.1 - start.1)
}

fn timeflip(parts: [ReedsSheppSegmentPart; 5]) -> [ReedsSheppSegmentPart; 5] {
    parts.map(|p| p.timeflip())
}
fn reflect(parts: [ReedsSheppSegmentPart; 5]) -> [ReedsSheppSegmentPart; 5] {
    parts.map(|p| p.reflect())
}

// No const map in stable rust :/
macro_rules! reeds_shepp_fns {
    ($($f: ident),*) => {
        [$(
                |x, y, phi| $f(x, y, phi),
                |x, y, phi| $f(-x, y, -phi).map(timeflip),
                |x, y, phi| $f(x, -y, -phi).map(reflect),
                |x, y, phi| $f(-x, -y, phi).map(timeflip).map(reflect),
        )*]
    };
}
macro_rules! dubins_fns {
    ($($f: ident),*) => {
        [$(
                |x, y, phi| $f(x, y, phi),
                |x, y, phi| $f(x, -y, -phi).map(reflect),
        )*]
    };
}

type PathFn = fn(f64, f64, f64) -> Option<[ReedsSheppSegmentPart; 5]>;
const REEDS_SHEPP_FN: [PathFn; 48] = reeds_shepp_fns!(
    path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12
);
const DUBINS_FN: [PathFn; 6] = dubins_fns!(path_rsr_dubin, path_rsl_dubin, path_rlr_dubins);

pub fn get_best_path(
    start: OrientedCoord,
    end: OrientedCoord,
    radius: f64,
    forward_only: bool,
) -> ReedsSheppSegment {
    if start == end {
        return ReedsSheppSegment {
            start,
            end,
            length: 0.,
            parts: [ReedsSheppSegmentPart::NONE; 5],
        };
    }
    let new_pos = change_basis(start, end, radius);
    let x = new_pos.0[0];
    let y = new_pos.0[1];
    let phi = normalize_angle_closest(*new_pos.1);

    (if forward_only {
        DUBINS_FN.iter()
    } else {
        REEDS_SHEPP_FN.iter()
    })
    .filter_map(|f| f(x, y, phi))
    .map(|s| (s, s.map(|p| p.length).iter().sum::<f64>()))
    .min_by(|a, b| a.1.partial_cmp(&b.1).expect("There should not be NaN"))
    .map(|(s, dist)| ReedsSheppSegment {
        start,
        end,
        length: dist * radius,
        parts: s.map(|p| p.scale(radius)),
    })
    .expect("There should be at least one path !")
}

fn path_rsr_dubin(x: f64, y: f64, phi: f64) -> Option<[ReedsSheppSegmentPart; 5]> {
    let (d, theta) = cartesian_to_polar(x + phi.sin(), y + 1.0 - phi.cos());

    let t = normalize_angle_positive(2. * PI - theta);
    let v = normalize_angle_positive(theta - phi);

    Some([
        ReedsSheppSegmentPart::new(t, Steering::Right, Gear::Forward),
        ReedsSheppSegmentPart::new(d, Steering::Straight, Gear::Forward),
        ReedsSheppSegmentPart::new(v, Steering::Right, Gear::Forward),
        ReedsSheppSegmentPart::NONE,
        ReedsSheppSegmentPart::NONE,
    ])
}

fn path_rsl_dubin(x: f64, y: f64, phi: f64) -> Option<[ReedsSheppSegmentPart; 5]> {
    let (d, theta) = cartesian_to_polar(x - phi.sin(), y + 1.0 + phi.cos());
    if d < 2. {
        return None;
    }
    let a = (d * d - 4.).sqrt();
    let alpha = (2. / d).asin();

    let t = normalize_angle_positive(-theta + alpha);
    let v = normalize_angle_positive(phi - theta + alpha);

    Some([
        ReedsSheppSegmentPart::new(t, Steering::Right, Gear::Forward),
        ReedsSheppSegmentPart::new(a, Steering::Straight, Gear::Forward),
        ReedsSheppSegmentPart::new(v, Steering::Left, Gear::Forward),
        ReedsSheppSegmentPart::NONE,
        ReedsSheppSegmentPart::NONE,
    ])
}

fn path_rlr_dubins(x: f64, y: f64, phi: f64) -> Option<[ReedsSheppSegmentPart; 5]> {
    let (d, theta) = cartesian_to_polar(x + phi.sin(), y + 1.0 - phi.cos());

    if d > 4.0 {
        return None;
    }

    let alpha = (d / 4.0).acos();

    let t = normalize_angle_positive(alpha + PI / 2. - theta);
    let u = normalize_angle_positive(PI + 2.0 * alpha);
    let v = normalize_angle_positive(alpha + theta + PI / 2. - phi);

    Some([
        ReedsSheppSegmentPart::new(t, Steering::Right, Gear::Forward),
        ReedsSheppSegmentPart::new(u, Steering::Left, Gear::Forward),
        ReedsSheppSegmentPart::new(v, Steering::Right, Gear::Forward),
        ReedsSheppSegmentPart::NONE,
        ReedsSheppSegmentPart::NONE,
    ])
}

fn path1(x: f64, y: f64, phi: f64) -> Option<[ReedsSheppSegmentPart; 5]> {
    let (rho, theta) = cartesian_to_polar(x - phi.sin(), y - 1. + phi.cos());
    let v = normalize_angle_closest(phi - theta);

    Some([
        ReedsSheppSegmentPart::new(theta, Steering::Left, Gear::Forward),
        ReedsSheppSegmentPart::new(rho, Steering::Straight, Gear::Forward),
        ReedsSheppSegmentPart::new(v, Steering::Left, Gear::Forward),
        ReedsSheppSegmentPart::NONE,
        ReedsSheppSegmentPart::NONE,
    ])
}

fn path2(x: f64, y: f64, phi: f64) -> Option<[ReedsSheppSegmentPart; 5]> {
    let (rho, theta) = cartesian_to_polar(x + phi.sin(), y - 1. - phi.cos());

    if rho * rho >= 4. {
        let u = (rho * rho - 4.).sqrt();
        let t = normalize_angle_closest(theta + (2.0_f64).atan2(u));
        let v = normalize_angle_closest(t - phi);

        Some([
            ReedsSheppSegmentPart::new(t, Steering::Left, Gear::Forward),
            ReedsSheppSegmentPart::new(u, Steering::Straight, Gear::Forward),
            ReedsSheppSegmentPart::new(v, Steering::Right, Gear::Forward),
            ReedsSheppSegmentPart::NONE,
            ReedsSheppSegmentPart::NONE,
        ])
    } else {
        None
    }
}

fn path3(x: f64, y: f64, phi: f64) -> Option<[ReedsSheppSegmentPart; 5]> {
    let xi = x - phi.sin();
    let eta = y - 1. + phi.cos();
    let (rho, theta) = cartesian_to_polar(xi, eta);

    if rho <= 4. {
        let a = (rho / 4.).acos();
        let t = normalize_angle_closest(theta + PI / 2. + a);
        let u = normalize_angle_closest(PI - 2. * a);
        let v = normalize_angle_closest(phi - t - u);

        Some([
            ReedsSheppSegmentPart::new(t, Steering::Left, Gear::Forward),
            ReedsSheppSegmentPart::new(u, Steering::Right, Gear::Backward),
            ReedsSheppSegmentPart::new(v, Steering::Left, Gear::Forward),
            ReedsSheppSegmentPart::NONE,
            ReedsSheppSegmentPart::NONE,
        ])
    } else {
        None
    }
}

fn path4(x: f64, y: f64, phi: f64) -> Option<[ReedsSheppSegmentPart; 5]> {
    let xi = x - phi.sin();
    let eta = y - 1. + phi.cos();
    let (rho, theta) = cartesian_to_polar(xi, eta);

    if rho <= 4. {
        let a = (rho / 4.).acos();
        let t = normalize_angle_closest(theta + PI / 2. + a);
        let u = normalize_angle_closest(PI - 2. * a);
        let v = normalize_angle_closest(t + u - phi);

        Some([
            ReedsSheppSegmentPart::new(t, Steering::Left, Gear::Forward),
            ReedsSheppSegmentPart::new(u, Steering::Right, Gear::Backward),
            ReedsSheppSegmentPart::new(v, Steering::Left, Gear::Backward),
            ReedsSheppSegmentPart::NONE,
            ReedsSheppSegmentPart::NONE,
        ])
    } else {
        None
    }
}

fn path5(x: f64, y: f64, phi: f64) -> Option<[ReedsSheppSegmentPart; 5]> {
    let xi = x - phi.sin();
    let eta = y - 1. + phi.cos();
    let (rho, theta) = cartesian_to_polar(xi, eta);

    if rho <= 4. {
        let u = (1. - rho * rho / 8.).acos();
        let a = (2. * u.sin() / rho).asin();
        let t = normalize_angle_closest(theta + PI / 2. - a);
        let v = normalize_angle_closest(t - u - phi);

        Some([
            ReedsSheppSegmentPart::new(t, Steering::Left, Gear::Forward),
            ReedsSheppSegmentPart::new(u, Steering::Right, Gear::Forward),
            ReedsSheppSegmentPart::new(v, Steering::Left, Gear::Backward),
            ReedsSheppSegmentPart::NONE,
            ReedsSheppSegmentPart::NONE,
        ])
    } else {
        None
    }
}

fn path6(x: f64, y: f64, phi: f64) -> Option<[ReedsSheppSegmentPart; 5]> {
    let xi = x + phi.sin();
    let eta = y - 1. - phi.cos();
    let (rho, theta) = cartesian_to_polar(xi, eta);

    if rho <= 4. {
        let (t, u, v);
        if rho <= 2. {
            let a = ((rho + 2.) / 4.).acos();
            t = normalize_angle_closest(theta + PI / 2. + a);
            u = normalize_angle_closest(a);
            v = normalize_angle_closest(phi - t + 2. * u);
        } else {
            let a = ((rho - 2.) / 4.).acos();
            t = normalize_angle_closest(theta + PI / 2. - a);
            u = normalize_angle_closest(PI - a);
            v = normalize_angle_closest(phi - t + 2. * u);
        }

        Some([
            ReedsSheppSegmentPart::new(t, Steering::Left, Gear::Forward),
            ReedsSheppSegmentPart::new(u, Steering::Right, Gear::Forward),
            ReedsSheppSegmentPart::new(u, Steering::Left, Gear::Backward),
            ReedsSheppSegmentPart::new(v, Steering::Right, Gear::Backward),
            ReedsSheppSegmentPart::NONE,
        ])
    } else {
        None
    }
}

fn path7(x: f64, y: f64, phi: f64) -> Option<[ReedsSheppSegmentPart; 5]> {
    let xi = x + phi.sin();
    let eta = y - 1. - phi.cos();
    let (rho, theta) = cartesian_to_polar(xi, eta);

    let u1 = (20. - rho * rho) / 16.;

    if rho <= 6. && (0. ..=1.).contains(&u1) {
        let u = u1.acos();
        let asin_arg = (2. * u.sin() / rho).clamp(-1.0, 1.0);
        let a = asin_arg.asin();
        let t = normalize_angle_closest(theta + PI / 2. + a);
        let v = normalize_angle_closest(t - phi);

        Some([
            ReedsSheppSegmentPart::new(t, Steering::Left, Gear::Forward),
            ReedsSheppSegmentPart::new(u, Steering::Right, Gear::Backward),
            ReedsSheppSegmentPart::new(u, Steering::Left, Gear::Backward),
            ReedsSheppSegmentPart::new(v, Steering::Right, Gear::Forward),
            ReedsSheppSegmentPart::NONE,
        ])
    } else {
        None
    }
}

fn path8(x: f64, y: f64, phi: f64) -> Option<[ReedsSheppSegmentPart; 5]> {
    let xi = x - phi.sin();
    let eta = y - 1. + phi.cos();
    let (rho, theta) = cartesian_to_polar(xi, eta);

    if rho >= 2. {
        let sqrt_arg = rho * rho - 4.;
        if sqrt_arg < 0. {
            return None;
        }

        let s = sqrt_arg.sqrt();
        let u_param = s - 2.;

        let a = (2.0_f64).atan2(s);
        let t = normalize_angle_closest(theta + PI / 2. + a);
        let v = normalize_angle_closest(t - phi + PI / 2.);

        Some([
            ReedsSheppSegmentPart::new(t, Steering::Left, Gear::Forward),
            ReedsSheppSegmentPart::new(PI / 2., Steering::Right, Gear::Backward),
            ReedsSheppSegmentPart::new(u_param, Steering::Straight, Gear::Backward),
            ReedsSheppSegmentPart::new(v, Steering::Left, Gear::Backward),
            ReedsSheppSegmentPart::NONE,
        ])
    } else {
        None
    }
}

fn path9(x: f64, y: f64, phi: f64) -> Option<[ReedsSheppSegmentPart; 5]> {
    let xi = x - phi.sin();
    let eta = y - 1. + phi.cos();
    let (rho, theta) = cartesian_to_polar(xi, eta);

    if rho >= 2. {
        let sqrt_arg = rho * rho - 4.;
        if sqrt_arg < 0. {
            return None;
        }

        let s = sqrt_arg.sqrt();
        let u_param = s - 2.;

        let a = s.atan2(2.0_f64);
        let t = normalize_angle_closest(theta + PI / 2. - a);
        let v = normalize_angle_closest(t - phi - PI / 2.);

        Some([
            ReedsSheppSegmentPart::new(t, Steering::Left, Gear::Forward),
            ReedsSheppSegmentPart::new(u_param, Steering::Straight, Gear::Forward),
            ReedsSheppSegmentPart::new(PI / 2., Steering::Right, Gear::Forward),
            ReedsSheppSegmentPart::new(v, Steering::Left, Gear::Backward),
            ReedsSheppSegmentPart::NONE,
        ])
    } else {
        None
    }
}

fn path10(x: f64, y: f64, phi: f64) -> Option<[ReedsSheppSegmentPart; 5]> {
    let xi = x + phi.sin();
    let eta = y - 1. - phi.cos();
    let (rho, theta) = cartesian_to_polar(xi, eta);

    if rho >= 2. {
        let t = normalize_angle_closest(theta + PI / 2.);
        let u = rho - 2.;
        let v = normalize_angle_closest(phi - t - PI / 2.);

        Some([
            ReedsSheppSegmentPart::new(t, Steering::Left, Gear::Forward),
            ReedsSheppSegmentPart::new(PI / 2., Steering::Right, Gear::Backward),
            ReedsSheppSegmentPart::new(u, Steering::Straight, Gear::Backward),
            ReedsSheppSegmentPart::new(v, Steering::Right, Gear::Backward),
            ReedsSheppSegmentPart::NONE,
        ])
    } else {
        None
    }
}

fn path11(x: f64, y: f64, phi: f64) -> Option<[ReedsSheppSegmentPart; 5]> {
    let xi = x + phi.sin();
    let eta = y - 1. - phi.cos();
    let (rho, theta) = cartesian_to_polar(xi, eta);

    if rho >= 2. {
        let t = normalize_angle_closest(theta);
        let u = rho - 2.;
        let v = normalize_angle_closest(phi - t - PI / 2.);

        Some([
            ReedsSheppSegmentPart::new(t, Steering::Left, Gear::Forward),
            ReedsSheppSegmentPart::new(u, Steering::Straight, Gear::Forward),
            ReedsSheppSegmentPart::new(PI / 2., Steering::Left, Gear::Forward),
            ReedsSheppSegmentPart::new(v, Steering::Right, Gear::Backward),
            ReedsSheppSegmentPart::NONE,
        ])
    } else {
        None
    }
}

fn path12(x: f64, y: f64, phi: f64) -> Option<[ReedsSheppSegmentPart; 5]> {
    let xi = x + phi.sin();
    let eta = y - 1. - phi.cos();
    let (rho, theta) = cartesian_to_polar(xi, eta);

    if rho >= 4. {
        let sqrt_base_arg = rho * rho - 4.;
        if sqrt_base_arg < 0. {
            return None;
        }

        let u_base = sqrt_base_arg.sqrt();
        let u_param = u_base - 4.;

        let s_equiv = u_base;

        let a = (2.0_f64).atan2(s_equiv);
        let t = normalize_angle_closest(theta + PI / 2. + a);
        let v = normalize_angle_closest(t - phi);

        Some([
            ReedsSheppSegmentPart::new(t, Steering::Left, Gear::Forward),
            ReedsSheppSegmentPart::new(PI / 2., Steering::Right, Gear::Backward),
            ReedsSheppSegmentPart::new(u_param, Steering::Straight, Gear::Backward),
            ReedsSheppSegmentPart::new(PI / 2., Steering::Left, Gear::Backward),
            ReedsSheppSegmentPart::new(v, Steering::Right, Gear::Forward),
        ])
    } else {
        None
    }
}
