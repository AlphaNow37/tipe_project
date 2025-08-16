/// Implements reeds-shepp path using a general implementation
/// Inspiration from https://github.com/LinusWeigand/reeds-shepp-rust
use std::f64::consts::PI;

use crate::workspace::reeds_shepp::{OrientedCoord, ReedsSheppSegment};


pub fn normalize_angle(theta: f64) -> f64 {
    let mut theta = theta % (2. * PI);
    if theta >= PI {
        theta -= 2. * PI
    } else if theta < -PI {
        theta += 2. * PI
    }
    theta
}

fn cartesian_to_polar(x: f64, y: f64) -> (f64, f64) {
    (x.hypot(y), y.atan2(x))
}

fn seg_from_3(parts: [(f64, Steering, Gear); 3]) -> Option<ReedsSheppSegment> {
    todo!()
}
fn seg_from_4(parts: [(f64, Steering, Gear); 4]) -> Option<ReedsSheppSegment> {
    todo!()
}
fn seg_from_5(parts: [(f64, Steering, Gear); 5]) -> Option<ReedsSheppSegment> {
    todo!()
}

pub type PathFn = fn(f64, f64, f64) -> Option<ReedsSheppSegment>;
pub const PATH_FNS: [PathFn; 12] = [
    path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12,
];

pub fn get_best_path(start: OrientedCoord, end: OrientedCoord) -> ReedsSheppSegment {
    
}

fn path1(x: f64, y: f64, phi: f64) -> Option<ReedsSheppSegment> {
    let (rho, theta) = cartesian_to_polar(x - phi.sin(), y - 1. + phi.cos());
    let v = normalize_angle(phi - theta);

    seg_from_3([
        (theta, Steering::Left, Gear::Forward),
        (rho, Steering::Straight, Gear::Forward),
        (v, Steering::Left, Gear::Forward),
    ])
}

fn path2(x: f64, y: f64, phi: f64) -> Option<ReedsSheppSegment> {
    let (rho, theta) = cartesian_to_polar(x + phi.sin(), y - 1. - phi.cos());

    if rho * rho >= 4. {
        let u = (rho * rho - 4.).sqrt();
        let t = normalize_angle(theta + (2.0_f64).atan2(u));
        let v = normalize_angle(t - phi);

        seg_from_3([
            (t, Steering::Left, Gear::Forward),
            (u, Steering::Straight, Gear::Forward),
            (v, Steering::Right, Gear::Forward),
        ])
    } else {
        None
    }
}

fn path3(x: f64, y: f64, phi: f64) -> Option<ReedsSheppSegment> {
    let xi = x - phi.sin();
    let eta = y - 1. + phi.cos();
    let (rho, theta) = cartesian_to_polar(xi, eta);

    if rho <= 4. {
        let a = (rho / 4.).acos();
        let t = normalize_angle(theta + PI / 2. + a);
        let u = normalize_angle(PI - 2. * a);
        let v = normalize_angle(phi - t - u);

        seg_from_3([
            (t, Steering::Left, Gear::Forward),
            (u, Steering::Right, Gear::Backward),
            (v, Steering::Left, Gear::Forward),
        ])
    } else {
        None
    }
}

fn path4(x: f64, y: f64, phi: f64) -> Option<ReedsSheppSegment> {
    let xi = x - phi.sin();
    let eta = y - 1. + phi.cos();
    let (rho, theta) = cartesian_to_polar(xi, eta);

    if rho <= 4. {
        let a = (rho / 4.).acos();
        let t = normalize_angle(theta + PI / 2. + a);
        let u = normalize_angle(PI - 2. * a);
        let v = normalize_angle(t + u - phi);

        seg_from_3([
            (t, Steering::Left, Gear::Forward),
            (u, Steering::Right, Gear::Backward),
            (v, Steering::Left, Gear::Backward),
        ])
    } else {
        None
    }
}

fn path5(x: f64, y: f64, phi: f64) -> Option<ReedsSheppSegment> {
    let xi = x - phi.sin();
    let eta = y - 1. + phi.cos();
    let (rho, theta) = cartesian_to_polar(xi, eta);

    if rho <= 4. {
        let u = (1. - rho * rho / 8.).acos();
        let a = (2. * u.sin() / rho).asin();
        let t = normalize_angle(theta + PI / 2. - a);
        let v = normalize_angle(t - u - phi);

        seg_from_3([
            (t, Steering::Left, Gear::Forward),
            (u, Steering::Right, Gear::Forward),
            (v, Steering::Left, Gear::Backward),
        ])
    } else {
        None
    }
}

fn path6(x: f64, y: f64, phi: f64) -> Option<ReedsSheppSegment> {
    let xi = x + phi.sin();
    let eta = y - 1. - phi.cos();
    let (rho, theta) = cartesian_to_polar(xi, eta);

    if rho <= 4. {
        let (t, u, v);
        if rho <= 2. {
            let a = ((rho + 2.) / 4.).acos();
            t = normalize_angle(theta + PI / 2. + a);
            u = normalize_angle(a);
            v = normalize_angle(phi - t + 2. * u);
        } else {
            let a = ((rho - 2.) / 4.).acos();
            t = normalize_angle(theta + PI / 2. - a);
            u = normalize_angle(PI - a);
            v = normalize_angle(phi - t + 2. * u);
        }

        seg_from_4([
            (t, Steering::Left, Gear::Forward),
            (u, Steering::Right, Gear::Forward),
            (u, Steering::Left, Gear::Backward),
            (v, Steering::Right, Gear::Backward),
        ])
    } else {
        None
    }
}

fn path7(x: f64, y: f64, phi: f64) -> Option<ReedsSheppSegment> {
    let xi = x + phi.sin();
    let eta = y - 1. - phi.cos();
    let (rho, theta) = cartesian_to_polar(xi, eta);

    let u1 = (20. - rho * rho) / 16.;

    if rho <= 6. && (0. ..=1.).contains(&u1) {
        let u = u1.acos();
        let asin_arg = (2. * u.sin() / rho).clamp(-1.0, 1.0);
        let a = asin_arg.asin();
        let t = normalize_angle(theta + PI / 2. + a);
        let v = normalize_angle(t - phi);

        seg_from_4([
            (t, Steering::Left, Gear::Forward),
            (u, Steering::Right, Gear::Backward),
            (u, Steering::Left, Gear::Backward),
            (v, Steering::Right, Gear::Forward),
        ])
    } else {
        None
    }
}

fn path8(x: f64, y: f64, phi: f64) -> Option<ReedsSheppSegment> {
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
        let t = normalize_angle(theta + PI / 2. + a);
        let v = normalize_angle(t - phi + PI / 2.);

        seg_from_4([
            (t, Steering::Left, Gear::Forward),
            (PI / 2., Steering::Right, Gear::Backward),
            (u_param, Steering::Straight, Gear::Backward),
            (v, Steering::Left, Gear::Backward),
        ])
    } else {
        None
    }
}

fn path9(x: f64, y: f64, phi: f64) -> Option<ReedsSheppSegment> {
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
        let t = normalize_angle(theta + PI / 2. - a);
        let v = normalize_angle(t - phi - PI / 2.);

        seg_from_4([
            (t, Steering::Left, Gear::Forward),
            (u_param, Steering::Straight, Gear::Forward),
            (PI / 2., Steering::Right, Gear::Forward),
            (v, Steering::Left, Gear::Backward),
        ])
    } else {
        None
    }
}

fn path10(x: f64, y: f64, phi: f64) -> Option<ReedsSheppSegment> {
    let xi = x + phi.sin();
    let eta = y - 1. - phi.cos();
    let (rho, theta) = cartesian_to_polar(xi, eta);

    if rho >= 2. {
        let t = normalize_angle(theta + PI / 2.);
        let u = rho - 2.;
        let v = normalize_angle(phi - t - PI / 2.);

        seg_from_4([
            (t, Steering::Left, Gear::Forward),
            (PI / 2., Steering::Right, Gear::Backward),
            (u, Steering::Straight, Gear::Backward),
            (v, Steering::Right, Gear::Backward),
        ])
    } else {
        None
    }
}

fn path11(x: f64, y: f64, phi: f64) -> Option<ReedsSheppSegment> {
    let xi = x + phi.sin();
    let eta = y - 1. - phi.cos();
    let (rho, theta) = cartesian_to_polar(xi, eta);

    if rho >= 2. {
        let t = normalize_angle(theta);
        let u = rho - 2.;
        let v = normalize_angle(phi - t - PI / 2.);

        seg_from_4([
            (t, Steering::Left, Gear::Forward),
            (u, Steering::Straight, Gear::Forward),
            (PI / 2., Steering::Left, Gear::Forward),
            (v, Steering::Right, Gear::Backward),
        ])
    } else {
        None
    }
}

fn path12(x: f64, y: f64, phi: f64) -> Option<ReedsSheppSegment> {
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
        let t = normalize_angle(theta + PI / 2. + a);
        let v = normalize_angle(t - phi);

        seg_from_5([
            (t, Steering::Left, Gear::Forward),
            (PI / 2., Steering::Right, Gear::Backward),
            (u_param, Steering::Straight, Gear::Backward),
            (PI / 2., Steering::Left, Gear::Backward),
            (v, Steering::Right, Gear::Forward),
        ])
    } else {
        None
    }
}
