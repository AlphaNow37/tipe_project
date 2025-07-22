use rand::rng;

use crate::geometry::{obstacles::ObstaclesEnv, space::Space};

pub struct Parameters<'a, S: Space, O: ObstaclesEnv<S>> {
    obstacles: &'a O,
    start: S,
    goal: S,
    box_delimitation: &'a S::BoxDelimParam,
    moving_radius: f64,
}

fn rrt<S: Space>(param: Parameters<S, impl ObstaclesEnv<S>>) {
    let mut rng = rng();
    for _ in 0..1000 {
        let xrand = S::sample(param.box_delimitation, &mut rng);
        let xnear = xrand.steer_in_disc(param.start, param.moving_radius);
    }
}
