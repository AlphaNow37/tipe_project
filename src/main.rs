#![feature(get_many_mut)]
#![feature(sort_floats)]

use crate::framework::constraint::{FitnessAdd, FitnessMul};
use crate::framework::statistics::StatWatcher;
use crate::framework::{run_ga, GaSettings};
use crate::timetable::{CollisionChecker, RoomAmountChecker, Settings, TimeLengthChecker};
use rand::rng;
use rand::seq::IteratorRandom;

pub mod framework;
mod timetable;

fn main() {
    let stats = StatWatcher::from_experiment(1, || {
        let mut rng = rng();

        let mut constraint = FitnessAdd(
            FitnessMul(10., FitnessAdd(RoomAmountChecker, TimeLengthChecker)),
            FitnessMul(1., CollisionChecker::default()),
        );
        let ga_settings = GaSettings {
            population_size: 1000,
            drain_size: 500,
            nb_generations: 1000,
        };
        let settings = Settings {
            lengths: (0..1000).choose_multiple(&mut rng, 500),
            nb_rooms: 700,
            day_length: 1000,
        };
        let r = run_ga(&settings, &ga_settings, &mut constraint, &mut rng).1;
        r
    });
    stats.show_cli_seq();
    stats.show_cli_stats();

    // let (table, fit) = run_ga(&settings, ga_settings, &mut constraint, &mut rng);

    // dbg!(table);
}
