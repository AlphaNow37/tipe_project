#![feature(get_many_mut)]

use crate::framework::constraint::{FitnessAdd, FitnessMul};
use crate::framework::{run_ga, GaSettings};
use crate::timetable::{
    BasicTimeTable, CollisionChecker, RoomAmountChecker, Settings, TimeLengthChecker,
};

pub mod framework;
mod timetable;

fn main() {
    let table: BasicTimeTable = run_ga(
        &Settings {
            lengths: vec![2, 2, 2, 3, 4, 5, 6],
            nb_rooms: 6,
            day_length: 8,
        },
        GaSettings {
            population_size: 20,
            drain_size: 10,
            repeats: 5,
        },
        FitnessAdd(
            FitnessMul(10., FitnessAdd(RoomAmountChecker, TimeLengthChecker)),
            FitnessMul(1., CollisionChecker::default()),
        ),
    );
    dbg!(table);
}
