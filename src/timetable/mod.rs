use crate::framework::constraint::Constraint;
use crate::framework::individual::Individual;
use rand::Rng;

pub struct Settings {
    pub lengths: Vec<usize>,
    pub nb_rooms: usize,
    pub day_length: usize,
}

#[derive(Clone, Copy, Debug)]
pub struct Lesson {
    room: usize,
    start: usize,
}

#[derive(Debug, Clone)]
pub struct BasicTimeTable {
    classes: Vec<Lesson>,
}
impl Individual for BasicTimeTable {
    type Settings = Settings;
    fn new_random(settings: &Self::Settings, rng: &mut impl Rng) -> BasicTimeTable {
        Self {
            classes: settings
                .lengths
                .iter()
                .map(|len| Lesson {
                    room: rng.random_range(0..settings.nb_rooms),
                    start: rng.random_range(0..(settings.day_length.max(*len) - *len)),
                })
                .collect(),
        }
    }
    fn mutate(&mut self, settings: &Self::Settings, rng: &mut impl Rng) {
        let i = rng.random_range(0..self.classes.len());
        match rng.random_range(0..2) {
            0 => self.classes[i].start = rng.random_range(0..settings.day_length),
            1 => self.classes[i].room = rng.random_range(0..settings.nb_rooms),
            _ => unreachable!(),
        }
    }
    fn crossover(&mut self, _: &Self::Settings, other: &Self, rng: &mut impl Rng) {
        for i in 0..self.classes.len() {
            if rng.random() {
                self.classes[i] = other.classes[i];
            }
        }
    }
}

pub struct TimeLengthChecker;
impl Constraint<BasicTimeTable> for TimeLengthChecker {
    fn fitness(&mut self, ind: &BasicTimeTable, settings: &Settings) -> f64 {
        -(ind
            .classes
            .iter()
            .zip(&settings.lengths)
            .map(|(less, len)| (less.start + len).max(settings.day_length) - settings.day_length)
            .sum::<usize>() as f64)
    }
}
pub struct RoomAmountChecker;
impl Constraint<BasicTimeTable> for RoomAmountChecker {
    fn fitness(&mut self, ind: &BasicTimeTable, settings: &Settings) -> f64 {
        -(ind
            .classes
            .iter()
            .filter(|less| less.room >= settings.nb_rooms)
            .count() as f64)
    }
}
#[derive(Default)]
pub struct CollisionChecker(Vec<(usize, usize, usize)>);
impl Constraint<BasicTimeTable> for CollisionChecker {
    fn fitness(&mut self, ind: &BasicTimeTable, settings: &Settings) -> f64 {
        self.0.clear();
        self.0.extend(
            ind.classes
                .iter()
                .zip(&settings.lengths)
                .map(|(less, len)| (less.room, less.start, less.start + len)),
        );
        self.0.sort();
        let mut collision_count = 0;
        for i in 0..self.0.len() {
            let (r, _, end) = self.0[i];
            let mut i2 = i + 1;
            while i2 < self.0.len() && r == self.0[i2].0 && end > self.0[i2].1 {
                i2 += 1;
                collision_count += 1;
            }
        }
        -collision_count as f64
    }
}
