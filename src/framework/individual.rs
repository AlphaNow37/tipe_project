use rand::Rng;

pub trait Individual: Clone {
    type Settings;
    fn new_random(settings: &Self::Settings, rng: &mut impl Rng) -> Self;

    fn mutate(&mut self, settings: &Self::Settings, rng: &mut impl Rng);

    fn crossover(&mut self, settings: &Self::Settings, other: &Self, rng: &mut impl Rng);
}
