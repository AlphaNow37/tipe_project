use super::{individual::Individual};

pub trait Constraint<Ind: Individual> {
    fn fitness(&mut self, ind: &Ind, settings: &Ind::Settings) -> f64;
}

pub struct FitnessMul<C>(pub f64, pub C);
impl<Ind: Individual, C: Constraint<Ind>> Constraint<Ind> for FitnessMul<C> {
    fn fitness(&mut self, ind: &Ind, settings: &Ind::Settings) -> f64 {
        self.1.fitness(ind, settings) * self.0
    }
}
pub struct FitnessAdd<A, B>(pub A, pub B);
impl<
        Ind: Individual,
        A: Constraint<Ind>,
        B: Constraint<Ind>,
    > Constraint<Ind> for FitnessAdd<A, B>
{
    fn fitness(&mut self, ind: &Ind, settings: &Ind::Settings) -> f64 {
        self.0.fitness(ind, settings) + self.1.fitness(ind, settings)
    }
}
