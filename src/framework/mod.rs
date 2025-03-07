use crate::framework::constraint::Constraint;
use crate::framework::individual::Individual;
use rand::seq::IndexedRandom;
use rand::Rng;

pub mod constraint;
pub mod fitness;
pub mod individual;
pub mod statistics;

pub struct GaSettings {
    pub nb_generations: usize,
    pub population_size: usize,
    pub drain_size: usize,
}

pub fn run_ga<Ind: Individual, C: Constraint<Ind>>(
    settings: &Ind::Settings,
    ga_settings: &GaSettings,
    constraint: &mut C,
    rng: &mut impl Rng,
) -> (Ind, f64) {
    let mut population: Vec<_> = (0..ga_settings.population_size)
        .map(|_| Ind::new_random(settings, rng))
        .collect();
    let mut best = population[0].clone();
    let mut best_fit = constraint.fitness(&best, settings);
    let mut fitnesses = vec![0.; ga_settings.population_size];
    let range = (0..ga_settings.population_size).collect::<Vec<_>>();
    for _ in 0..ga_settings.nb_generations {
        // Mutations
        for ind in population.iter_mut() {
            ind.mutate(settings, rng);
        }
        // for i in 0..ga_settings.population_size {
        //     let j = rng.random_range((i + 1)..ga_settings.population_size);
        //     let [a, b] = population.get_many_mut([i, j]).unwrap();
        //     a.crossover(settings, b, &mut rng);
        // }
        // Fitness evaluation
        for i in 0..ga_settings.population_size {
            fitnesses[i] = constraint.fitness(&population[i], settings)
        }
        let (min_fit, max_fit, best_idx) = fitnesses.iter().enumerate().fold(
            (fitnesses[0], fitnesses[0], 0),
            |(mut min, mut max, mut best_i), (i, fit)| {
                if *fit < min {
                    min = *fit;
                }
                if *fit > max {
                    max = *fit;
                    best_i = i;
                }
                (min, max, best_i)
            },
        );
        if max_fit > best_fit {
            best_fit = max_fit;
            best = population[best_idx].clone();
            assert_eq!(constraint.fitness(&best, settings), best_fit);
        }
        // Selection
        let mut to_remove: Vec<_> = range
            .choose_multiple_weighted(rng, ga_settings.drain_size, |&i| {
                (max_fit - fitnesses[i]) / (max_fit - min_fit).max(0.001) + 0.0001
            })
            .unwrap()
            .collect();
        to_remove.sort_unstable();
        for &i in to_remove.into_iter().rev() {
            population.swap_remove(i);
        }
        // Crossovers
        let rest = ga_settings.population_size - ga_settings.drain_size;
        for _ in 0..ga_settings.drain_size {
            let i = rng.random_range(0..rest);
            let j = rng.random_range(0..rest);
            let mut new = population[i].clone();
            new.crossover(settings, &population[j], rng);
            population.push(new);
        }
    }
    (best, best_fit)
}
