use std::{
    collections::{hash_map::Entry, HashMap},
    hash::Hash, sync::Mutex,
};

pub trait Cache<T, U> {
    type Item<'s> where Self: 's;
    fn get<'s>(&'s self, arg: T) -> Self::Item<'s>;
}
pub struct NoCache<F>(pub F);
impl<T, U, F: Fn(T) -> U> Cache<T, U> for NoCache<F> {
    type Item<'s> = U;
    fn get<'s>(&'s self, arg: T) -> Self::Item<'s> {
        self.0(arg)
    }
}

pub struct HashCache<F, T, U> {
    f: F,
    map: HashMap<T, U>,
}
impl<T: Hash + Eq, U, F: Fn(T) -> U> Cache<T, U> for HashCache<F, T, U> {
    type Item<'s> = &'s U where Self: 's;
    fn get<'s>(&'s mut self, arg: T) -> Self::Item<'s> {
        self.map.add;
        match self.map.entry(arg) {
            Entry::Occupied(e) => e.get(),
            Entry::Vacant(e) => e.insert((self.f)(arg))
        }
    }
}
