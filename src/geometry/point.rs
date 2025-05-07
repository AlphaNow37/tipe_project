use std::fmt::Debug;

#[derive(Clone, Copy)]
pub struct Point<const N: usize> {
    coords: [f32; N],
}
impl<const N: usize> Debug for Point<N> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "(")?;
        for i in 0..(N - 1) {
            write!(f, "{:.2}, ", self.coords[i])?;
        }
        write!(f, "{:.2}", self.coords[N - 1])?;
        write!(f, ")")?;
        Ok(())
    }
}
