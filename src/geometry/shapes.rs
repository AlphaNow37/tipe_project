use super::VecN;

#[derive(Default, Clone, Copy, Debug)]
pub struct Cube<const N: usize> {
    pub start: VecN<N, f64>,
    pub end: VecN<N, f64>,
}
impl<const N: usize> Cube<N> {
    pub fn join(self, other: Self) -> Self {
        let start = self
            .start
            .zip(other.start)
            .map_component(|(a, b)| f64::min(a, b));
        let end = self
            .end
            .zip(other.end)
            .map_component(|(a, b)| f64::max(a, b));
        Self { start, end }
    }
    pub fn with_point(self, pt: VecN<N, f64>) -> Self {
        self.join(Self { start: pt, end: pt })
    }
    pub fn from_point(pt: VecN<N, f64>) -> Self {
        Self { start: pt, end: pt }
    }
    pub fn size(self) -> VecN<N, f64> {
        self.end - self.start
    }
}

#[derive(Default, Clone, Debug)]
pub struct Polygon(pub Vec<VecN<2, f64>>);

#[derive(Default, Clone, Copy, Debug)]
pub struct Segment<const N: usize> {
    pub start: VecN<N, f64>,
    pub end: VecN<N, f64>,
}
