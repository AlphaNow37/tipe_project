use crate::datastructures::priority_queue::Weight;

pub trait Graph {
    type Vertex;
    fn neighbors(&self, vertex: Self::Vertex) -> impl Iterator<Item = Self::Vertex>;
    fn djkstra_with<W: Weight>(
        &self,
        start: Self::Vertex,
        end: Self::Vertex,
        cost_fn: impl Fn(Self::Vertex, Self::Vertex) -> W,
    ) {
    }
}

pub trait WeightedGraph: Graph {
    type Weight;
    fn weight_between(&self, a: Self::Vertex, b: Self::Vertex) -> Self::Weight;
}
