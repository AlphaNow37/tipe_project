
pub trait ObstaclesEnv<Vertex> {
    fn contains(&self, a: Vertex, b: Vertex) -> bool;
    fn visible_small_segment(&self, a: Vertex, b: Vertex) -> bool;
    fn visible(&self, a: Vertex, b: Vertex) -> bool;
}
