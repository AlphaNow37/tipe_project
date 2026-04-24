
#[cfg(feature = "gpu")]
mod structs;
#[cfg(feature = "gpu")]
mod vis_graphs;
#[cfg(feature = "gpu")]
mod utils;
#[cfg(feature = "gpu")]
pub use vis_graphs::*;

#[cfg(not(feature = "gpu"))]
mod inner {
    use crate::geometry::shapes::Polygon;
    use crate::graphs::MapGraph;

    pub fn compute_vis_graph_gpu_adjacencymatrix(
        obstacles: &[Polygon],
    ) -> MapGraph<(usize, usize)> {
        unimplemented!()
    }
    pub fn compute_vis_graph_gpu_edgelist(
        obstacles: &[Polygon],
        nb_edges_estimated: usize,
    ) -> MapGraph<(usize, usize)> {
        unimplemented!()
    }
}
#[cfg(not(feature = "gpu"))]
pub use inner::*;
