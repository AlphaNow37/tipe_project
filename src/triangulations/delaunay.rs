//     use std::collections::HashSet;
//     use crate::graphs::MapGraph;
//     use crate::utils::numbers::NotNanF64;
//     use super::*;
//
//     type Edge = [usize; 2];
//
//     const TOPLEFT_INF: usize = usize::MAX-1;
//     const BOTLEFT_INF: usize = usize::MAX-2;
//     const BOTRIGHT_INF: usize = usize::MAX-3;
//     const TOPRIGHT_INF: usize = usize::MAX-4;
//
//     const TOP_INF_EDGE: Edge = [TOPLEFT_INF; TOPRIGHT_INF];
//     const BOT_INF_EDGE: Edge = [BOTLEFT_INF; BOTRIGHT_INF];
//     const LEFT_INF_EDGE: Edge = [TOPLEFT_INF; BOTLEFT_INF];
//     const RIGHT_INF_EDGE: Edge = [BOTRIGHT_INF; TOPRIGHT_INF];
//
//     #[derive(Clone, Copy, Debug)]
//     struct Region {
//         bot_edge: Edge,
//         top_edge: Edge,
//         xleft: NotNanF64,
//         xright: NotNanF64,
//     }
//     impl Region {
//         fn topleft(self) -> GraphVertex {
//             GraphVertex::RegionTopLeft(FakeVertex {x: self.xleft, edge: self.top_edge})
//         }
//         fn botleft(self) -> GraphVertex {
//             GraphVertex::RegionTopLeft(FakeVertex {x: self.xleft, edge: self.bot_edge})
//         }
//         fn topright(self) -> GraphVertex {
//             GraphVertex::RegionTopLeft(FakeVertex {x: self.xright, edge: self.top_edge})
//         }
//         fn botright(self) -> GraphVertex {
//             GraphVertex::RegionTopLeft(FakeVertex {x: self.xright, edge: self.bot_edge})
//         }
//     }
//
//     #[derive(Default, Clone, Debug)]
//     struct Stripe {
//         regions: Vec<Region>,
//     }
//
//     #[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
//     struct FakeVertex {
//         edge: Edge,
//         x: NotNanF64(f64),
//     }
//
//     #[derive(Copy, Clone, Hash, Eq, PartialEq, Debug)]
//     enum GraphVertex {
//         RealVertex(usize),
//         RegionTopLeft(FakeVertex),
//         RegionTopRight(FakeVertex),
//         RegionBotLeft(FakeVertex),
//         RegionBotRight(FakeVertex),
//     }
//
//     fn graph_add_new_link(graph: &mut MapGraph<GraphVertex>, v1: GraphVertex, v2: GraphVertex) {
//         graph.add_new_link(v1, v2);
//         graph.add_new_link(v2, v1);
//     }
//
//     /// Returns an array containing, for each vertex, the next edge higher and lower (and not an edge of the vertex itself)
//     /// (higher, lower)
//     fn find_topbot_edges(sorted_indices: &[usize], positions: &[VecN<2, f64>], edges: &[Edge]) -> Vec<(Edge, Edge)> {
//         todo!()
//     }
//
//     /// Returns a list of stripes containing one region per vertex
//     /// The leftmost stripe has x=-inf, the rightmost x=+inf
//     /// Also return the initial graph
//     /// There are links between the fake vertices too
//     fn init_stripes_graph(vertices: &[VecN<2, f64>], edges: &[Edge]) -> (Vec<Stripe>, MapGraph<GraphVertex>) {
//         let n = vertices.len();
//         let mut indexes: Vec<usize> = (0..n).collect();
//         indexes.sort_by_key(|&i| NotNanF64::new(vertices[i][0]));
//         let topbot = find_topbot_edges(&indexes, vertices, edges);
//         let stripes = vec![Stripe::default(); n];
//         let mut graph = MapGraph::default();
//         for sorted_i in 0..n {
//             let idx = indexes[sorted_i];
//             let x = vertices[idx][0];
//             let xbef = if sorted_i == 0 {f64::NEG_INFINITY} else {vertices[indexes[sorted_i-1]][0]};
//             let xaft = if sorted_i == n-1 {f64::INFINITY} else {vertices[indexes[sorted_i+1]][0]};
//             let xleft = NotNanF64::new(xbef.midpoint(x));
//             let xright = NotNanF64::new(x.midpoint(xaft));
//             let (top_edge, bot_edge) = topbot[idx];
//             let region = Region {
//                 top_edge,
//                 bot_edge,
//                 xleft,
//                 xright
//             };
//
//             graph_add_new_link(&mut graph, GraphVertex::RealVertex(idx), region.botright());
//             graph_add_new_link(&mut graph, GraphVertex::RealVertex(idx), region.topright());
//             graph_add_new_link(&mut graph, GraphVertex::RealVertex(idx), region.botleft());
//             graph_add_new_link(&mut graph, GraphVertex::RealVertex(idx), region.topleft());
//             graph_add_new_link(&mut graph, topright, topleft);
//             graph_add_new_link(&mut graph, botright, botleft);
//             graph_add_new_link(&mut graph, botleft, topleft);
//             graph_add_new_link(&mut graph, topright, botright);
//
//             stripes[idx] = Stripe {
//                 regions: vec![
//                     region
//                 ]
//             };
//         }
//         for [i, j] in edges {
//             graph_add_new_link(&mut graph, GraphVertex::RealVertex(*i), GraphVertex::RealVertex(*j));
//         }
//         (stripes, graph)
//     }
//
//     /// Merges two adjacent stripes
//     fn merge_2_stripe(sleft: Stripe, sright: Stripe, graph: &mut MapGraph<GraphVertex>) -> Stripe {
//         graph.remove_node()
//
//         todo!()
//     }
//     /// Merges stripes two-by-two, creating a new stripe
//     fn merge_stripes_round(stripes: Vec<Stripe>, graph: &mut MapGraph<GraphVertex>) -> Vec<Stripe> {
//         let mut new_stripes = Vec::new();
//         let mut curr = None;
//         for s in stripes {
//             match curr {
//                 None => curr = Some(s),
//                 Some(s1) => {
//                     curr = None;
//                     new_stripes.push(merge_2_stripe(s1, s, graph))
//                 }
//             }
//         }
//         if let Some(s) = curr {
//             new_stripes.push(s)
//         }
//         new_stripes
//     }
//
//     fn graph_to_triangulation(graph: MapGraph<GraphVertex>) -> Triangulation {
//         todo!()
//     }
//
//     pub fn cdt_from_edges(vertices: Vec<VecN<2, f64>>, edges: Vec<Edge>) -> ConstrainedTriangulation {
//         let mut cdt = ConstrainedTriangulation {
//             obstacle_edge_tris: HashSet::new(),
//             triangulation: Triangulation {
//                 vertices,
//                 triangles: Vec::new(),
//             }
//         };
//
//         todo!()
//     }