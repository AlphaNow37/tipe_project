use std::collections::HashSet;
use crate::geometry::VecN;
use crate::graphs::LinkGraph;

#[derive(Copy, Clone, Debug, Hash)]
pub struct TriAdjacentEdge {
    pub other_tri: Option<usize>,
    pub verts: [usize; 2]  // counterclockwise
}

#[derive(Copy, Clone, Debug, Hash)]
pub struct VertexIndidentEdge {
    pub other_vert: usize,
    pub tris: [usize; 2], // counterclockwise
}

#[derive(Debug, Clone)]
pub struct Triangulation {
    pub triangles: Vec<[TriAdjacentEdge; 3]>,
    pub vertex_poss: Vec<VecN<2, f64>>,
}

impl Triangulation {
    pub fn new(vertices: Vec<VecN<2, f64>>) -> Self {
        Self {
            triangles: Vec::new(),
            vertex_poss: vertices,
        }
    }
    pub fn triangle_vertex(&self, i: usize) -> [usize; 3] {
        self.triangles[i].map(|tae| tae.verts[0])
    }
    /// Triangle must be counterclockwise
    pub fn add_triangle(&mut self, edges: [TriAdjacentEdge; 3]) -> usize {
        let i = self.triangles.len();
        self.triangles.push(edges);
        'a: for k in 0..3 {
            if let Some(other) = edges[k].other_tri {
                let [v1, v2] = edges[k].verts;
                for k2 in 0..3 {
                    debug_assert_ne!(self.triangles[other][k2].verts, [v1, v2]);
                    if self.triangles[other][k2].verts == [v2, v1] {
                        debug_assert_eq!(self.triangles[other][k2].other_tri, None);
                        self.triangles[other][k2].other_tri = Some(i);
                        continue 'a;
                    }
                }
                panic!("Could not find the neighbors");
            }
        }
        i
    }
    pub fn localise(&self, pos: VecN<2, f64>) -> Option<usize> {
        'a: for tri_i in 0..self.triangles.len() {
            for tae in self.triangles[tri_i] {
                let pos1 = self.vertex_poss[tae.verts[0]];
                let pos2 = self.vertex_poss[tae.verts[1]];
                let prod = (pos-pos1).dot((pos2-pos1).rotate_left());
                if prod < 0. {
                    continue 'a;
                }
            }
            return Some(tri_i)
        }
        None
    }
    pub fn to_vertex_graph(&self) -> LinkGraph {
        let mut graph = LinkGraph::default();
        for tri in &self.triangles {
            for edge in tri {
                graph.add_new_link(edge.verts[0], edge.verts[1]);
            }
        }
        graph
    }
    pub fn to_triangle_graph(&self) -> LinkGraph {
        let mut graph = LinkGraph::default();
        for (i, tri) in self.triangles.iter().enumerate() {
            for edge in tri {
                if let Some(j) = edge.other_tri {
                    graph.add_new_link(i, j);
                }
            }
        }
        graph
    }
}

pub struct ConstrainedTriangulation {
    pub triangulation: Triangulation,
    pub obstacle_edge_tris: HashSet<[usize; 2]>
}

impl ConstrainedTriangulation {
    /// Build a constrained delaunay triangulation
    /// No two vertices should have the same x coordinate, and no three vertices should be aligned
    // pub fn delaynay_from_edges(vertices: Vec<VecN<2, f64>>, edges: Vec<[usize; 2]>) -> Self {
    //     delaunay::cdt_from_edges(vertices, edges)
    // }
    // pub fn delaynay_from_polygons(polygons: &[Polygon]) -> Self {
    //     let mut vertices = Vec::new();
    //     let mut edges = Vec::new();
    //     for poly in polygons {
    //         let start_i = vertices.len();
    //         for i in 0..poly.len() {
    //             vertices.push(poly.0[i]);
    //             if i+1 == poly.len() {
    //                 edges.push([start_i, start_i+i]);
    //             } else {
    //                 edges.push([start_i+i, start_i+i+1]);
    //             }
    //         }
    //     }
    //     Self::Self::delaynay_from_edges(vertices, edges)
    // }
    pub fn polyania(&self, start: VecN<2, f32>, goal: VecN<2, f32>) {
        todo!()
    }
}
