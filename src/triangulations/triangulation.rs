use std::collections::HashSet;
use crate::geometry::shapes::are_counter_clockwise;
use crate::geometry::VecN;
use crate::graphs::LinkGraph;
use crate::triangulations::delaunay::make_delaynay;

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
    pub triangles: Vec<[TriAdjacentEdge; 3]>, // counterclockwise
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

    pub fn get_tri_center(&self, i: usize) -> VecN<2, f64> {
        let [a, b, c] = self.triangle_vertex(i).map(|j| self.vertex_poss[j]);
        (a+b+c)/3.
    }

    pub fn make_delaunay(&mut self) {
        make_delaynay(self);
    }

    pub fn verify_invariants(&self) {
        for (i, t) in self.triangles.iter().enumerate() {
            debug_assert_eq!(t[0].verts[1], t[1].verts[0]);
            debug_assert_eq!(t[1].verts[1], t[2].verts[0]);
            debug_assert_eq!(t[2].verts[1], t[0].verts[0]);

            let poss = [
                self.vertex_poss[t[0].verts[0]],
                self.vertex_poss[t[1].verts[0]],
                self.vertex_poss[t[2].verts[0]],
            ];
            debug_assert!(are_counter_clockwise(&poss));

            for k in 0..3 {
                if let Some(other) = t[k].other_tri {
                    let mut count = 0;
                    for k2 in 0..3 {
                        if self.triangles[other][k2].other_tri == Some(i) {
                            count += 1;
                            debug_assert_eq!(self.triangles[other][k2].verts[0], t[k].verts[1]);
                            debug_assert_eq!(self.triangles[other][k2].verts[1], t[k].verts[0]);
                        }
                    }
                    debug_assert_eq!(count, 1);
                }
                // for (j, t2) in self.triangles.iter().enumerate() {
                //     if i != j && t[k].other_tri != Some(j) {
                //         for k2 in 0..3 {
                //             debug_assert!(t2[k2].verts[0] != t[k].verts[0] || t2[k2].verts[1] != t[k].verts[0]);
                //             debug_assert!(t2[k2].verts[1] != t[k].verts[0] || t2[k2].verts[0] != t[k].verts[0]);
                //         }
                //     }
                // }
            }
        }
    }
}
