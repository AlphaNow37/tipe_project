use crate::triangulations::triangulation::{TriAdjacentEdge, Triangulation};
use std::collections::BTreeSet;
use rand::prelude::SliceRandom;
use crate::geometry::shapes::are_counter_clockwise;
use crate::geometry::VecN;
use crate::utils::numbers::UsizeExt;

fn find_idx_other(t: &Triangulation, i: usize, other: usize) -> Option<usize> {
    for k in 0..3 {
        if t.triangles[i][k].other_tri == Some(other) {
            return Some(k);
        }
    }
    None
}

// fn last_pt_in_circle(vertices: [VecN<2, f64>; 4]) -> bool {
//     let [VecN([xa, ya]), VecN([xb, yb]), VecN([xc, yc]), VecN([xd, yd])] = vertices;
//     // I love geometry
//     let y0 = ((xb-xc)*(xa-xc)*(xb-xa) + (yb-yc)*(xa-xc)*(yb+yc)-(ya-yc)*(xb-xc)*(ya+yc)) / ((ya-yc)*(xb-xc) - (yb-yc)*(xa-xc)) / 2.;
//     let x0 = ((yb-yc) * (yb+yc-2.*y0) / (xb-xc) + xb + xc) / 2.;
//     let d2 = (xc - x0) * (xc-x0) + (yc-y0) * (yc-y0);
//     let dist_d = (xd - x0) * (xd-x0) + (yd-y0) * (yd-y0);
//     d2 >= dist_d


// suppose les pts counterclockwise
// Méthode de Shewchuk
fn last_pt_in_circle(vertices: [VecN<2, f64>; 4]) -> bool {
    let [VecN([xa, ya]), VecN([xb, yb]), VecN([xc, yc]), VecN([xd, yd])] = vertices;

    // Translation
    let adx = xa - xd;
    let ady = ya - yd;
    let bdx = xb - xd;
    let bdy = yb - yd;
    let cdx = xc - xd;
    let cdy = yc - yd;

    // Coord sur la Paraboloïde
    let alift = adx * adx + ady * ady;
    let blift = bdx * bdx + bdy * bdy;
    let clift = cdx * cdx + cdy * cdy;

    // On regarde si l'origine est au dessus ou au dessous de l'origine via un calcul de déterminant
    let det = alift * (bdx * cdy - cdx * bdy)
        + blift * (cdx * ady - adx * cdy)
        + clift * (adx * bdy - bdx * ady);

    det > 0.0
}

pub fn make_delaynay(t: &mut Triangulation) -> usize {
    let mut nb_changes = 0;

    let mut edges_to_update = Vec::new();
    for i in 0..t.triangles.len() {
        for k in 0..3 {
            if let Some(other) = t.triangles[i][k].other_tri {
                if i < other {
                    edges_to_update.push([i, other]);
                }
            }
        }
    }

   // dbg!(edges_to_update.len());

    let mut rng = rand::rng();
    edges_to_update.shuffle(&mut rng);

    debug_assert!(t.verify_invariants() == ());

    while let Some([v1, v2]) = edges_to_update.pop() {
        debug_assert!(t.verify_invariants() == ());
        let Some(k1) = find_idx_other(t, v1, v2) else {continue};
        let k2 = find_idx_other(t, v2, v1).expect("The other edge is there !");
        let t1 = t.triangles[v1];
        let t2 = t.triangles[v2];
        debug_assert_eq!(t1[k1].verts[0], t2[k2].verts[1]);
        debug_assert_eq!(t1[k1].verts[1], t2[k2].verts[0]);
        debug_assert_eq!(t1[k1.add_rem(1, 3)].verts[1], t1[k1.add_rem(-1, 3)].verts[0]);
        debug_assert_eq!(t2[k2.add_rem(1, 3)].verts[1], t2[k2.add_rem(-1, 3)].verts[0]);
        let vertices = [
            t1[k1].verts[0],
            t2[k2.add_rem(1, 3)].verts[1],
            t1[k1].verts[1],
            t1[k1.add_rem(1, 3)].verts[1],
        ];
        let poss = vertices.map(|i| t.vertex_poss[i]);
        debug_assert!(are_counter_clockwise(&poss));
        if last_pt_in_circle(poss) {
       //     println!("Flipping an edge !");
            nb_changes += 1;
            let new_t1 = [
                t2[k2.add_rem(-1, 3)],
                t1[k1.add_rem(1, 3)],
                TriAdjacentEdge {
                    verts: [vertices[3], vertices[1]],
                    other_tri: Some(v2),
                }
            ];
            let new_t2 = [
                t1[k1.add_rem(-1, 3)],
                t2[k2.add_rem(1, 3)],
                TriAdjacentEdge {
                    verts: [vertices[1], vertices[3]],
                    other_tri: Some(v1),
                }
            ];
            if let Some(other) = t2[k2.add_rem(-1, 3)].other_tri {
                let k3 = find_idx_other(t, other, v2).expect("There is an edge on the other side !");
                debug_assert_eq!(t.triangles[other][k3].other_tri, Some(v2));
                t.triangles[other][k3].other_tri = Some(v1);
            }
            if let Some(other) = t1[k1.add_rem(-1, 3)].other_tri {
                let k3 = find_idx_other(t, other, v1).expect("There is an edge on the other side !");
                debug_assert_eq!(t.triangles[other][k3].other_tri, Some(v1));
                t.triangles[other][k3].other_tri = Some(v2);
            }
            //dbg!(new_t1, new_t2);
            t.triangles[v1] = new_t1;
            t.triangles[v2] = new_t2;
            t.verify_invariants();
            for k in 0..3 {
                for v in [v1, v2] {
                    if let Some(other) = t.triangles[v][k].other_tri {
                        if other != v1 && other != v2 {
                            edges_to_update.push([v, other]);
                        }
                    }
                }
            }
        }
    }
    nb_changes
}

pub fn check_is_delaynay(t: &Triangulation) -> bool {
    for i in 0..t.triangles.len() {
        for k in 0..3 {
            if let Some(i2) = t.triangles[i][k].other_tri {
                let k2 = find_idx_other(t, i2, i).expect("There should be an other triangle there");
                let t1 = t.triangles[i];
                let t2 = t.triangles[i2];
                let poss = [
                    t.vertex_poss[t1[k].verts[0]],
                    t.vertex_poss[t2[k2.add_rem(1, 3)].verts[1]],
                    t.vertex_poss[t1[k].verts[1]],
                    t.vertex_poss[t1[k.add_rem(1, 3)].verts[1]],
                ];
                if last_pt_in_circle(poss) {
                    return false;
                }
            }
        }
    }
    true
}
