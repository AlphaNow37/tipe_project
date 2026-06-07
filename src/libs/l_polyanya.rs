use crate::geometry::shapes::{Cube, Polygon};
use crate::geometry::VecN;
use polyanya::geo::LineString;
use polyanya::{Mesh, Triangulation};

/// Ce fichier utilise la librairie rust polyanya pour tester les performances


fn bounding_box(obstacles: &[Polygon]) -> Cube<2> {
    let mut c = None;
    for p in obstacles {
        for pt in &p.0 {
            c = match c {
                None => Some(Cube::from_point(*pt)),
                Some(c) => Some(c.with_point(*pt)),
            }
        }
    }
    match c {
        None => Cube::default(),
        Some(c) => Cube {
            start: c.start + VecN([-50., -50.]),
            end: c.end + VecN([50., 50.]),
        },
    }
}

fn poly_to_linestring(poly: &Polygon) -> LineString<f32> {
    LineString::from_iter(poly.0.iter().copied())
}
fn cube_to_linestring(c: Cube<2>) -> LineString<f32> {
    LineString::from_iter([
        c.start,
        VecN([c.start[0], c.end[1]]),
        c.end,
        VecN([c.end[0], c.start[1]]),
    ])
}

pub fn precompute_polyanya_lib(obstacles: &[Polygon]) -> Mesh {
    let bbox = bounding_box(obstacles);
    let polygon = polyanya::geo::Polygon::new(
        cube_to_linestring(bbox),
        obstacles.iter().map(|p| poly_to_linestring(p)).collect(),
    );
    let triangulation = Triangulation::from_geo_polygon(polygon);
    let mesh = triangulation.as_navmesh();
    mesh
}

pub fn find_path_polyanya_lib(
    start: VecN<2, f64>,
    goal: VecN<2, f64>,
    mesh: Mesh,
) -> Option<(Vec<VecN<2, f64>>, f64)> {
    let path = mesh.path(start, goal);
    match path {
        None => None,
        Some(p) => Some((
            p.path
                .into_iter()
                .map(|coord| VecN(coord.to_array().map(|f| f as f64)))
                .collect(),
            p.length as f64,
        )),
    }
}

pub fn shortest_path_polyanya_lib(
    obstacles: &[Polygon],
    start: VecN<2, f64>,
    goal: VecN<2, f64>,
) -> Option<(Vec<VecN<2, f64>>, f64)> {
    let mesh = precompute_polyanya_lib(obstacles);
    find_path_polyanya_lib(start, goal, mesh)
}
