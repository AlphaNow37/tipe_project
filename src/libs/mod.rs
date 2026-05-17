#[cfg(feature = "polyanya")]
pub mod l_polyanya;

#[cfg(not(feature = "polyanya"))]
pub mod l_polyanya {
    use crate::geometry::shapes::Polygon;
    use crate::geometry::VecN;
    use crate::triangulations::triangulation::Triangulation;

    pub fn shortest_path_polyanya_lib(
        obstacles: &[Polygon],
        start: VecN<2, f64>,
        goal: VecN<2, f64>,
    ) -> Option<(Vec<VecN<2, f64>>, f64)> {
        unimplemented!()
    }

    pub fn find_path_polyanya_lib(
        start: VecN<2, f64>,
        goal: VecN<2, f64>,
        mesh: (),
    ) -> Option<(Vec<VecN<2, f64>>, f64)> {
        unimplemented!()
    }

    pub fn precompute_polyanya_lib(obstacles: &[Polygon]) -> () {
        unimplemented!()
    }
}
