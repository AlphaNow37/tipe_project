
#[cfg(feature = "polyanya")]
pub mod l_polyanya;

#[cfg(not(feature = "polyanya"))]
pub mod l_polyanya {
    use crate::geometry::shapes::Polygon;
    use crate::geometry::VecN;
    
    pub fn shortest_path_polyanya_lib(
        obstacles: &[Polygon],
        start: VecN<2, f64>,
        goal: VecN<2, f64>,
    ) -> Option<(Vec<VecN<2, f64>>, f64)> {
        unimplemented!()
    }
}
