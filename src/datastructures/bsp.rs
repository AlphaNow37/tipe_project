use core::f64;

use crate::{
    geometry::{shapes::Cube, VecN},
};

#[derive(Debug, Clone)]
enum BspNode<const N: usize> {
    Leafs(Vec<VecN<N, f64>>),
    Inner(Box<[(BspNode<N>, Cube<N>); 2]>),
}
impl<const N: usize> BspNode<N> {
    /// Utility function for nearest
    fn find_nearest_from(
        &self,
        distance: &impl Fn(VecN<N, f64>) -> f64,
        distance_to_cube: &impl Fn(Cube<N>) -> f64,
        curr_min_dist: &mut f64,
        curr_nearest: &mut Option<VecN<N, f64>>,
    ) {
        match self {
            BspNode::Leafs(leafs) => {
                // Linear search
                for leaf in leafs {
                    let dist = distance(*leaf);
                    if dist < *curr_min_dist {
                        *curr_min_dist = dist;
                        *curr_nearest = Some(*leaf);
                    }
                }
            }
            BspNode::Inner(parts) => {
                // Testing on the nearest cube then the other
                let [(left_node, left_bb), (right_node, right_bb)] = &**parts;
                let left_dist = distance_to_cube(*left_bb);
                let right_dist = distance_to_cube(*right_bb);
                let sorted = if left_dist < right_dist {
                    [(left_node, left_dist), (right_node, right_dist)]
                } else {
                    [(right_node, right_dist), (left_node, left_dist)]
                };
                for (node, dist) in sorted {
                    if dist < *curr_min_dist {
                        Self::find_nearest_from(
                            node,
                            distance,
                            distance_to_cube,
                            curr_min_dist,
                            curr_nearest,
                        );
                    }
                }
            }
        }
    }

    /// Applies f on every contained vertex
    fn foreach(&self, f: &mut impl FnMut(VecN<N, f64>)) {
        match self {
            Self::Leafs(children) => children.iter().copied().for_each(f),
            Self::Inner(parts) => parts.iter().for_each(|node| node.0.foreach(f)),
        }
    }

    /// Applies f on every vertex v such that distance(v) <= radius
    pub fn foreach_r_neighborhood(
        &self,
        radius: f64,
        distance: &impl Fn(VecN<N, f64>) -> f64,
        distance_to_cube: &impl Fn(Cube<N>) -> f64,
        f: &mut impl FnMut(VecN<N, f64>),
    ) {
        match self {
            Self::Leafs(children) => children
                .iter()
                .copied()
                .filter(|pt| distance(*pt) <= radius)
                .for_each(f),
            Self::Inner(parts) => {
                for (sub_node, bbox) in &**parts {
                    let dist = distance_to_cube(*bbox);
                    if dist <= radius {
                        sub_node.foreach_r_neighborhood(radius, distance, distance_to_cube, f);
                    }
                }
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct Bsp<const N: usize> {
    node: BspNode<N>,
    bounding_box: Cube<N>,

    max_children_leaf: usize,
}
impl<const N: usize> Bsp<N> {
    pub fn new_default_config(bounding_box: Cube<N>) -> Self {
        Self {
            node: BspNode::Leafs(Vec::new()),
            bounding_box,

            max_children_leaf: 15,
        }
    }
    /// Insert a vector, assuming it wasn't already in the tree
    pub fn insert(&mut self, pt: VecN<N, f64>) {
        debug_assert!(self.bounding_box.contains_point(pt));
        let mut node = &mut self.node;
        let mut bbox = self.bounding_box;
        'a: loop {
            match node {
                BspNode::Inner(parts) => {
                    for (sub_node, sub_bbox) in parts.iter_mut() {
                        if sub_bbox.contains_point(pt) {
                            node = sub_node;
                            bbox = *sub_bbox;
                            continue 'a;
                        }
                    }
                    unreachable!()
                }
                BspNode::Leafs(children) => {
                    children.push(pt);
                    if children.len() > self.max_children_leaf {
                        let dim_to_split = bbox.size().max_component_index();

                        let mut leafs_left = Vec::new();
                        let mut leafs_right = Vec::new();

                        let mid = (bbox.start[dim_to_split] + bbox.end[dim_to_split]) / 2.;
                        let mut bbox_left = bbox;
                        let mut bbox_right = bbox;
                        bbox_left.end[dim_to_split] = mid;
                        bbox_right.start[dim_to_split] = mid;

                        for pt in children {
                            debug_assert!(bbox.contains_point(*pt), "bbox={:?}, pt={:?}", bbox, *pt);
                            if pt[dim_to_split] < mid {
                                leafs_left.push(*pt);
                            } else {
                                leafs_right.push(*pt);
                            }
                        }
                        *node = BspNode::Inner(Box::new([
                            (BspNode::Leafs(leafs_left), bbox_left),
                            (BspNode::Leafs(leafs_right), bbox_right),
                        ]));
                    }
                    return;
                }
            }
        }
    }

    /// Fetch the nearest vertex, if there is one
    pub fn nearest(
        &self,
        distance: &impl Fn(VecN<N, f64>) -> f64,
        distance_to_cube: &impl Fn(Cube<N>) -> f64,
    ) -> Option<VecN<N, f64>> {
        let mut curr_min_dist = f64::INFINITY;
        let mut curr_nearest = None;

        self.node.find_nearest_from(
            distance,
            distance_to_cube,
            &mut curr_min_dist,
            &mut curr_nearest,
        );

        curr_nearest
    }

    /// Applies f on every contained vertex
    pub fn foreach(&self, f: &mut impl FnMut(VecN<N, f64>)) {
        self.node.foreach(f);
    }

    /// Applies f on every vertex v such that distance(v) <= radius
    pub fn foreach_r_neighborhood(
        &self,
        radius: f64,
        distance: &impl Fn(VecN<N, f64>) -> f64,
        distance_to_cube: &impl Fn(Cube<N>) -> f64,
        f: &mut impl FnMut(VecN<N, f64>),
    ) {
        self.node
            .foreach_r_neighborhood(radius, distance, distance_to_cube, f);
    }

    /// Checks if pt is contained is the tree
    pub fn contains(&self, pt: VecN<N, f64>) -> bool {
        debug_assert!(self.bounding_box.contains_point(pt));
        let mut node = &self.node;
        loop {
            match node {
                BspNode::Leafs(children) => return children.contains(&pt),
                BspNode::Inner(parts) => {
                    node = parts
                        .iter()
                        .filter_map(|(sub_node, sub_box)| {
                            sub_box.contains_point(pt).then_some(sub_node)
                        })
                        .next()
                        .expect("Vertex should be in one of the sub boxes")
                }
            }
        }
    }
}

#[cfg(test)]
mod test {
    use rand::{rng, Rng};

    use crate::{
        datastructures::bsp::Bsp,
        geometry::{shapes::Cube, VecN},
    };

    #[test]
    fn test_bsp() {
        let mut bsp = Bsp::new_default_config(Cube::<3> {
            start: VecN::splat(0.),
            end: VecN::splat(1.),
        });
        let mut rng = rng();

        dbg!("testing");

        let pt1 = VecN([0.2, 0.4, 0.9]);
        bsp.insert(pt1);

        for _ in 0..100 {
            let pt = VecN([
                rng.random_range(0.0..1.0),
                rng.random_range(0.0..1.0),
                rng.random_range(0.0..1.0),
            ]);
            dbg!(pt, &bsp);
            bsp.insert(pt);
        }
    }
}
