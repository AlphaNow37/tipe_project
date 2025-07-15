use crate::datastructures::r_tree::{RTree, RTreeleaf};
use crate::svg::object::Style;
use crate::svg::SvgGroup;

pub fn put_rtree<T: RTreeleaf<2>>(
    svg: &mut SvgGroup,
    rtree: &RTree<2, T>,
    box_style: Style,
    height: f64,
    final_placer: Option<&impl Fn(&mut SvgGroup, &T)>,
) {
    match rtree {
        RTree::Leaf(t) => {
            svg.push(t.bounding_box(), height, box_style.clone());
            if let Some(p) = final_placer {
                p(svg, t)
            }
        }
        RTree::Node {
            bounding_box,
            children,
        } => {
            svg.push(*bounding_box, height, box_style.clone());
            for child in children {
                put_rtree(svg, child, box_style.clone(), height, final_placer);
            }
        }
    }
}
