/// Un union-find impératif
pub struct UnionFind {
    parent_height: Vec<(usize, usize)>,
}
impl UnionFind {
    pub fn new(size: usize) -> Self {
        Self {
            parent_height: (0..size).map(|i| (i, 0)).collect(),
        }
    }
    pub fn find(&mut self, i: usize) -> usize {
        let j = self.parent_height[i].0;
        if i == j {
            return i;
        }
        let parent = self.find(j);
        self.parent_height[i].0 = parent;
        parent
    }
    pub fn union(&mut self, a: usize, b: usize) {
        let p = self.find(a);
        let q = self.find(b);
        if self.parent_height[p].1 > self.parent_height[q].1 {
            self.parent_height[q].0 = p;
        } else {
            self.parent_height[p].0 = q;
            self.parent_height[q].1 = self.parent_height[q].1.max(self.parent_height[p].1 + 1);
        }
    }
    pub fn are_in_same_class(&mut self, a: usize, b: usize) -> bool {
        self.find(a) == self.find(b)
    }
}
