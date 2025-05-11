pub struct UnionFind {
    parent_height: Vec<(usize, usize)>,
}
impl UnionFind {
    pub fn new(size: usize) -> Self {
        Self {
            parent_height: (0..size).map(|i| (i, 0)).collect(),
        }
    }
    pub fn find_parent(&mut self, i: usize) -> usize {
        let j = self.parent_height[i].0;
        if i == j {
            return i;
        }
        let parent = self.find_parent(j);
        self.parent_height[i].0 = parent;
        parent
    }
    pub fn merge(&mut self, a: usize, b: usize) {
        let p = self.find_parent(a);
        let q = self.find_parent(b);
        if self.parent_height[p].1 > self.parent_height[q].1 {
            self.parent_height[q].0 = p;
        } else {
            self.parent_height[p].0 = q;
            self.parent_height[q].1 = self.parent_height[q].1.max(self.parent_height[p].1 + 1);
        }
    }
    pub fn connexe(&mut self, a: usize, b: usize) -> bool {
        self.find_parent(a) == self.find_parent(b)
    }
}
