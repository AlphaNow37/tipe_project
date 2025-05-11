use std::array::from_fn;

use crate::{datastructures::union_find::UnionFind, geometry::shapes::Segment, utils::numbers::UsizeExt};

use super::{shapes::Polygon, VecN};
use rand::{rng, seq::SliceRandom, Rng};

pub fn gen_pol_map_luck(n: usize, map_size: f64) -> Vec<Polygon> {
    assert!(n >= 3);

    let mut rng = rng();

    let verteces = (0..n)
        .map(|_| {
            VecN([
                rng.random_range(0.0..map_size),
                rng.random_range(0.0..map_size),
            ])
        })
        .collect::<Vec<VecN<2, f64>>>();

    let mut next = (0..n).collect::<Vec<usize>>();
    next.shuffle(&mut rng);

    let mut one_changed = true;
    while one_changed {
        one_changed = false;
        for i in 0..n {
            for j in 0..n {
                if i == j || next[i] == j || next[j] == i {
                    continue;
                }
                if (Segment {
                    start: verteces[i],
                    end: verteces[next[i]],
                })
                .intersect_segment(Segment {
                    start: verteces[j],
                    end: verteces[next[j]],
                }) {
                    next.swap(i, j);
                    one_changed = true;
                }
            }
        }
    }

    let mut polygons = Vec::new();
    let mut seen = vec![false; n];
    for i in 0..n {
        if seen[i] {
            continue;
        }
        let mut pts = vec![verteces[i]];
        let mut j = next[i];
        while i != j {
            seen[j] = true;
            pts.push(verteces[j]);
            j = next[j];
        }
        if pts.len() > 2 {
            polygons.push(Polygon(pts));
        }
    }

    polygons
}

#[derive(Default)]
struct GridVertex {
    has_neigh: [bool; 4],  // T, R, B, L
    verteces: [VecN<2, f64>; 4],  // TL, TR, BR, BL
    seen_times: usize,
}

const OFFSETS: [(isize, isize); 4] = [(0, -1), (1, 0), (0, 1), (-1, 0)];

pub fn gen_pol_map_square(width: usize, map_size: f64, nmerges: usize) -> Vec<Polygon> {
    let mut rng = rng();

    let n = width * width;
    let grid_size = map_size / (width as f64);
    let sub_grid_size = grid_size / 2.;

    let mut grid = (0..width)
        .map(|y| {
            (0..width)
                .map(|x| {
                    let x_abs = (x as f64) * grid_size;
                    let y_abs = (y as f64) * grid_size;
                    GridVertex {
                        has_neigh: [false; 4],
                        verteces: [
                            VecN([
                                x_abs + rng.random_range(0.0..sub_grid_size),
                                y_abs + rng.random_range(0.0..sub_grid_size),
                            ]),
                            VecN([
                                x_abs + rng.random_range(sub_grid_size..grid_size),
                                y_abs + rng.random_range(0.0..sub_grid_size),
                            ]),
                            VecN([
                                x_abs + rng.random_range(sub_grid_size..grid_size),
                                y_abs + rng.random_range(sub_grid_size..grid_size),
                            ]),
                            VecN([
                                x_abs + rng.random_range(0.0..sub_grid_size),
                                y_abs + rng.random_range(sub_grid_size..grid_size),
                            ]),
                        ],
                        seen_times: 0,
                    }
                })
                .collect()
        })
        .collect::<Vec<Vec<_>>>();

    let mut groups = UnionFind::new(n);

    for _ in 0..nmerges {
        let i = rng.random_range(0..n);
        let x = i % width;
        let y = i / width;
        let ni = rng.random_range(0..4);
        let (offx, offy) = OFFSETS[ni];
        if grid[y][x].has_neigh[ni] {
            continue;
        }
        if x as isize + offx < 0
            || x as isize + offx >= width as isize
            || y as isize + offy < 0
            || y as isize + offy >= width as isize
        {
            continue;
        }
        let new_x = (x as isize + offx) as usize;
        let new_y = (y as isize + offy) as usize;
        let new_i = new_y * width + new_x;
        if groups.connexe(i, new_i) {
            continue;
        }
        groups.merge(i, new_i);
        grid[y][x].has_neigh[ni] = true;
        assert!(!grid[new_y][new_x].has_neigh[(ni+2)%4]);
        grid[new_y][new_x].has_neigh[(ni+2)%4] = true;
    }

    let mut polygons = Vec::new();
    for y in 0..width {
        for x in 0..width {
            if grid[y][x].seen_times != 0 {
                assert!(grid[y][x].seen_times == 4);
                continue;
            }
            let mut pts = Vec::new();
            let mut curr_dir = 0;
            let mut currx = x;
            let mut curry = y;
            loop {
                grid[curry][currx].seen_times += 1;
                assert!(grid[curry][currx].seen_times <= 4);
                pts.push(grid[curry][currx].verteces[curr_dir]);
                if grid[curry][currx].has_neigh[curr_dir] {
                    let (offx, offy) = OFFSETS[curr_dir];
                    currx = (currx as isize + offx) as usize;
                    curry = (curry as isize + offy) as usize;
                    curr_dir = curr_dir.add_rem(-1, 4);
                } else {
                    curr_dir = curr_dir.add_rem(1, 4);
                }
                if currx == x && curry == y && curr_dir == 0 {
                    break;
                }
            }

            polygons.push(Polygon(pts));
        }
    }
    polygons
}
