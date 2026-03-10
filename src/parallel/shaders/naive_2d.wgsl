
struct Uniforms {
    nb_segs: u32,
    nb_pts: u32,
    num_edges_estimated: u32,  // not for adjency matrix
    _pad: u32,
}
struct PolyPos {
    pos: vec2<f32>,
    idxg: u32,
    idxd: u32,
}
struct Seg {
    a: vec2<f32>,
    b: vec2<f32>,
}

@group(0) @binding(0) var<uniform> uniforms: Uniforms;
@group(0) @binding(1) var<storage, read> pts: array<PolyPos>;
@group(0) @binding(2) var<storage, read> segs: array<Seg>;
@group(0) @binding(3) var<storage, read_write> mat_adj: array<atomic<u32>>;
@group(0) @binding(4) var<storage, read_write> edge_list: array<vec2<u32>>;
@group(0) @binding(5) var<storage, read_write> edge_count: atomic<u32>;

fn is_in_bounds(id: vec3<u32>) -> bool {
    return id.x < uniforms.nb_pts && id.y < uniforms.nb_pts;
}

fn are_counter_clockwise(A: vec2<f32>, B: vec2<f32>, C: vec2<f32>) -> bool {
    return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x);
}

fn intersects(A: vec2<f32>, B: vec2<f32>, C: vec2<f32>, D: vec2<f32>) -> bool {
    return (are_counter_clockwise(A, C, D) != are_counter_clockwise(B, C, D)) && (are_counter_clockwise(A, B, C) != are_counter_clockwise(A, B, D));
}

fn in_cone(p: PolyPos, q: vec2<f32>) -> bool {
    let vg = pts[p.idxg];
    let vd = pts[p.idxd];
    if(are_counter_clockwise(vg.pos, p.pos, vd.pos)) {
        return are_counter_clockwise(vg.pos, p.pos, q) && are_counter_clockwise(q, p.pos, vd.pos);
    } else {
        return are_counter_clockwise(vg.pos, p.pos, q) || are_counter_clockwise(q, p.pos, vd.pos);
    }
}

fn set_mat(idx: u32, bit: u32) {
    atomicOr(&mat_adj[idx], (1u << bit));
}

@compute @workgroup_size(16, 16)
fn naive_2d_mat(
    @builtin(global_invocation_id) global_id: vec3<u32>,
) {
    if(!is_in_bounds(global_id)) {
        return;
    }
    let i = global_id.x;
    let j = global_id.y;
    let i0 = i + uniforms.nb_pts * j;
    let idx = i0 / 32;
    let bit = i0 % 32;

    if(i == j) {
        return;
    }

    let p1 = pts[i];
    let p2 = pts[j];
    if(in_cone(p1, p2.pos) || in_cone(p2, p1.pos)) {
        return;
    }

    var visible = 1u;

    for (var k = 0u; k < uniforms.nb_segs; k = k + 1u) {
        let seg = segs[k];
        if (all(seg.a == p1.pos) || all(seg.b == p1.pos) || all(seg.a == p2.pos) || all(seg.b == p2.pos)) {
            continue;
        }

        if (intersects(p1.pos, p2.pos, seg.a, seg.b)) {
            visible = 0u;
        }
    }

    if(visible == 1) {
         set_mat(idx, bit);
    }
}

@compute @workgroup_size(16, 16)
fn naive_2d_elist(
    @builtin(global_invocation_id) global_id: vec3<u32>,
) {
    if(!is_in_bounds(global_id)) {
        return;
    }
    let i = global_id.x;
    let j = global_id.y;

    if(i == j) {
        return;
    }

    let p1 = pts[i];
    let p2 = pts[j];
    if(in_cone(p1, p2.pos) || in_cone(p2, p1.pos)) {
        return;
    }

    var visible = 1u;

    for (var k = 0u; k < uniforms.nb_segs; k = k + 1u) {
        let seg = segs[k];
        if (all(seg.a == p1.pos) || all(seg.b == p1.pos) || all(seg.a == p2.pos) || all(seg.b == p2.pos)) {
            continue;
        }

        if (intersects(p1.pos, p2.pos, seg.a, seg.b)) {
            visible = 0u;
        }
    }

    if(visible == 1) {
         let index = atomicAdd(&edge_count, 1u);
         if (index < uniforms.num_edges_estimated) {
             edge_list[index] = vec2<u32>(i, j);
         }
    }
}
