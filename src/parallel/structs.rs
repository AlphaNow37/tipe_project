use bytemuck::{Pod, Zeroable};

#[derive(Pod, Copy, Clone, Debug, Zeroable)]
#[repr(C)]
pub struct GpuSeg {
    pub x1: f32,
    pub y1: f32,
    pub x2: f32,
    pub y2: f32,
}

#[derive(Pod, Copy, Clone, Debug, Zeroable)]
#[repr(C)]
pub struct GpuPolyPos {
    pub x: f32,
    pub y: f32,
    pub idxg: u32,
    pub idxd: u32,
}

#[derive(Pod, Copy, Clone, Debug, Zeroable)]
#[repr(C)]
pub struct GpuPolyBounds {
    pub nb_segs: u32,
    pub nb_pts: u32,
}
