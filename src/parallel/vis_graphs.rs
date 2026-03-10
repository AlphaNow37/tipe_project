use crate::geometry::shapes::Polygon;
use crate::graphs::MapGraph;
use crate::parallel::structs::{GpuPolyBounds, GpuPolyPos, GpuSeg};
use crate::parallel::utils::{create_uniform, get_base_holder, WgpuHolder};
use wgpu::include_wgsl;
use wgpu::util::DeviceExt;

fn to_segs_polypos(obstacles: &[Polygon]) -> (Vec<GpuSeg>, Vec<GpuPolyPos>, Vec<(usize, usize)>) {
    let mut segs = Vec::new();
    let mut polypos = Vec::new();
    let mut coords = Vec::new();
    for (i, p) in obstacles.iter().enumerate() {
        for j in 0..p.len() {
            let curr_idx = coords.len();
            let idxg = if j == 0 {
                curr_idx + p.len() - 1
            } else {
                curr_idx - 1
            } as u32;
            let idxd = if j == p.len() - 1 {
                curr_idx + 1 - p.len()
            } else {
                curr_idx + 1
            } as u32;
            coords.push((i, j));
            polypos.push(GpuPolyPos {
                x: p.0[j][0] as f32,
                y: p.0[j][1] as f32,
                idxg,
                idxd,
            });
        }
        if p.len() > 1 {
            for i in 0..p.len() {
                let j = (i + 1) % p.len();
                segs.push(GpuSeg {
                    x1: p.0[i][0] as f32,
                    y1: p.0[i][1] as f32,
                    x2: p.0[j][0] as f32,
                    y2: p.0[j][1] as f32,
                })
            }
        }
    }
    (segs, polypos, coords)
}

fn get_input_buffers(
    holder: &WgpuHolder,
    polypos: &[GpuPolyPos],
    segs: &[GpuSeg],
    nb_edges_estimated: usize,
) -> (wgpu::Buffer, wgpu::Buffer, wgpu::Buffer) {
    let uniform_buffer = create_uniform(
        &holder,
        GpuPolyBounds {
            nb_pts: polypos.len() as u32,
            nb_segs: segs.len() as u32,
            nb_edges_estimated: nb_edges_estimated as u32,
            _pad: 0,
        },
    );
    let points_buffer = holder
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("PPoly points Buffer"),
            contents: bytemuck::cast_slice(&polypos),
            usage: wgpu::BufferUsages::STORAGE,
        });
    let segs_buffer = holder
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Segs points Buffer"),
            contents: bytemuck::cast_slice(&segs),
            usage: wgpu::BufferUsages::STORAGE,
        });
    (uniform_buffer, points_buffer, segs_buffer)
}

pub fn compute_vis_graph_gpu_adjacencymatrix(obstacles: &[Polygon]) -> MapGraph<(usize, usize)> {
    let holder = get_base_holder();

    let (segs, polypos, coords) = to_segs_polypos(obstacles);

    let shader = holder
        .device
        .create_shader_module(include_wgsl!("shaders/naive_2d.wgsl"));

    let (uniform_buffer, points_buffer, segs_buffer) = get_input_buffers(&holder, &polypos, &segs, 0);

    let matrix_size =
        ((polypos.len() * polypos.len() * size_of::<u32>()) as wgpu::BufferAddress / 32).next_multiple_of(4) + 4;
    let visibility_buffer = holder
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Visibility matrix Buffer"),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            contents: &vec![0; matrix_size as usize],
        });
    let staging_buffer = holder.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Staging Buffer"),
        size: matrix_size,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let compute_pipeline =
        holder
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("Compute Pipeline"),
                layout: None,
                module: &shader,
                entry_point: Some("naive_2d_mat"),
                cache: None,
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            });
    let bind_group_layout = compute_pipeline.get_bind_group_layout(0);
    let bind_group = holder.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("Bind Group"),
        layout: &bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: points_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: segs_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: visibility_buffer.as_entire_binding(),
            },
        ],
    });

    let mut encoder = holder
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
    {
        let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: None,
            timestamp_writes: None,
        });
        compute_pass.set_pipeline(&compute_pipeline);
        compute_pass.set_bind_group(0, &bind_group, &[]);

        let workgroups_x = (polypos.len() + 15) / 16;
        let workgroups_y = (polypos.len() + 15) / 16;
        compute_pass.dispatch_workgroups(workgroups_x as u32, workgroups_y as u32, 1);
    }
    encoder.copy_buffer_to_buffer(&visibility_buffer, 0, &staging_buffer, 0, matrix_size);
    holder.queue.submit(Some(encoder.finish()));

    let buffer_slice = staging_buffer.slice(..);

    let (sender, receiver) = std::sync::mpsc::channel();
    buffer_slice.map_async(wgpu::MapMode::Read, move |v| {
        sender.send(v.unwrap()).unwrap()
    });
    holder.device.poll(wgpu::PollType::Wait).unwrap();
    receiver.recv().unwrap();

    let mat_view = buffer_slice.get_mapped_range();
    let mat: &[u32] = bytemuck::cast_slice(&mat_view);
    
    let mut graph = MapGraph::default();
    for i in 0..polypos.len() {
        for j in 0..polypos.len() {
            let i0 = i + polypos.len() * j;
            let idx = i0 / 32;
            let bit = i0 % 32;
            if mat[idx] & (1 << bit) != 0 {
                graph.add_new_link(coords[i], coords[j]);
            }
        }
    }

    graph
}

pub fn compute_vis_graph_gpu_edgelist(
    obstacles: &[Polygon],
    nb_edges_estimated: usize,
) -> MapGraph<(usize, usize)> {
    let holder = get_base_holder();

    let (segs, polypos, coords) = to_segs_polypos(obstacles);

    let shader = holder
        .device
        .create_shader_module(include_wgsl!("shaders/naive_2d.wgsl"));

    let (uniform_buffer, points_buffer, segs_buffer) = get_input_buffers(&holder, &polypos, &segs, nb_edges_estimated);

    let list_max_size = (nb_edges_estimated * size_of::<u32>() * 2) as wgpu::BufferAddress;

    let visibility_buffer = holder
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Visibility list Buffer"),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            contents: &vec![0; list_max_size as usize],
        });
    let vis_staging_buffer = holder.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Staging vis Buffer"),
        size: list_max_size,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let count_buffer = holder
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Count Buffer"),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST,
            contents: &[0],
        });
    let count_staging_buffer = holder.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Staging count Buffer"),
        size: 4,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let compute_pipeline =
        holder
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("Compute Pipeline"),
                layout: None,
                module: &shader,
                entry_point: Some("naive_2d_elist"),
                cache: None,
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            });
    let bind_group_layout = compute_pipeline.get_bind_group_layout(0);
    let bind_group = holder.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("Bind Group"),
        layout: &bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: points_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: segs_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: visibility_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 5,
                resource: count_buffer.as_entire_binding(),
            }
        ],
    });

    let mut encoder = holder
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
    {
        let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: None,
            timestamp_writes: None,
        });
        compute_pass.set_pipeline(&compute_pipeline);
        compute_pass.set_bind_group(0, &bind_group, &[]);

        let workgroups_x = (polypos.len() + 15) / 16;
        let workgroups_y = (polypos.len() + 15) / 16;
        compute_pass.dispatch_workgroups(workgroups_x as u32, workgroups_y as u32, 1);
    }
    encoder.copy_buffer_to_buffer(&visibility_buffer, 0, &vis_staging_buffer, 0, list_max_size);
    encoder.copy_buffer_to_buffer(&count_buffer, 0, &count_staging_buffer, 0, 4);
    holder.queue.submit(Some(encoder.finish()));

    let buffer_staging_slice = vis_staging_buffer.slice(..);
    let buffer_count_slice = count_staging_buffer.slice(..);

    let (sender1, receiver1) = std::sync::mpsc::channel();
    let (sender2, receiver2) = std::sync::mpsc::channel();
    buffer_staging_slice.map_async(wgpu::MapMode::Read, move |v| {
        sender1.send(v.unwrap()).unwrap()
    });
    buffer_count_slice.map_async(wgpu::MapMode::Read, move |v| {
        sender2.send(v.unwrap()).unwrap()
    });

    holder.device.poll(wgpu::PollType::Wait).unwrap();
    receiver1.recv().unwrap();
    receiver2.recv().unwrap();

    let list_view = buffer_staging_slice.get_mapped_range();
    let list: &[u32] = bytemuck::cast_slice(&list_view);
    let count: u32 = bytemuck::cast_slice(&buffer_count_slice.get_mapped_range())[0];
    if count as usize >= list_max_size as usize {
        panic!("Too many edges !")
    }
    println!("Edges: {}/{}", count, nb_edges_estimated);

    let mut graph = MapGraph::default();
    for idx in 0..(count as usize) {
        let i = list[2*idx] as usize;
        let j = list[2*idx+1] as usize;
        // TODO: filter to only process i<j (*2 time improvement)
        graph.add_new_link(coords[i], coords[j]);
    }

    graph
}
