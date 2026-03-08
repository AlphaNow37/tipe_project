use std::borrow::Cow;
use wgpu::util::DeviceExt;

pub struct WgpuHolder {
    pub instance: wgpu::Instance,
    pub adapter: wgpu::Adapter,
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
}

async fn get_base_holder_async() -> WgpuHolder {
    let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor::default());

    let adapter = instance
        .request_adapter(&wgpu::RequestAdapterOptions::default())
        .await
        .expect("There should be an available adapter");

    let (device, queue) = adapter
        .request_device(&wgpu::DeviceDescriptor {
            ..wgpu::DeviceDescriptor::default()
        })
        .await
        .expect("There should be a device");

    WgpuHolder {
        instance,
        adapter,
        device,
        queue,
    }
}

pub fn get_base_holder() -> WgpuHolder {
    pollster::block_on(get_base_holder_async())
}

pub fn create_uniform<T: bytemuck::Pod>(holder: &WgpuHolder, content: T) -> wgpu::Buffer {
    holder.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("Uniform Buffer"),
        contents: bytemuck::cast_slice(&[content]),
        usage: wgpu::BufferUsages::UNIFORM,
    })
}
