mod maths;
mod utils;

use std::{ffi::c_float, mem};

use maths::{
    calculate_quaternion, float3_add, float3_subtract, quat_mult, scale3, update_quat_angle,
    Float2, Float3, Float4,
};
use objc::rc::autoreleasepool;
use objc2_app_kit::{
    NSAnyEventMask, NSApp, NSApplication, NSApplicationActivationPolicy, NSBitmapImageRep,
    NSEventType, NSImage, NSWindowStyleMask,
};
use objc2_foundation::{
    MainThreadMarker, NSComparisonResult, NSData, NSDate, NSDefaultRunLoopMode,
};
use rand::{rngs::StdRng, seq::SliceRandom, thread_rng, Rng, SeedableRng};
use utils::{
    copy_to_buf, get_library, get_next_frame, initialize_window, make_buf, new_metal_layer,
    new_render_pass_descriptor, prepare_pipeline_state, set_window_layer,
};

use metal::*;

//New work to do:
//Set up some enemies in the maze, maybe just one actually.
//Give lights/mirrors some thickness for accurate collision
//Some map seems somewhat useful for multiplayer purposes,
//but also it seems like a bad idea to add navigation help to a maze game

#[repr(C)]
#[derive(Debug, Clone)]
struct Camera {
    camera_center: Float3,
    focal_length: c_float,
    rotation: Float4,
    viewport: Float2,
}

#[repr(C)]
#[derive(Debug, Clone)]
struct Uniform {
    cam: Camera,
    view_width: f32,
    view_height: f32,
    chunk_width: u32,
    time: u32,
}

#[repr(C)]
#[derive(Clone, Debug)]
struct Plane {
    origin: Float3,
    v: Float3,
    u: Float3,
    color: Float3,
}

impl Plane {
    fn new(origin: Float3, side1: Float3, side2: Float3, color: Float3) -> Self {
        Plane {
            origin,
            v: side1,
            u: side2,
            color,
        }
    }
    fn get_center(&self) -> Float3 {
        float3_add(self.origin, scale3(float3_add(self.u, self.v), 0.5))
    }
}

#[derive(Debug, Clone)]
#[repr(C)]
struct BVHNode {
    aabb_min: Float3,
    aabb_max: Float3,
    left_first: u32,
    tri_count: u32,
}
impl BVHNode {
    fn new(left_first: u32, tri_count: u32) -> Self {
        BVHNode {
            aabb_min: Float3::single(1e30f32),
            aabb_max: Float3::single(-1e30f32),
            left_first,
            tri_count,
        }
    }
    fn update_node_bounds(&mut self, planes: &Vec<Plane>, plane_indices: &Vec<u32>) {
        let mut current_aabb = aabb::default();
        for i in self.left_first..(self.left_first + self.tri_count) {
            let plane = &planes[plane_indices[i as usize] as usize];
            current_aabb.grow(plane.origin);
            current_aabb.grow(float3_add(plane.origin, plane.u));
            current_aabb.grow(float3_add(plane.origin, plane.v));
        }
        self.aabb_min = current_aabb.bmin;
        self.aabb_max = current_aabb.bmax;
    }
    fn subdivide(
        &mut self,
        planes: &Vec<Plane>,
        plane_centers: &Vec<Float3>,
        plane_indices: &mut Vec<u32>,
        nodes: &mut Vec<BVHNode>,
    ) {
        // println!("Iter count: {}", nodes_used);
        if self.tri_count == 1 {
            return;
        }
        let axis;
        let mut best_pos = 0.0;
        let mut best_cost = 1e30f32;
        let mut best_axis = 6;

        for axis in 0..=2 {
            for i in self.left_first..(self.left_first + self.tri_count) {
                let plane = &planes[plane_indices[i as usize] as usize];
                let candidate = plane.get_center()[axis];
                let cost = self.eval_sah(axis, candidate, planes, plane_indices);
                if cost <= best_cost {
                    best_cost = cost;
                    best_pos = candidate;
                    best_axis = axis;
                }
            }
        }
        let diag = float3_subtract(self.aabb_max, self.aabb_min);
        let area = diag.0 * diag.1 + diag.1 * diag.2 + diag.2 * diag.0;
        let parent_cost = self.tri_count as f32 * area;
        if best_cost > parent_cost {
            return;
        }
        axis = best_axis;
        let split_pos = best_pos;
        // println!("{:?}", self.aabb_min);
        // println!("{:?}", self.aabb_max);
        // println!("{}", self.tri_count);
        let mut i = self.left_first as i32;
        let mut j = i + self.tri_count as i32 - 1;

        while i <= j {
            let plane_center = plane_centers[plane_indices[i as usize] as usize];
            let axis_pos = plane_center[axis];
            // println!("i: {i}, j: {j}");
            // print!("Compare: {}   ", split_pos);
            // println!("{}", axis_pos);
            if axis_pos < split_pos {
                i += 1;
            } else {
                // println!("swap!");
                plane_indices.swap(i as usize, j as usize);
                j -= 1;
            }
        }
        let left_count = i as u32 - self.left_first;
        if left_count == 0 || left_count == self.tri_count {
            return;
        }
        let mut new_left_child = BVHNode::new(self.left_first, left_count);
        new_left_child.update_node_bounds(planes, &plane_indices);
        self.left_first = nodes.len() as u32;
        nodes.push(new_left_child.clone());
        let mut new_right_child = BVHNode::new(i as u32, self.tri_count - left_count);
        new_right_child.update_node_bounds(planes, &plane_indices);
        nodes.push(new_right_child.clone());

        new_left_child.subdivide(planes, plane_centers, plane_indices, nodes);
        new_right_child.subdivide(planes, plane_centers, plane_indices, nodes);
        let _ = std::mem::replace(&mut nodes[self.left_first as usize], new_left_child);
        let _ = std::mem::replace(&mut nodes[self.left_first as usize + 1], new_right_child);

        // println!("should go zero");
        self.tri_count = 0;
        // println!("{}", self.left_first);
        // println!("{}", self.tri_count);
    }
    fn eval_sah(
        &self,
        axis: usize,
        pos: f32,
        planes: &Vec<Plane>,
        plane_indices: &Vec<u32>,
    ) -> f32 {
        let mut left_box = aabb::default();
        let mut right_box = aabb::default();
        let mut left_count = 0;
        let mut right_count = 0;
        for i in self.left_first..(self.left_first + self.tri_count) {
            let plane = &planes[plane_indices[i as usize] as usize];
            if plane.get_center()[axis] < pos {
                left_count += 1;
                left_box.grow(plane.origin);
                left_box.grow(float3_add(plane.origin, plane.u));
                left_box.grow(float3_add(plane.origin, plane.v));
            } else {
                right_count += 1;
                right_box.grow(plane.origin);
                right_box.grow(float3_add(plane.origin, plane.u));
                right_box.grow(float3_add(plane.origin, plane.v));
            }
        }
        let cost = left_count as f32 * left_box.area() + right_count as f32 * right_box.area();
        if cost > 0.0 {
            return cost;
        } else {
            return 1e30f32;
        }
    }
}

#[allow(non_camel_case_types)]
struct aabb {
    bmin: Float3,
    bmax: Float3,
}

impl Default for aabb {
    fn default() -> Self {
        aabb {
            bmin: Float3::single(1e30f32),
            bmax: Float3::single(-1e30f32),
        }
    }
}
impl aabb {
    fn grow(&mut self, point: Float3) {
        self.bmin = self.bmin.fminf(point);
        self.bmax = self.bmax.fmaxf(point);
    }
    fn area(&self) -> f32 {
        let e = float3_subtract(self.bmax, self.bmin);
        e.0 * e.1 + e.1 * e.2 + e.2 * e.0
    }
    fn intersect(&self, other: aabb) -> bool {
        self.bmin.0 <= other.bmax.0
            && self.bmax.0 >= other.bmin.0
            && self.bmin.1 <= other.bmax.1
            && self.bmax.1 >= other.bmin.1
            && self.bmin.2 <= other.bmax.2
            && self.bmax.2 >= other.bmin.2
    }
}

fn build_bvh(n: usize, planes: Vec<Plane>) -> (Vec<BVHNode>, Vec<u32>) {
    let mut nodes: Vec<BVHNode> = Vec::with_capacity(2 * n - 1);
    let mut plane_centers = Vec::with_capacity(n);
    let mut plane_indices = Vec::with_capacity(n);

    for i in 0..n {
        plane_centers.push(planes[i].get_center());
        plane_indices.push(i as u32);
    }
    let mut root = BVHNode::new(0, n as u32);
    // let nodes_used = 1;
    root.update_node_bounds(&planes, &plane_indices);
    nodes.push(root.clone());
    root.subdivide(&planes, &plane_centers, &mut plane_indices, &mut nodes);
    let _ = mem::replace(&mut nodes[0], root);
    (nodes, plane_indices)
}

fn check_collision(nodes: &Vec<BVHNode>, bbox: &aabb, node_index: usize) -> Option<usize> {
    let node = &nodes[node_index];
    if node.tri_count == 1 {
        if bbox.intersect(aabb {
            bmin: node.aabb_min,
            bmax: node.aabb_max,
        }) {
            return Some(node_index);
        } else {
            return None;
        }
    }
    if bbox.intersect(aabb {
        bmin: node.aabb_min,
        bmax: node.aabb_max,
    }) {
        if let Some(hit) = check_collision(nodes, bbox, node.left_first as usize) {
            return Some(hit);
        }
        if let Some(hit) = check_collision(nodes, bbox, node.left_first as usize + 1) {
            return Some(hit);
        }
    } else {
        return None;
    }
    None
}

fn gen_pixels(view_width: f32, view_height: f32, pixel_chunk_size: u32) -> Vec<(u32, u32)> {
    let width = view_width as u32 / pixel_chunk_size;
    let height = view_height as u32 / pixel_chunk_size;
    let mut out_pixels: Vec<(u32, u32)> = Vec::with_capacity((width * height) as usize);

    for i in 0..width {
        for j in 0..height {
            out_pixels.push((pixel_chunk_size * i, pixel_chunk_size * j));
        }
    }
    let mut rng = thread_rng();
    // println!("len {}", out_pixels.len());
    out_pixels.shuffle(&mut rng);
    out_pixels
}

fn random_pixels(
    width: u64,
    height: u64,
    pixels: &mut Vec<(u32, u32)>,
    original: &Vec<(u32, u32)>,
) -> Vec<(u32, u32)> {
    let mut pixel_update_data: Vec<(u32, u32)> = Vec::with_capacity((height * width) as usize);
    // println!("{}", pixels.len());
    for _ in 0..height * width {
        if let Some(pixel) = pixels.pop() {
            pixel_update_data.push(pixel);
        } else {
            pixels.append(&mut original.clone());
            pixel_update_data.push(pixels.pop().unwrap());
        }
    }
    pixel_update_data
}

struct TreeBuilder {
    nodes: Vec<Option<usize>>,
}

impl TreeBuilder {
    fn new() -> Self {
        TreeBuilder { nodes: Vec::new() }
    }
    fn new_node(&mut self) {
        self.nodes.push(None);
    }
    fn get_root(&self, index: usize) -> usize {
        match self.nodes[index] {
            Some(parent) => self.get_root(parent),
            None => index,
        }
    }
    fn connected(&self, left: usize, right: usize) -> bool {
        self.get_root(left) == self.get_root(right)
    }
    fn connect(&mut self, parent: usize, child: usize) {
        let root = self.get_root(child);
        self.nodes[root] = Some(parent);
    }
}

const IMAGE: &[u8] = include_bytes!(".././textures/noiseTexture-2.png");

fn main() {
    let mut builder = TreeBuilder::new();
    let mut edges = Vec::new();
    let mut sets: Vec<Vec<usize>> = Vec::new();
    let mut grid: Vec<Vec<u8>> = Vec::new();

    let height = 10;
    let width = 10;
    for y in 0..height {
        sets.push(Vec::new());
        grid.push(Vec::new());
        for x in 0..width {
            if y != 0 {
                edges.push((x, y, true))
            };
            if x != 0 {
                edges.push((x, y, false))
            };
            // sets[y].push(builder.nodes.len());
            sets[y].push(builder.nodes.len());
            grid[y].push(0);
            builder.new_node();
        }
    }

    let mut rng = StdRng::seed_from_u64(0);
    edges.shuffle(&mut rng);

    for (x, y, up) in edges {
        let (nx, ny) = if up { (x, y - 1) } else { (x - 1, y) };
        if !builder.connected(sets[y][x], sets[ny][nx]) {
            builder.connect(sets[y][x], sets[ny][nx]);
            if up {
                grid[y][x] |= 1;
                grid[ny][nx] |= 2;
            } else {
                grid[y][x] |= 4;
                grid[ny][nx] |= 8;
            }
        }
    }
    let mut vert_walls = Vec::new();
    for x in 0..width {
        let mut wall_start = 0;
        let mut wall_height = 0;
        for y in 0..height {
            if x == 0 {
                wall_height += 1;
                continue;
            } else if grid[y][x] & 4 == 0 && grid[y][x - 1] & 8 == 0 {
                wall_height += 1;
            } else {
                if wall_height > 0 {
                    vert_walls.push((x as f32, wall_start as f32, wall_height as f32));
                }
                wall_height = 0;
                wall_start = y + 1;
            }
            // println!("{:?}", grid[y]);
        }
        vert_walls.push((x as f32, wall_start as f32, wall_height as f32));
    }

    let mut hori_walls = Vec::new();
    for y in 0..height {
        let mut wall_start = 0;
        let mut wall_length = 0;
        for x in 0..width {
            if y == 0 {
                wall_length += 1;
                continue;
            } else if grid[y][x] & 1 == 0 && grid[y - 1][x] & 2 == 0 {
                wall_length += 1;
            } else {
                if wall_length > 0 {
                    hori_walls.push((y as f32, wall_start as f32, wall_length as f32));
                }
                wall_length = 0;
                wall_start = x + 1;
            }
        }
        hori_walls.push((y as f32, wall_start as f32, wall_length as f32));
    }

    // println!("{vert_walls:?}");
    // println!("{hori_walls:?}");

    let mut mirrors: Vec<Plane> = Vec::new();
    let mut materials: Vec<bool> = Vec::new();
    let mut emissions: Vec<Float4> = Vec::new();

    let wall_color = Float3(0.3, 0.35, 0.4);

    for wall in vert_walls {
        mirrors.push(Plane::new(
            Float3(
                -10.0 * (height as f32 / 2.0) + (wall.0 * 10.0),
                2.0,
                -10.0 * (height as f32 / 2.0) + (wall.1 * 10.0),
            ),
            Float3(0.0, 0.0, wall.2 * 10.0),
            Float3(0.0, -10.0, 0.0),
            wall_color,
        ));
        if rng.gen::<f32>() < 0.85 {
            materials.push(false);
        } else {
            materials.push(true);
        }
        emissions.push(Float4(1.0, 0.0, 0.0, 0.0));

        if wall.2 <= 2.0 && rng.gen::<f32>() < 0.3 {
            mirrors.push(Plane::new(
                Float3(
                    -10.0 * (height as f32 / 2.0) + (wall.0 * 10.0) + 0.1,
                    2.0,
                    -10.0 * (height as f32 / 2.0) + (wall.1 * 10.0),
                ),
                Float3(0.0, 0.0, 9.9),
                Float3(0.0, -6.0, 0.0),
                wall_color,
            ));
            materials.push(false);
            emissions.push(Float4(1.0, 0.8, 0.3, 2.0));
        }
    }

    for wall in hori_walls {
        mirrors.push(Plane::new(
            Float3(
                -10.0 * (height as f32 / 2.0) + (wall.1 * 10.0),
                2.0,
                -10.0 * (height as f32 / 2.0) + (wall.0 * 10.0),
            ),
            Float3(wall.2 * 10.0, 0.0, 0.0),
            Float3(0.0, -10.0, 0.0),
            wall_color,
        ));
        if rng.gen::<f32>() < 0.90 {
            materials.push(false);
        } else {
            materials.push(true);
        }
        emissions.push(Float4(1.0, 0.0, 0.0, 0.0));

        if wall.2 <= 2.0 && rng.gen::<f32>() < 0.3 {
            mirrors.push(Plane::new(
                Float3(
                    -10.0 * (height as f32 / 2.0) + (wall.1 * 10.0),
                    2.0,
                    -10.0 * (height as f32 / 2.0) + (wall.0 * 10.0) + 0.1,
                ),
                Float3(9.9, 0.0, 0.0),
                Float3(0.0, -6.0, 0.0),
                wall_color,
            ));
            materials.push(false);
            emissions.push(Float4(1.0, 0.8, 0.3, 2.0));
        }
    }

    mirrors.push(Plane::new(
        Float3(-50.0, 2.0, -50.0),
        Float3(0.0, -20.0, 0.0),
        Float3(100.0, 0.0, 0.0),
        wall_color,
    ));
    materials.push(false);
    emissions.push(Float4(1.0, 1.0, 1.0, 0.0));
    mirrors.push(Plane::new(
        Float3(-50.0, 2.0, 50.0),
        Float3(100.0, 0.0, 0.0),
        Float3(0.0, -20.0, 0.0),
        wall_color,
    ));
    materials.push(false);
    emissions.push(Float4(1.0, 1.0, 1.0, 0.0));
    mirrors.push(Plane::new(
        Float3(-50.0, 2.0, -50.0),
        Float3(0.0, 0.0, 100.0),
        Float3(0.0, -20.0, 0.0),
        wall_color,
    ));
    materials.push(false);
    emissions.push(Float4(1.0, 1.0, 1.0, 0.0));
    mirrors.push(Plane::new(
        Float3(50.0, 2.0, -50.0),
        Float3(0.0, -20.0, 0.0),
        Float3(0.0, 0.0, 100.0),
        wall_color,
    ));
    materials.push(false);
    emissions.push(Float4(1.0, 1.0, 1.0, 0.0));
    mirrors.push(Plane::new(
        Float3(-50.0, 2.0, 50.0),
        Float3(0.0, 0.0, -100.0),
        Float3(100.0, 0.0, 0.0),
        Float3(0.4, 0.45, 0.3),
    ));
    materials.push(false);
    emissions.push(Float4(1.0, 1.0, 1.0, 0.0));
    //usually where roof goes

    mirrors.push(Plane::new(
        Float3(-5.0, 2.0, -49.9),
        Float3(10.0, 0.0, 0.0),
        Float3(0.0, -6.0, 0.0),
        Float3(0.0, 0.0, 0.0),
    ));
    materials.push(false);
    emissions.push(Float4(1.0, 0.8, 0.3, 2.0));

    // mirrors.push(Plane::new(
    //     Float3(-40.0, 2.0, -19.9),
    //     Float3(10.0, 0.0, 0.0),
    //     Float3(0.0, -6.0, 0.0),
    //     Float3(0.0, 0.0, 0.0)
    // ));
    // materials.push(false);
    // emissions.push(Float4(1.0, 0.8, 0.3, 2.0));

    //roof
    mirrors.push(Plane::new(
        Float3(-50.0, -8.0, 50.0),
        Float3(0.0, 0.0, -100.0),
        Float3(100.0, 0.0, 0.0),
        Float3(0.0, 0.0, 0.0),
    ));
    materials.push(false);
    emissions.push(Float4(1.0, 0.8, 0.3, 0.02));
    // emissions.push(Float4(1.0, 0.45, 0.15, 0.6));

    let (nodes, indices) = build_bvh(mirrors.len(), mirrors.clone());

    //Many macOS api calls are main thread only
    let mtm = MainThreadMarker::new().expect("Current thread isn't safe?");

    //Get a reference to the shared application instance object
    let app = NSApplication::sharedApplication(mtm);
    app.setActivationPolicy(NSApplicationActivationPolicy::Regular);

    let style_mask = NSWindowStyleMask::Titled.union(NSWindowStyleMask::Closable);

    let view_width: f32 = 512.0 * 2.0;
    let view_height: f32 = 384.0 * 2.0;

    let chunk_width = 4u32;
    let pixels_per_chunk = chunk_width * chunk_width;

    //Initializes an NSWindow object with a size, backing color, title, and style_mask
    let window = initialize_window(
        view_width as f64,
        view_height as f64,
        (0.0, 0.0, 0.0, 1.0),
        "Traced rays!",
        style_mask,
        mtm,
    );

    //Grab system GPU device
    let device = Device::system_default().expect("Default device not found!");

    //Grab new MetalLayer object from the GPU to draw to and set our window's content to that MetalLayer
    let layer = new_metal_layer(&device);
    set_window_layer(&window, &layer);

    //Create a queue of GPU command buffers to write to
    let command_queue = device.new_command_queue();

    //Get shader library file
    let shader_lib = get_library(&device);

    //Create a render pipeline consisting of a vertex and fragment function to give to the encoder
    let render_pipeline_state =
        prepare_pipeline_state(&device, "vertex_shader", "fragment_shader", &shader_lib);

    let compute_function = shader_lib
        .get_function("compute_shader", None)
        .expect("Could not find compute function in library");
    let compute_descriptor = ComputePipelineDescriptor::new();
    compute_descriptor.set_compute_function(Some(&compute_function));
    //let compute_pipeline_state = device.new_compute_pipeline_state_with_function(&compute_function).expect("Error creating compute pipeline");
    let compute_pipeline_state = device
        .new_compute_pipeline_state(&compute_descriptor)
        .expect("Error creating compute pipleine");
    let threadgroup_width = compute_pipeline_state.thread_execution_width();
    let threadgroup_height =
        compute_pipeline_state.max_total_threads_per_threadgroup() / threadgroup_width;
    let threads_per_threadgroup = MTLSize::new(threadgroup_width, threadgroup_height, 1);

    let threadgroups_per_grid = MTLSize::new(
        (view_width / 2.0 / pixels_per_chunk as f32) as u64,
        (view_height / 2.0 / pixels_per_chunk as f32) as u64,
        1,
    );
    println!(
        "{}",
        compute_pipeline_state.max_total_threads_per_threadgroup()
    );
    println!("{threadgroup_width}, {threadgroup_height}");
    println!(
        "{}",
        threadgroups_per_grid.height * threadgroups_per_grid.width
    );

    unsafe {
        app.finishLaunching();
        app.activate();
        window.makeKeyAndOrderFront(None);
    };

    //load noise for rng and put into metal texture
    println!("Loading noise image");
    let image_data = unsafe {
        NSData::initWithBytes_length(mtm.alloc::<NSData>(), IMAGE.as_ptr() as *mut _, IMAGE.len())
    };
    let noise_image = NSImage::initWithData(mtm.alloc::<NSImage>(), &image_data)
        .expect("Error initializing noise image");

    let noise_data = unsafe { noise_image.TIFFRepresentation().unwrap() };

    // let bitmap = unsafe { NSBitmapImageRep::init };
    let bitmap = unsafe {
        NSBitmapImageRep::initWithData(mtm.alloc::<NSBitmapImageRep>(), &noise_data).unwrap()
    };

    let mtl_resource_shared =
        MTLResourceOptions::CPUCacheModeDefaultCache | MTLResourceOptions::StorageModeShared;
    println!("Creating noise texture from image");
    let noise_width = 512;
    let noise_height = 512;
    let tex_descriptor = TextureDescriptor::new();
    tex_descriptor.set_pixel_format(MTLPixelFormat::RGBA8Unorm);
    tex_descriptor.set_storage_mode(MTLStorageMode::Shared);
    tex_descriptor.set_width(noise_width);
    tex_descriptor.set_height(noise_height);

    println!("...");

    let noise_tex = device.new_texture(&tex_descriptor);
    let region = MTLRegion::new_2d(0, 0, noise_width, noise_height);
    unsafe { noise_tex.replace_region(region, 0, bitmap.bitmapData() as *mut _, 4 * noise_width) };

    let tex_usage = MTLTextureUsage::ShaderRead.union(MTLTextureUsage::ShaderWrite);

    println!("Creating screen texture");
    // let screen_tex_width = view_width as u64 * threadgroup_width;
    // let screen_tex_height = view_height as u64 * threadgroup_height;
    let screen_tex_descriptor = TextureDescriptor::new();
    screen_tex_descriptor.set_storage_mode(MTLStorageMode::Private);
    screen_tex_descriptor.set_pixel_format(MTLPixelFormat::RGBA8Unorm);
    screen_tex_descriptor.set_usage(tex_usage);
    screen_tex_descriptor.set_width(view_width as u64);
    screen_tex_descriptor.set_height(view_height as u64);

    let screen_tex = device.new_texture(&screen_tex_descriptor);

    println!("Generating pixel addresses..");

    let mut original_pixels = gen_pixels(view_width, view_height, chunk_width);
    let mut pixels = original_pixels.clone();
    let initial_pixel_data = random_pixels(
        threadgroups_per_grid.width,
        threadgroups_per_grid.height,
        &mut pixels,
        &original_pixels,
    );
    println!("Done!");

    let pixel_update_buf = make_buf(&initial_pixel_data, None, &device);

    let mut pixel_vec: Vec<Float3> = Vec::new();
    for _ in 0..threadgroups_per_grid.width * threadgroups_per_grid.height * 16 {
        pixel_vec.push(Float3(0.0, 0.0, 0.0));
    }
    // let mut pixel_vec : Vec<Float4> = Vec::new();
    let pixel_data_buf = make_buf(&pixel_vec, Some(mtl_resource_shared), &device);

    // let quad_buf = make_buf(&quad, None, &device);

    let mirror_buf = make_buf(&mirrors, None, &device);
    let mat_buf = make_buf(&materials, None, &device);
    let emi_buf = make_buf(&emissions, None, &device);

    let node_buf = make_buf(&nodes, None, &device);
    let index_buf = make_buf(&indices, None, &device);

    let viewport_height = 2.0;
    let viewport_width = viewport_height * (view_width as f32 / view_height as f32);

    let mut camera_center = Float3(-5.0, 0.0, -45.0);
    let focal_length = 1.0;

    let player_diag = Float3(0.5, 0.2, 0.5);

    let mut quat = calculate_quaternion(&Float3(0.1, 0.0, 1.0));
    let mut half_theta = quat.3.acos();

    let init_cam: Camera = Camera {
        camera_center,
        focal_length,
        rotation: quat,
        viewport: Float2(viewport_width, viewport_height),
    };
    let mut uni = Uniform {
        view_width,
        view_height,
        cam: init_cam,
        chunk_width,
        time: 0,
    };
    // let cam_buf = make_buf(&vec![init_cam], &device);

    let _mirror_count_data = vec![mirrors.len() as u32];

    let fps = 60.0f32;
    let mut frames = 0;
    let mut frame_time = get_next_frame(fps as f64);

    let mut keys_pressed = vec![112];
    let mut rot_updated = false;

    loop {
        autoreleasepool(|| {
            // println!("{:?}", now.elapsed());
            // now = Instant::now();

            if app.windows().is_empty() {
                unsafe { app.terminate(None) };
            }
            if unsafe { frame_time.compare(&NSDate::now()) } == NSComparisonResult::Ascending {
                frame_time = get_next_frame(fps as f64);
                frames += 1;
                let pixel_data = random_pixels(
                    threadgroups_per_grid.width,
                    threadgroups_per_grid.height,
                    &mut pixels,
                    &original_pixels,
                );
                copy_to_buf(&pixel_data, &pixel_update_buf);

                let prev_cam_center = camera_center.clone();
                for key in keys_pressed.iter() {
                    match key {
                        0 => {
                            camera_center = float3_subtract(
                                camera_center,
                                quat_mult(Float3(5.0 / fps, 0.0, 0.0), quat),
                            )
                        }
                        1 => {
                            camera_center = float3_subtract(
                                camera_center,
                                quat_mult(Float3(0.0, 0.0, 5.0 / fps), quat),
                            )
                        }
                        2 => {
                            camera_center = float3_add(
                                camera_center,
                                quat_mult(Float3(5.0 / fps, 0.0, 0.0), quat),
                            )
                        }
                        13 => {
                            camera_center = float3_add(
                                camera_center,
                                quat_mult(Float3(0.0, 0.0, 5.0 / fps), quat),
                            )
                        }
                        _ => (),
                    }
                }

                if let Some(_) = check_collision(
                    &nodes,
                    &aabb {
                        bmin: float3_subtract(camera_center, player_diag),
                        bmax: float3_add(camera_center, player_diag),
                    },
                    0,
                ) {
                    camera_center = prev_cam_center;
                }

                if rot_updated {
                    let new_quat = update_quat_angle(&quat, half_theta);
                    if new_quat.0.is_nan()
                        || new_quat.1.is_nan()
                        || new_quat.2.is_nan()
                        || new_quat.3.is_nan()
                    {
                        println!("Help!");
                    } else {
                        quat = new_quat;
                        original_pixels = gen_pixels(view_width, view_height, chunk_width);
                        pixels.splice(.., original_pixels.clone());
                    }
                    rot_updated = false;
                }
                if quat.0.is_nan() || quat.1.is_nan() || quat.2.is_nan() || quat.3.is_nan() {
                    println!("Help!");
                } else {
                    let cam: Camera = Camera {
                        camera_center,
                        focal_length,
                        rotation: quat,
                        viewport: Float2(viewport_width, viewport_height),
                    };
                    uni = Uniform {
                        view_width,
                        view_height,
                        cam,
                        chunk_width,
                        time: frames,
                    };
                }
                //Get a texture from the MetalLayer that we can actually draw to
                let drawable = layer.next_drawable().expect("Unable to find drawable");
                let texture = drawable.texture();

                //Create a new command buffer and an encoder to write to it
                let command_buffer = command_queue.new_command_buffer();
                let render_pass = new_render_pass_descriptor(texture);
                let compute_encoder = command_buffer.new_compute_command_encoder();

                // println!("aaaand..");
                compute_encoder.set_compute_pipeline_state(&compute_pipeline_state);
                compute_encoder.set_buffer(0, Some(&pixel_update_buf), 0);
                compute_encoder.set_buffer(1, Some(&mirror_buf), 0);
                compute_encoder.set_buffer(2, Some(&node_buf), 0);
                compute_encoder.set_buffer(3, Some(&index_buf), 0);
                compute_encoder.set_bytes(
                    4,
                    size_of::<Uniform>() as u64,
                    vec![uni.clone()].as_ptr() as *const _,
                );
                compute_encoder.set_buffer(5, Some(&mat_buf), 0);
                compute_encoder.set_buffer(6, Some(&emi_buf), 0);
                compute_encoder.set_buffer(7, Some(&pixel_data_buf), 0);
                compute_encoder.set_texture(0, Some(&screen_tex));
                compute_encoder.set_texture(1, Some(&noise_tex));
                compute_encoder
                    .dispatch_thread_groups(threadgroups_per_grid, threads_per_threadgroup);
                compute_encoder.end_encoding();

                let encoder = command_buffer.new_render_command_encoder(render_pass);
                encoder.set_render_pipeline_state(&render_pipeline_state);
                encoder.set_fragment_texture(0, Some(&screen_tex));
                encoder.draw_primitives(MTLPrimitiveType::TriangleStrip, 0, 4);
                encoder.end_encoding();
                command_buffer.present_drawable(&drawable);
                command_buffer.commit();
            }
            loop {
                let event = unsafe {
                    NSApp(mtm).nextEventMatchingMask_untilDate_inMode_dequeue(
                        NSAnyEventMask,
                        None,
                        NSDefaultRunLoopMode,
                        true,
                    )
                };

                match event {
                    Some(ref e) => {
                        unsafe {
                            match e.r#type() {
                                NSEventType::KeyDown => {
                                    if !keys_pressed.contains(&e.keyCode()) {
                                        keys_pressed.push(e.keyCode());
                                    }
                                }
                                NSEventType::KeyUp => {
                                    if let Some(index) =
                                        keys_pressed.iter().position(|key| key == &e.keyCode())
                                    {
                                        keys_pressed.remove(index);
                                    }
                                }
                                NSEventType::MouseMoved => {
                                    half_theta = (half_theta - e.deltaX() as f32 / 512.0)
                                        .rem_euclid(std::f32::consts::PI);
                                    rot_updated = true;
                                    NSApp(mtm).sendEvent(&e);
                                    // println!("{}", half_theta);
                                }
                                _ => {
                                    NSApp(mtm).sendEvent(&e);
                                }
                            }
                        };
                    }
                    None => break,
                }
            }
        })
    }
}
